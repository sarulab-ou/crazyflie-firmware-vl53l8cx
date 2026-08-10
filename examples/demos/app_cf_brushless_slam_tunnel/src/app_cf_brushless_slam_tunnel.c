/*
 * app_cf_brushless_slam_tunnel.c
 *
 * Standalone app-layer mission for the Crazyflie Brushless with the
 * vl53l8cx deck (9 sensors) and a flow deck: arm, take off, then creep
 * forward down a tunnel -- only forward motion, no corner negotiation
 * or wall-following -- gated purely on whether the front fan
 * (frontleft/center/frontright) currently reports it safe to keep
 * going, with light lateral centering from the left/right beams so it
 * doesn't drift into a side wall. Lands automatically either on
 * detecting the tunnel has opened up (exit) or if progress stalls too
 * long (stuck). Same arming/takeoff/landing/logging/trigger scaffolding
 * as app_cf_brushless_hover_move.c -- see that file's header comment
 * for more detail on those parts. This is the Brushless/vl53l8cx
 * counterpart to app_cf21_slam_tunnel.c -- see that file's header for
 * the shared design rationale (why this app doesn't run SLAM itself).
 *
 * IMPORTANT -- same as app_cf21_slam_tunnel.c: no SLAM runs on the
 * Crazyflie. This app flies using only the flow deck's live (relative,
 * drifting) position estimate and the vl53l8cx's raw ranges for its
 * immediate go/no-go decision, and logs both, at rate, to the uSD card.
 * The point cloud / mapping-quality / localization-drift analysis is
 * built OFFLINE from that log -- see build_pointcloud_from_log.m in the
 * companion MATLAB project.
 *
 * Ranges must NEVER be read over a live radio LogConfig for this deck
 * (risk of a watchdog reboot, per the firmware author) -- this app only
 * ever touches them via the onboard uSD logger's own log variable
 * lookups and via logGetFloat() for its own local go/no-go decision
 * (both read the live firmware log table directly, not over radio).
 *
 * Trigger: set param `slamTunnel.start` = 1 (radio/USB), or hold a hand
 * within `slamTunnel.handMm` of the "up"-facing sensor (vl53l8cx.s8) for
 * `slamTunnel.handHoldMs`.
 *
 * uSD-card logging: TWO separate logical streams in one physical file,
 * decoded into two separate CSVs by sd_log_to_csv.py (it splits by
 * event name automatically) -- see sdcard/config.txt in this folder.
 */
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "eventtrigger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "SLAMAPP_BL"

/* Zero-payload custom event: the "pose" stream's variable list is
 * defined entirely in config.txt (same as the fixedFrequency ranges
 * stream), not in this payload -- see file header comment. */
EVENTTRIGGER(pose)

/* ========================= Mission parameters ========================= */
/* Set to 1 to arm+fly on every boot with no radio trigger at all -- only
 * do this once you trust the mission and the flying area is clear, since
 * there would be no way to prevent it starting via the GUI. */
static uint8_t startMission = 0;

static float takeoffHeight = 0.3f;   /* m */

/* Forward-advance gating: 0 forward speed at/below dStop, ramps up to
 * vCruise at/above dSlow. Front distance is min(frontleft,center,
 * frontright), the same conservative front-fan aggregate used in
 * ctrl_wall_extend_ref_mixed_opt1_sym21_vl53l8cx9.m, for consistency
 * across this project's tooling. */
static float vCruise    = 0.15f;  /* m/s, max forward speed */
static float dStop      = 0.15f;  /* m, front distance -> full stop */
static float dSlow      = 0.35f;  /* m, front distance -> starts slowing from vCruise */
static float kCenter    = 0.7f;   /* lateral centering gain */
static float yCenterMax = 0.10f;  /* m/s, centering command clamp */

/* Exit detection (tunnel opened up into non-confined space) -- same
 * openCount>=3-of-4 idea as app_cf21_slam_tunnel.c, applied to the
 * front fan (conservative min) + left/right/up. Values are floats here
 * (vl53l8cx.sN are LOG_FLOAT, in mm despite the type -- see
 * sensor_decks.py in the companion Python project), 0 means "no
 * detection", treated as open, same convention as multiranger. */
static float exitFrontMm  = 1200.0f;
static float exitSideMm   = 1000.0f;
static float exitUpMm     = 800.0f;
static float exitHoldTimeS = 0.6f;

/* Safety: land if forward progress stalls (front stays blocked) for
 * this long, or after this much total time advancing regardless -- see
 * app_cf21_slam_tunnel.c for why landing, not maneuvering, is the
 * correct response in this "mostly forward" task. */
static float stuckTimeoutS    = 10.0f;
static float maxAdvanceTimeS  = 180.0f;

/* No-PC-link trigger: hold a hand this close (mm) over the up-facing
 * vl53l8cx sensor (s8) for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: constant, gentle descent velocity -- see
 * app_cf_brushless_hover_move.c for why this replaced a fixed-duration
 * position ramp. Motors cut once estZ drops below landCutoffM, or
 * after landMaxTime_s regardless. */
static float landSpeed   = 0.15f;
static float landCutoffM = 0.04f;

static const float takeoffTime_s   = 3.0f;
static const float landMaxTime_s   = 6.0f;
static const uint16_t loopDt_ms    = 20;   /* 50 Hz */
static const uint16_t deckStableMs = 500;
static const uint16_t prearmMs     = 1000;
static const uint16_t armingTimeoutMs = 3000;
static const float R_INF = 10.0f;   /* m, stand-in for "no detection" beyond gating logic */

/* ========================= App state machine ========================= */
typedef enum {
  APP_IDLE = 0,
  APP_WAIT_FOR_DECK,
  APP_PREARM,
  APP_ARMING,
  APP_TAKEOFF,
  APP_ADVANCE,
  APP_LAND
} appState_t;

static appState_t appState = APP_IDLE;

/* ========================= Logging vars ========================= */
static uint8_t fsmStateLog = 0;
static uint8_t isTumbledLog = 0;
static uint8_t usdLoggingActiveLog = 0;
static uint8_t flowOkLog = 0;
static uint8_t armedLog = 0;
static uint8_t handNearLog = 0;
static uint8_t exitDetectedLog = 0;

static float estXLog = 0.0f;
static float estYLog = 0.0f;
static float estZLog = 0.0f;
static float estYawDegLog = 0.0f;

static float gateLog = 0.0f;          /* 0..1 forward-speed gate, for live monitoring */
static float advanceDistLog = 0.0f;   /* m, straight-line progress since takeoff */

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float localGate(float d, float d_stop, float d_slow)
{
  if (d <= d_stop) return 0.0f;
  if (d >= d_slow) return 1.0f;
  return (d - d_stop) / (d_slow - d_stop);
}

/* vl53l8cx.sN read as mm (float type, per firmware); 0 means "no
 * detection". */
static inline float mmToMOrInf(float mm, float fallback)
{
  if (mm <= 0.0f) return fallback;
  return 0.001f * mm;
}

/* Absolute position + yaw setpoint (world/estimator frame). */
static void setAbsSetpoint(setpoint_t *setpoint, float x, float y, float z, float yawDeg)
{
  memset(setpoint, 0, sizeof(setpoint_t));

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawDeg;
}

/* Body-frame forward/lateral velocity + absolute height/yaw. Used for
 * the open-ended forward advance -- see app_cf21_slam_tunnel.c. */
static void setAdvanceSetpoint(setpoint_t *setpoint, float vxBody, float vyBody, float z, float yawDeg)
{
  memset(setpoint, 0, sizeof(setpoint_t));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawDeg;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vxBody;
  setpoint->velocity.y = vyBody;
  setpoint->velocity_body = true;
}

static void stopSetpoint(setpoint_t *setpoint)
{
  memset(setpoint, 0, sizeof(setpoint_t));
}

/* Fixed x/y position, constant z velocity (negative = descending). */
static void setLandDescentSetpoint(setpoint_t *setpoint, float x, float y, float vz, float yawDeg)
{
  memset(setpoint, 0, sizeof(setpoint_t));

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->position.x = x;
  setpoint->position.y = y;

  setpoint->mode.z = modeVelocity;
  setpoint->velocity.z = vz;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawDeg;
}

void appMain(void)
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));

  /* State estimate */
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  logVarId_t idYaw = logGetVarId("stabilizer", "yaw");
  logVarId_t idIsTumbled = logGetVarId("sys", "isTumbled");

  /* vl53l8cx: frontleft(s0), center(s1), frontright(s2), left(s5),
   * right(s6), up(s8) -- see sensor_decks.py for the full index map. */
  logVarId_t idFL = logGetVarId("vl53l8cx", "s0");
  logVarId_t idC  = logGetVarId("vl53l8cx", "s1");
  logVarId_t idFR = logGetVarId("vl53l8cx", "s2");
  logVarId_t idL  = logGetVarId("vl53l8cx", "s5");
  logVarId_t idR  = logGetVarId("vl53l8cx", "s6");
  logVarId_t idUp = logGetVarId("vl53l8cx", "s8");

  /* Deck / uSD / estimator params */
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idUsdLogging = paramGetVarId("usd", "logging");
  paramVarId_t idEstimator = paramGetVarId("stabilizer", "estimator");

  uint32_t stateStartTick = xTaskGetTickCount();
  uint32_t deckSeenTick = 0;
  bool deckSeenStarted = false;
  bool usdLoggingActive = false;

  uint32_t handSeenTick = 0;
  bool handSeenStarted = false;

  uint32_t exitSeenTick = 0;
  bool exitSeenStarted = false;

  uint32_t stuckSeenTick = 0;
  bool stuckSeenStarted = false;

  /* Fixed reference points, captured once per state entry -- NOT
   * re-sampled from the live estimate every tick (see
   * app_cf_brushless_hover_move.c for why). */
  float takeoffX = 0.0f, takeoffY = 0.0f;
  float landX = 0.0f, landY = 0.0f;

  DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  DEBUG_PRINT("Brushless SLAM-tunnel app started\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    bool armed = supervisorIsArmed();

    float estX = logGetFloat(idX);
    float estY = logGetFloat(idY);
    float estZ = logGetFloat(idZ);
    float yawDeg = logGetFloat(idYaw);
    uint8_t isTumbled = logGetUint(idIsTumbled);

    float flMm = logGetFloat(idFL);
    float cMm  = logGetFloat(idC);
    float frMm = logGetFloat(idFR);
    float lMm  = logGetFloat(idL);
    float rMm  = logGetFloat(idR);
    float upMm = logGetFloat(idUp);

    estXLog = estX;
    estYLog = estY;
    estZLog = estZ;
    estYawDegLog = yawDeg;
    isTumbledLog = isTumbled;
    flowOkLog = positioningInit;
    armedLog = armed ? 1 : 0;
    fsmStateLog = (uint8_t)appState;
    usdLoggingActiveLog = usdLoggingActive ? 1 : 0;

    /* Trigger the "pose" uSD stream every tick; usddeck itself no-ops
     * this when usd.logging is off, so it's cheap to call unconditionally. */
    eventTrigger(&eventTrigger_pose);

    bool inFlightState =
        (appState == APP_ARMING) ||
        (appState == APP_TAKEOFF) ||
        (appState == APP_ADVANCE) ||
        (appState == APP_LAND);

    if (inFlightState && isTumbled) {
      DEBUG_PRINT("CRASH detected (sys.isTumbled)\n");

      if (usdLoggingActive) {
        DEBUG_PRINT("Stop uSD logging\n");
        paramSetInt(idUsdLogging, 0);
        usdLoggingActive = false;
      }

      startMission = 0;
      stopSetpoint(&setpoint);
      commanderSetSetpoint(&setpoint, 3);
      supervisorRequestArming(false);

      appState = APP_IDLE;
      stateStartTick = xTaskGetTickCount();
      continue;
    }

    switch (appState) {
      case APP_IDLE: {
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        deckSeenStarted = false;
        gateLog = 0.0f;
        advanceDistLog = 0.0f;
        exitDetectedLog = 0;

        if (usdLoggingActive) {
          paramSetInt(idUsdLogging, 0);
          usdLoggingActive = false;
        }

        /* No-PC-link trigger: hand held over the up sensor. */
        bool handNear = (upMm > 0.0f) && (upMm < (float)handTriggerMm);
        handNearLog = handNear ? 1 : 0;
        if (handNear) {
          if (!handSeenStarted) {
            handSeenStarted = true;
            handSeenTick = xTaskGetTickCount();
          } else if ((xTaskGetTickCount() - handSeenTick) > M2T(handTriggerHoldMs)) {
            DEBUG_PRINT("Hand-wave trigger -> starting mission\n");
            startMission = 1;
            handSeenStarted = false;
          }
        } else {
          handSeenStarted = false;
        }

        if (startMission) {
          DEBUG_PRINT("Trigger received\n");
          appState = APP_WAIT_FOR_DECK;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_WAIT_FOR_DECK: {
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!startMission) {
          appState = APP_IDLE;
          break;
        }

        if (positioningInit) {
          if (!deckSeenStarted) {
            deckSeenStarted = true;
            deckSeenTick = xTaskGetTickCount();
          }
          if ((xTaskGetTickCount() - deckSeenTick) > M2T(deckStableMs)) {
            DEBUG_PRINT("Deck stable -> PREARM\n");
            appState = APP_PREARM;
            stateStartTick = xTaskGetTickCount();
          }
        } else {
          deckSeenStarted = false;
        }
        break;
      }

      case APP_PREARM: {
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!startMission) {
          appState = APP_IDLE;
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost in PREARM\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if ((xTaskGetTickCount() - stateStartTick) > M2T(prearmMs)) {
          DEBUG_PRINT("PREARM done -> ARMING\n");
          appState = APP_ARMING;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_ARMING: {
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!startMission) {
          appState = APP_IDLE;
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during ARMING\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if (armed) {
          DEBUG_PRINT("Armed -> TAKEOFF\n");
          takeoffX = estX;
          takeoffY = estY;
          appState = APP_TAKEOFF;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        if ((xTaskGetTickCount() - stateStartTick) > M2T(armingTimeoutMs)) {
          DEBUG_PRINT("Arming timed out -> IDLE\n");
          startMission = 0;
          appState = APP_IDLE;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        supervisorRequestArming(true);
        break;
      }

      case APP_TAKEOFF: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during takeoff -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        if (!usdLoggingActive) {
          DEBUG_PRINT("Start uSD logging\n");
          paramSetInt(idUsdLogging, 1);
          usdLoggingActive = true;
        }

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        float a = clampf(t / takeoffTime_s, 0.0f, 1.0f);
        float zRef = a * takeoffHeight;

        setAbsSetpoint(&setpoint, takeoffX, takeoffY, zRef, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        if (a >= 1.0f) {
          DEBUG_PRINT("Takeoff complete -> ADVANCE\n");
          exitSeenStarted = false;
          stuckSeenStarted = false;
          appState = APP_ADVANCE;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_ADVANCE: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during advance -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        float rFL = mmToMOrInf(flMm, R_INF);
        float rC  = mmToMOrInf(cMm,  R_INF);
        float rFR = mmToMOrInf(frMm, R_INF);
        float rFront = fminf(rFL, fminf(rC, rFR));
        float rLeft  = mmToMOrInf(lMm, R_INF);
        float rRight = mmToMOrInf(rMm, R_INF);

        float gate = localGate(rFront, dStop, dSlow);
        float vxBody = gate * vCruise;
        float vyBody = clampf(kCenter * (rLeft - rRight), -yCenterMax, yCenterMax);

        setAdvanceSetpoint(&setpoint, vxBody, vyBody, takeoffHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        gateLog = gate;
        advanceDistLog = estX - takeoffX;

        /* -------- exit detection (tunnel opened up) -------- */
        bool frontOpen = (flMm <= 0.0f || cMm <= 0.0f || frMm <= 0.0f) ||
                         (flMm > exitFrontMm && cMm > exitFrontMm && frMm > exitFrontMm);
        bool leftOpen  = (lMm  <= 0.0f) || (lMm  > exitSideMm);
        bool rightOpen = (rMm  <= 0.0f) || (rMm  > exitSideMm);
        bool upOpen    = (upMm <= 0.0f) || (upMm > exitUpMm);
        int openCount = (frontOpen?1:0) + (leftOpen?1:0) + (rightOpen?1:0) + (upOpen?1:0);
        bool nonConfinedNow = (openCount >= 3);

        if (nonConfinedNow) {
          if (!exitSeenStarted) {
            exitSeenStarted = true;
            exitSeenTick = xTaskGetTickCount();
          }
          float held = (float)(xTaskGetTickCount() - exitSeenTick) / (float)configTICK_RATE_HZ;
          if (held >= exitHoldTimeS) {
            DEBUG_PRINT("Tunnel exit detected -> LAND\n");
            exitDetectedLog = 1;
            landX = estX; landY = estY;
            appState = APP_LAND;
            stateStartTick = xTaskGetTickCount();
            break;
          }
        } else {
          exitSeenStarted = false;
        }

        /* -------- stuck detection (no maneuvering in this task, so land) -------- */
        if (gate < 0.05f) {
          if (!stuckSeenStarted) {
            stuckSeenStarted = true;
            stuckSeenTick = xTaskGetTickCount();
          }
          float stuckHeld = (float)(xTaskGetTickCount() - stuckSeenTick) / (float)configTICK_RATE_HZ;
          if (stuckHeld >= stuckTimeoutS) {
            DEBUG_PRINT("Stuck (front blocked) -> LAND\n");
            landX = estX; landY = estY;
            appState = APP_LAND;
            stateStartTick = xTaskGetTickCount();
            break;
          }
        } else {
          stuckSeenStarted = false;
        }

        /* -------- max-time safety cap -------- */
        float advT = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        if (advT >= maxAdvanceTimeS) {
          DEBUG_PRINT("Max advance time reached -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_LAND: {
        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        bool stillDescending = (estZ > landCutoffM) && (t < landMaxTime_s);

        if (stillDescending) {
          setLandDescentSetpoint(&setpoint, landX, landY, -landSpeed, 0.0f);
          commanderSetSetpoint(&setpoint, 3);
        } else {
          stopSetpoint(&setpoint);
          commanderSetSetpoint(&setpoint, 3);
          supervisorRequestArming(false);

          DEBUG_PRINT("Landing complete -> IDLE\n");
          if (usdLoggingActive) {
            DEBUG_PRINT("Stop uSD logging\n");
            paramSetInt(idUsdLogging, 0);
            usdLoggingActive = false;
          }
          startMission = 0;
          appState = APP_IDLE;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      default:
        appState = APP_IDLE;
        break;
    }
  }
}

PARAM_GROUP_START(slamTunnel)
PARAM_ADD(PARAM_UINT8, start, &startMission)
PARAM_ADD(PARAM_FLOAT, takeoffH, &takeoffHeight)
PARAM_ADD(PARAM_FLOAT, vCruise, &vCruise)
PARAM_ADD(PARAM_FLOAT, dStop, &dStop)
PARAM_ADD(PARAM_FLOAT, dSlow, &dSlow)
PARAM_ADD(PARAM_FLOAT, kCenter, &kCenter)
PARAM_ADD(PARAM_FLOAT, yCenterMax, &yCenterMax)
PARAM_ADD(PARAM_FLOAT, exitFrontMm, &exitFrontMm)
PARAM_ADD(PARAM_FLOAT, exitSideMm, &exitSideMm)
PARAM_ADD(PARAM_FLOAT, exitUpMm, &exitUpMm)
PARAM_ADD(PARAM_FLOAT, exitHoldS, &exitHoldTimeS)
PARAM_ADD(PARAM_FLOAT, stuckTimeoutS, &stuckTimeoutS)
PARAM_ADD(PARAM_FLOAT, maxAdvanceS, &maxAdvanceTimeS)
PARAM_ADD(PARAM_UINT16, handMm, &handTriggerMm)
PARAM_ADD(PARAM_UINT16, handHoldMs, &handTriggerHoldMs)
PARAM_ADD(PARAM_FLOAT, landSpeed, &landSpeed)
PARAM_ADD(PARAM_FLOAT, landCutoffM, &landCutoffM)
PARAM_GROUP_STOP(slamTunnel)

LOG_GROUP_START(slamTunnel)
LOG_ADD(LOG_UINT8, state, &fsmStateLog)
LOG_ADD(LOG_UINT8, isTumbled, &isTumbledLog)
LOG_ADD(LOG_UINT8, usdOn, &usdLoggingActiveLog)
LOG_ADD(LOG_UINT8, flowOk, &flowOkLog)
LOG_ADD(LOG_UINT8, armed, &armedLog)
LOG_ADD(LOG_UINT8, handNear, &handNearLog)
LOG_ADD(LOG_UINT8, exitDet, &exitDetectedLog)
LOG_ADD(LOG_FLOAT, estX, &estXLog)
LOG_ADD(LOG_FLOAT, estY, &estYLog)
LOG_ADD(LOG_FLOAT, estZ, &estZLog)
LOG_ADD(LOG_FLOAT, yawDeg, &estYawDegLog)
LOG_ADD(LOG_FLOAT, gate, &gateLog)
LOG_ADD(LOG_FLOAT, advanceDist, &advanceDistLog)
LOG_GROUP_STOP(slamTunnel)
