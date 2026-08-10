/*
 * app_cf21_slam_tunnel.c
 *
 * Standalone app-layer mission for the Crazyflie 2.1 (flow deck +
 * multiranger): take off, then creep forward down a tunnel -- only
 * forward motion, no corner negotiation or wall-following -- gated
 * purely on whether the front sensor currently reports it safe to keep
 * going, with light lateral centering from the left/right beams so it
 * doesn't drift into a side wall. Lands automatically either on
 * detecting the tunnel has opened up (exit) or if progress stalls too
 * long (stuck). Same takeoff/landing/logging/trigger scaffolding as
 * app_cf21_hover_move.c -- see that file for more detail on those parts.
 *
 * IMPORTANT -- what this app does NOT do: run SLAM on the Crazyflie
 * itself. The STM32 here has nowhere near the RAM/compute for real-time
 * loop closure or map optimization. What it DOES do is exactly the
 * onboard half of a realistic pipeline: fly using only the flow deck's
 * live (relative, drifting) position estimate and the multiranger's raw
 * ranges for its immediate go/no-go decision, and log both, at rate, to
 * the uSD card for the whole flight. The actual "SLAM" -- turning that
 * logged pose+range time series into a point cloud, evaluating mapping
 * quality, comparing localization drift against ground truth -- happens
 * OFFLINE afterward (see build_pointcloud_from_log.m in the companion
 * MATLAB project). This mirrors how every other data product in this
 * project is handled: log the raw ingredients onboard, do the heavy
 * analysis on the PC.
 *
 * Trigger: either set param `slamTunnel.start` = 1 (e.g. via cfclient
 * over radio or USB), or -- with no PC link at all -- hold a hand
 * within `slamTunnel.handMm` of the multiranger's top sensor for
 * `slamTunnel.handHoldMs`. Either path clears itself automatically once
 * the mission lands or aborts. Does NOT auto-start on boot by default.
 *
 * uSD-card logging: position/attitude + multiranger ranges are recorded
 * on the onboard microSD card for the whole flight, via the `usd.logging`
 * param -- this app turns it on right as takeoff begins and off once
 * landed. Requires a config.txt on the card listing those variables with
 * "enable on startup" set to 0 -- see sdcard/config.txt in this folder.
 */
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "SLAMAPP"

/* ========================= Mission parameters ========================= */
/* Set to 1 to arm+fly on every boot with no radio trigger at all -- only
 * do this once you trust the mission and the flying area is clear, since
 * there would be no way to prevent it starting via the GUI. */
static uint8_t startMission = 0;

static float takeoffHeight = 0.5f;   /* m */

/* Forward-advance gating: 0 forward speed at/below dStop, ramps up to
 * vCruise at/above dSlow. Same "local_gate" shape as the bug-tunnel
 * reference app in this project. */
static float vCruise    = 0.15f;  /* m/s, max forward speed */
static float dStop      = 0.15f;  /* m, front distance -> full stop */
static float dSlow      = 0.35f;  /* m, front distance -> starts slowing from vCruise */
static float kCenter    = 0.7f;   /* lateral centering gain */
static float yCenterMax = 0.10f;  /* m/s, centering command clamp */

/* Exit detection (tunnel opened up into non-confined space) -- same
 * openCount>=3-of-4 idea as app_tunnel_hover.c in this project, applied
 * to multiranger's front/left/right/up. mm values of 0 mean "no
 * detection" (out of range / no return), treated as open, same
 * convention as that reference app. */
static uint16_t exitFrontMm  = 1200;
static uint16_t exitSideMm   = 1000;
static uint16_t exitUpMm     = 800;
static float    exitHoldTimeS = 0.6f;

/* Safety: land if forward progress stalls (front stays blocked) for
 * this long, or after this much total time advancing regardless --
 * this is a "mostly forward, no maneuvering" task, so getting properly
 * stuck means something's wrong (dead end, or a bad reading) and
 * landing is the safe default rather than trying to route around it. */
static float stuckTimeoutS    = 10.0f;
static float maxAdvanceTimeS  = 180.0f;

/* No-PC-link trigger: hold a hand this close (mm) over the top
 * multiranger sensor for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: descend at a constant, gentle velocity -- see
 * app_cf21_hover_move.c for why this replaced a fixed-duration position
 * ramp. Motors cut once estZ drops below landCutoffM, or after
 * landMaxTime_s regardless. */
static float landSpeed   = 0.15f;
static float landCutoffM = 0.04f;

static const float takeoffTime_s   = 3.0f;
static const float landMaxTime_s   = 6.0f;
static const uint16_t loopDt_ms    = 20;   /* 50 Hz */
static const uint16_t deckStableMs = 500;
static const uint16_t prearmMs     = 1000;
static const float R_INF = 10.0f;   /* m, stand-in for "no detection" beyond gating logic */

/* ========================= App state machine ========================= */
typedef enum {
  APP_IDLE = 0,
  APP_WAIT_FOR_DECK,
  APP_PREARM,
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
static uint8_t mrOkLog = 0;
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

static inline float mmToMOrInf(uint16_t mm, float fallback)
{
  if (mm == 0) return fallback;
  return 0.001f * (float)mm;
}

/* Absolute position + yaw setpoint (world/estimator frame). Used for
 * takeoff and the fixed-point hold states. */
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
 * the open-ended forward advance, where there's no fixed target
 * position to command -- only a direction and a rate. */
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

  /* Multiranger */
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idLeft  = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idUp    = logGetVarId("range", "up");

  /* Deck / uSD / estimator params */
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");
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
   * app_cf21_hover_move.c for why: it would provide no restoring force
   * during hold phases). */
  float takeoffX = 0.0f, takeoffY = 0.0f;
  float landX = 0.0f, landY = 0.0f;

  DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  DEBUG_PRINT("CF2.1 SLAM-tunnel app started\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    float estX = logGetFloat(idX);
    float estY = logGetFloat(idY);
    float estZ = logGetFloat(idZ);
    float yawDeg = logGetFloat(idYaw);
    uint8_t isTumbled = logGetUint(idIsTumbled);

    uint16_t frontMm = logGetUint(idFront);
    uint16_t leftMm  = logGetUint(idLeft);
    uint16_t rightMm = logGetUint(idRight);
    uint16_t upMm    = logGetUint(idUp);

    estXLog = estX;
    estYLog = estY;
    estZLog = estZ;
    estYawDegLog = yawDeg;
    isTumbledLog = isTumbled;
    flowOkLog = positioningInit;
    mrOkLog = multirangerInit;
    fsmStateLog = (uint8_t)appState;
    usdLoggingActiveLog = usdLoggingActive ? 1 : 0;

    bool inFlightState =
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

        /* No-PC-link trigger: hand held over the top sensor for
         * handTriggerHoldMs sets startMission the same as the radio/USB
         * param would. mrOkLog gates this so it only runs once the
         * multiranger deck is actually present and reporting. */
        bool handNear = mrOkLog && (upMm > 0) && (upMm < handTriggerMm);
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

        if (positioningInit && multirangerInit) {
          if (!deckSeenStarted) {
            deckSeenStarted = true;
            deckSeenTick = xTaskGetTickCount();
          }
          if ((xTaskGetTickCount() - deckSeenTick) > M2T(deckStableMs)) {
            DEBUG_PRINT("Decks stable -> PREARM\n");
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
        if (!(positioningInit && multirangerInit)) {
          DEBUG_PRINT("Deck lost in PREARM\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if ((xTaskGetTickCount() - stateStartTick) > M2T(prearmMs)) {
          DEBUG_PRINT("PREARM done -> TAKEOFF\n");
          takeoffX = estX;
          takeoffY = estY;
          appState = APP_TAKEOFF;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_TAKEOFF: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!(positioningInit && multirangerInit)) {
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
        if (!(positioningInit && multirangerInit)) {
          DEBUG_PRINT("Deck lost during advance -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        float rFront = mmToMOrInf(frontMm, R_INF);
        float rLeft  = mmToMOrInf(leftMm,  R_INF);
        float rRight = mmToMOrInf(rightMm, R_INF);

        float gate = localGate(rFront, dStop, dSlow);
        float vxBody = gate * vCruise;
        float vyBody = clampf(kCenter * (rLeft - rRight), -yCenterMax, yCenterMax);

        setAdvanceSetpoint(&setpoint, vxBody, vyBody, takeoffHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        gateLog = gate;
        advanceDistLog = estX - takeoffX;

        /* -------- exit detection (tunnel opened up) -------- */
        bool frontOpen = (frontMm == 0) || (frontMm > exitFrontMm);
        bool leftOpen  = (leftMm  == 0) || (leftMm  > exitSideMm);
        bool rightOpen = (rightMm == 0) || (rightMm > exitSideMm);
        bool upOpen    = (upMm    == 0) || (upMm    > exitUpMm);
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
PARAM_ADD(PARAM_UINT16, exitFrontMm, &exitFrontMm)
PARAM_ADD(PARAM_UINT16, exitSideMm, &exitSideMm)
PARAM_ADD(PARAM_UINT16, exitUpMm, &exitUpMm)
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
LOG_ADD(LOG_UINT8, mrOk, &mrOkLog)
LOG_ADD(LOG_UINT8, handNear, &handNearLog)
LOG_ADD(LOG_UINT8, exitDet, &exitDetectedLog)
LOG_ADD(LOG_FLOAT, estX, &estXLog)
LOG_ADD(LOG_FLOAT, estY, &estYLog)
LOG_ADD(LOG_FLOAT, estZ, &estZLog)
LOG_ADD(LOG_FLOAT, yawDeg, &estYawDegLog)
LOG_ADD(LOG_FLOAT, gate, &gateLog)
LOG_ADD(LOG_FLOAT, advanceDist, &advanceDistLog)
LOG_GROUP_STOP(slamTunnel)
