/*
 * app_cf_brushless_hover_move.c
 *
 * Standalone app-layer mission for the Crazyflie Brushless with the
 * vl53l8cx deck (9 sensors) and a flow deck: arm, take off to 0.3 m,
 * hover in place, move 0.5 m forward, hover again, then land --
 * entirely on-board, no radio commands required once triggered. This
 * is app_cf_brushless_circle.c with the circle phase replaced by
 * hover/move/hover; arming, logging (including the two-stream uSD
 * setup), and the trigger mechanisms are unchanged -- see that file's
 * header comment for more detail on those parts.
 *
 * Trigger: set param `hoverMove.start` = 1 (radio/USB), or hold a hand
 * within `hoverMove.handMm` of the "up"-facing sensor (vl53l8cx.s8) for
 * `hoverMove.handHoldMs`.
 *
 * uSD-card logging: TWO separate logical streams in one physical file,
 * decoded into two separate CSVs by sd_log_to_csv.py (it splits by
 * event name automatically):
 *   - "fixedFrequency": the 9 vl53l8cx ranges, periodic via the
 *     synchronous-stabilizer hook (see sdcard/config.txt).
 *   - "pose": position/attitude, triggered explicitly once per loop
 *     tick below via eventTrigger(&eventTrigger_pose) -- its payload is
 *     empty on purpose; the variables logged for it come from the
 *     "group.name" list under "on:pose" in config.txt, exactly like the
 *     fixedFrequency block's variable list.
 * Both streams are gated by the same `usd.logging` param, which this
 * app turns on at takeoff and off at landing.
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

#define DEBUG_MODULE "HMAPP_BL"

/* Zero-payload custom event: the "pose" stream's variable list is
 * defined entirely in config.txt (same as the fixedFrequency ranges
 * stream), not in this payload -- see file header comment. */
EVENTTRIGGER(pose)

/* ========================= Mission parameters ========================= */
/* Set to 1 to arm+fly on every boot with no radio trigger at all -- only
 * do this once you trust the mission and the flying area is clear, since
 * there would be no way to prevent it starting via the GUI. */
static uint8_t startMission = 0;

static float takeoffHeight  = 0.3f;    /* m */
static float moveDistanceM  = 0.5f;    /* m, forward move distance */
static float hoverTimeS     = 5.0f;    /* s, each hover phase */
static float moveTimeS      = 5.0f;    /* s, duration of the forward move */

/* No-PC-link trigger: hold a hand this close (mm) over the up-facing
 * vl53l8cx sensor (s8) for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: constant, gentle descent velocity -- same rationale as
 * app_cf_brushless_circle.c (a fixed-duration position ramp made
 * touchdown hard). Motors cut once estZ drops below landCutoffM, or
 * after landMaxTime_s regardless. */
static float landSpeed   = 0.15f;  /* m/s, magnitude of descent rate */
static float landCutoffM = 0.04f;  /* m, height at which motors cut */

static const float takeoffTime_s   = 3.0f;
static const float landMaxTime_s   = 6.0f;
static const uint16_t loopDt_ms    = 20;   /* 50 Hz */
static const uint16_t deckStableMs = 500;
static const uint16_t prearmMs     = 1000;
static const uint16_t armingTimeoutMs = 3000;

/* ========================= App state machine ========================= */
typedef enum {
  APP_IDLE = 0,
  APP_WAIT_FOR_DECK,
  APP_PREARM,
  APP_ARMING,
  APP_TAKEOFF,
  APP_HOVER1,
  APP_MOVE,
  APP_HOVER2,
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

static float estXLog = 0.0f;
static float estYLog = 0.0f;
static float estZLog = 0.0f;
static float estYawDegLog = 0.0f;

static float phaseElapsedSLog = 0.0f;

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
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
  logVarId_t idUp = logGetVarId("vl53l8cx", "s8");  /* up-facing sensor */

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

  /* Fixed reference points, captured once per state entry -- NOT
   * re-sampled from the live estimate every tick (see
   * app_cf_brushless_circle.c for why: it would provide no restoring
   * force during hold phases). */
  float takeoffX = 0.0f, takeoffY = 0.0f;
  float moveX = 0.0f, moveY = 0.0f;
  float landX = 0.0f, landY = 0.0f;

  DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  DEBUG_PRINT("Brushless hover/move/hover app started\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    bool armed = supervisorIsArmed();

    float estX = logGetFloat(idX);
    float estY = logGetFloat(idY);
    float estZ = logGetFloat(idZ);
    float yawDeg = logGetFloat(idYaw);
    uint8_t isTumbled = logGetUint(idIsTumbled);
    uint16_t upMm = logGetUint(idUp);

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
        (appState == APP_HOVER1) ||
        (appState == APP_MOVE) ||
        (appState == APP_HOVER2) ||
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
        phaseElapsedSLog = 0.0f;

        if (usdLoggingActive) {
          paramSetInt(idUsdLogging, 0);
          usdLoggingActive = false;
        }

        /* No-PC-link trigger: hand held over the up sensor. */
        bool handNear = (upMm > 0) && (upMm < handTriggerMm);
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
          DEBUG_PRINT("Takeoff complete -> HOVER1\n");
          appState = APP_HOVER1;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_HOVER1: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during hover 1 -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        setAbsSetpoint(&setpoint, takeoffX, takeoffY, takeoffHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= hoverTimeS) {
          DEBUG_PRINT("Hover 1 complete -> MOVE\n");
          moveX = takeoffX + moveDistanceM;
          moveY = takeoffY;
          appState = APP_MOVE;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_MOVE: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during move -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        float a = clampf(t / moveTimeS, 0.0f, 1.0f);
        float xRef = takeoffX + a * moveDistanceM;

        setAbsSetpoint(&setpoint, xRef, takeoffY, takeoffHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        phaseElapsedSLog = t;

        if (a >= 1.0f) {
          DEBUG_PRINT("Move complete -> HOVER2\n");
          appState = APP_HOVER2;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_HOVER2: {
        if (!startMission) {
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          DEBUG_PRINT("Deck lost during hover 2 -> LAND\n");
          landX = estX; landY = estY;
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        setAbsSetpoint(&setpoint, moveX, moveY, takeoffHeight, 0.0f);
        commanderSetSetpoint(&setpoint, 3);

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= hoverTimeS) {
          DEBUG_PRINT("Hover 2 complete -> LAND\n");
          landX = moveX; landY = moveY;
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

PARAM_GROUP_START(hoverMove)
PARAM_ADD(PARAM_UINT8, start, &startMission)
PARAM_ADD(PARAM_FLOAT, takeoffH, &takeoffHeight)
PARAM_ADD(PARAM_FLOAT, moveDistM, &moveDistanceM)
PARAM_ADD(PARAM_FLOAT, hoverTimeS, &hoverTimeS)
PARAM_ADD(PARAM_FLOAT, moveTimeS, &moveTimeS)
PARAM_ADD(PARAM_UINT16, handMm, &handTriggerMm)
PARAM_ADD(PARAM_UINT16, handHoldMs, &handTriggerHoldMs)
PARAM_ADD(PARAM_FLOAT, landSpeed, &landSpeed)
PARAM_ADD(PARAM_FLOAT, landCutoffM, &landCutoffM)
PARAM_GROUP_STOP(hoverMove)

LOG_GROUP_START(hoverMove)
LOG_ADD(LOG_UINT8, state, &fsmStateLog)
LOG_ADD(LOG_UINT8, isTumbled, &isTumbledLog)
LOG_ADD(LOG_UINT8, usdOn, &usdLoggingActiveLog)
LOG_ADD(LOG_UINT8, flowOk, &flowOkLog)
LOG_ADD(LOG_UINT8, armed, &armedLog)
LOG_ADD(LOG_UINT8, handNear, &handNearLog)
LOG_ADD(LOG_FLOAT, estX, &estXLog)
LOG_ADD(LOG_FLOAT, estY, &estYLog)
LOG_ADD(LOG_FLOAT, estZ, &estZLog)
LOG_ADD(LOG_FLOAT, yawDeg, &estYawDegLog)
LOG_ADD(LOG_FLOAT, phaseElapsedS, &phaseElapsedSLog)
LOG_GROUP_STOP(hoverMove)
