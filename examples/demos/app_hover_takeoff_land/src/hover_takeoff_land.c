/*
 * hover_takeoff_land.c
 *
 * Minimal on-board mission for the Crazyflie 2.1 Brushless with a flow
 * deck: arm all motors, idle for `armDwell` seconds, take off to
 * `height`, hover for `hoverTime`, then land -- no radio commands needed
 * once triggered.
 *
 * Brushless notes:
 *   - Brushless motors will not spin until the supervisor *arms* the
 *     system (CONFIG_MOTORS_REQUIRE_ARMING). This app requests arming and
 *     waits until it succeeds.
 *   - After arming, all propellers spin at idle. We deliberately hold this
 *     idle state for `armDwell` seconds (default 3 s) before commanding
 *     any thrust, so the ESCs are fully spun-up and stable before takeoff.
 *   - We drive the flight with commanderSetSetpoint() absolute-position
 *     setpoints (NOT the high-level commander), which is the pattern that
 *     flies reliably on brushless in this repo (see
 *     app_cf_brushless_hover_move.c).
 *
 * Trigger: set param `hoverapp.enable` = 1 (radio/USB). Requires the
 * flow/Z deck (deck.bcFlow2) for position hold.
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "commander.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "HTLAPP"

/* ========================= Mission parameters ========================= */
/* Auto-start: the mission arms and flies on its own as soon as the flow
 * deck is present -- no radio trigger needed. Set to 0 (radio/USB) at any
 * time as an emergency abort: the app will land and disarm. */
static uint8_t enable = 1;

static float takeoffHeight = 0.5f;  /* m */
static float takeoffTime   = 2.0f;  /* s, takeoff ramp duration */
static float hoverTime     = 3.0f;  /* s */
static float landTime      = 2.0f;  /* s, landing ramp duration */
/* Seconds to keep every propeller spinning at idle (armed, zero thrust)
 * before takeoff. */
static float armDwell      = 3.0f;

/* Max time to wait for the supervisor to accept arming (ms). Arming only
 * succeeds once pre-flight checks pass. Must be < supervisor landedTimeout
 * so a stalled arm doesn't get auto-disarmed silently. */
static const uint16_t armingTimeoutMs = 3000;
static const uint16_t loopDt_ms = 20;   /* 50 Hz control loop */

/* Landing: gentle constant descent, then cut motors near the ground. */
static float landSpeed   = 0.15f;   /* m/s descent */
static float landCutoffM = 0.05f;   /* m, height at which motors cut */

/* ========================= App state machine ========================= */
typedef enum {
  STATE_IDLE = 0,
  STATE_ARMING,
  STATE_ARM_DWELL,
  STATE_TAKEOFF,
  STATE_HOVER,
  STATE_LAND,
} flightState_e;

static uint8_t flightState = STATE_IDLE;
static uint8_t reachedHeight = 0;

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* Absolute position setpoint (world/estimator frame), yaw held at 0. */
static void setAbsSetpoint(setpoint_t *sp, float x, float y, float z)
{
  memset(sp, 0, sizeof(setpoint_t));
  sp->mode.x = modeAbs;
  sp->mode.y = modeAbs;
  sp->mode.z = modeAbs;
  sp->position.x = x;
  sp->position.y = y;
  sp->position.z = z;
  sp->mode.yaw = modeAbs;
  sp->attitude.yaw = 0.0f;
}

/* Fixed x/y, constant descent velocity (vz negative). */
static void setLandSetpoint(setpoint_t *sp, float x, float y, float vz)
{
  memset(sp, 0, sizeof(setpoint_t));
  sp->mode.x = modeAbs;
  sp->mode.y = modeAbs;
  sp->position.x = x;
  sp->position.y = y;
  sp->mode.z = modeVelocity;
  sp->velocity.z = vz;
  sp->mode.yaw = modeAbs;
  sp->attitude.yaw = 0.0f;
}

static void stopSetpoint(setpoint_t *sp)
{
  memset(sp, 0, sizeof(setpoint_t));
}

void appMain(void)
{
  static setpoint_t setpoint;

  vTaskDelay(M2T(3000));

  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  logVarId_t idZ = logGetVarId("stateEstimate", "z");
  logVarId_t idIsTumbled = logGetVarId("sys", "isTumbled");

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idEstimator = paramGetVarId("stabilizer", "estimator");

  /* Position control needs the Kalman estimator. */
  paramSetInt(idEstimator, 2);

  uint32_t stateStartTick = xTaskGetTickCount();
  float takeoffX = 0.0f, takeoffY = 0.0f;
  float landX = 0.0f, landY = 0.0f;

  DEBUG_PRINT("Brushless hover/takeoff/land app started\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    bool armed = supervisorIsArmed();
    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    float estX = logGetFloat(idX);
    float estY = logGetFloat(idY);
    float estZ = logGetFloat(idZ);
    uint8_t isTumbled = logGetUint(idIsTumbled);
    uint32_t elapsed = xTaskGetTickCount() - stateStartTick;

    /* Crash guard: if tumbled while flying, cut and disarm. */
    bool inFlight = (flightState == STATE_ARM_DWELL) ||
                    (flightState == STATE_TAKEOFF)   ||
                    (flightState == STATE_HOVER)     ||
                    (flightState == STATE_LAND);
    if (inFlight && isTumbled) {
      DEBUG_PRINT("Tumble detected -> abort & disarm\n");
      stopSetpoint(&setpoint);
      commanderSetSetpoint(&setpoint, 3);
      supervisorRequestArming(false);
      enable = 0;
      reachedHeight = 0;
      flightState = STATE_IDLE;
      stateStartTick = xTaskGetTickCount();
      continue;
    }

    switch (flightState) {
      case STATE_IDLE: {
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);
        reachedHeight = 0;

        /* Auto-start: as soon as the flow deck is present, arm & fly.
         * `enable` can be cleared to hold on the ground / abort. */
        if (enable && positioningInit) {
          DEBUG_PRINT("Flow deck present -> arming\n");
          flightState = STATE_ARMING;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case STATE_ARMING: {
        /* Keep commander alive with zero setpoint while requesting arm. */
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!enable) { flightState = STATE_IDLE; break; }

        if (armed) {
          DEBUG_PRINT("Armed -> idle dwell %.1fs (props spinning)\n", (double)armDwell);
          flightState = STATE_ARM_DWELL;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (elapsed > M2T(armingTimeoutMs)) {
          DEBUG_PRINT("Arming timed out (pre-flight checks failing?) -> IDLE\n");
          enable = 0;
          flightState = STATE_IDLE;
          break;
        }
        /* Required for brushless: motors disabled until this succeeds. */
        supervisorRequestArming(true);
        break;
      }

      case STATE_ARM_DWELL: {
        /* All props at idle, zero thrust, for armDwell seconds. */
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!enable) {
          supervisorRequestArming(false);
          flightState = STATE_IDLE;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        /* If auto-disarmed during the wait, re-arm. */
        if (!armed) {
          DEBUG_PRINT("Disarmed during dwell -> re-arming\n");
          flightState = STATE_ARMING;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        if (elapsed >= M2T((uint32_t)(armDwell * 1000.0f))) {
          takeoffX = estX;
          takeoffY = estY;
          DEBUG_PRINT("Dwell done -> takeoff to %.2f m\n", (double)takeoffHeight);
          flightState = STATE_TAKEOFF;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case STATE_TAKEOFF: {
        if (!enable) {
          landX = estX; landY = estY;
          flightState = STATE_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        float t = (float)elapsed / (float)configTICK_RATE_HZ;
        float a = clampf(t / takeoffTime, 0.0f, 1.0f);
        setAbsSetpoint(&setpoint, takeoffX, takeoffY, a * takeoffHeight);
        commanderSetSetpoint(&setpoint, 3);

        if (a >= 1.0f) {
          reachedHeight = 1;
          DEBUG_PRINT("Reached height -> hover %.1fs\n", (double)hoverTime);
          flightState = STATE_HOVER;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case STATE_HOVER: {
        if (!enable) {
          landX = estX; landY = estY;
          flightState = STATE_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        setAbsSetpoint(&setpoint, takeoffX, takeoffY, takeoffHeight);
        commanderSetSetpoint(&setpoint, 3);

        if (elapsed >= M2T((uint32_t)(hoverTime * 1000.0f))) {
          DEBUG_PRINT("Hover done -> land\n");
          landX = takeoffX; landY = takeoffY;
          flightState = STATE_LAND;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case STATE_LAND: {
        float t = (float)elapsed / (float)configTICK_RATE_HZ;
        bool stillDescending = (estZ > landCutoffM) && (t < landTime + 3.0f);
        if (stillDescending) {
          setLandSetpoint(&setpoint, landX, landY, -landSpeed);
          commanderSetSetpoint(&setpoint, 3);
        } else {
          stopSetpoint(&setpoint);
          commanderSetSetpoint(&setpoint, 3);
          supervisorRequestArming(false);
          DEBUG_PRINT("Landed -> disarmed\n");
          reachedHeight = 0;
          enable = 0;
          flightState = STATE_IDLE;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      default:
        flightState = STATE_IDLE;
        break;
    }
  }
}

PARAM_GROUP_START(hoverapp)
PARAM_ADD(PARAM_UINT8, enable, &enable)
PARAM_ADD(PARAM_FLOAT, height, &takeoffHeight)
PARAM_ADD(PARAM_FLOAT, takeoffTime, &takeoffTime)
PARAM_ADD(PARAM_FLOAT, hoverTime, &hoverTime)
PARAM_ADD(PARAM_FLOAT, landTime, &landTime)
PARAM_ADD(PARAM_FLOAT, armDwell, &armDwell)
PARAM_GROUP_STOP(hoverapp)

LOG_GROUP_START(hoverapp)
LOG_ADD(LOG_UINT8, state, &flightState)
LOG_ADD(LOG_UINT8, reachedHeight, &reachedHeight)
LOG_GROUP_STOP(hoverapp)
