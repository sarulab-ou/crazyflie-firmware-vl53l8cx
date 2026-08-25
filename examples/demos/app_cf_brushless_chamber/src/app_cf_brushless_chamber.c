/*
 * app_cf_brushless_chamber.c
 *
 * Standalone app-layer mission for the Crazyflie Brushless with a flow
 * deck (plus an optional vl53l8cx multizone ToF deck for logging the
 * chamber walls): sit at the floor centre of a small test chamber, arm,
 * take straight off to a fixed hover height, hold there for a set time,
 * then land back on the same spot -- entirely on-board, no radio commands
 * needed once triggered.
 *
 * Intended geometry: a 50 x 50 x 50 cm chamber, drone placed at the floor
 * centre (chamber coords (25, 25, 0) cm). It takes off to (25, 25, 25) cm
 * -- centre of the box, 25 cm up -- hovers for `hoverTimeS`, then descends
 * back to (25, 25, 0) cm.
 *
 * COORDINATE NOTE: the Crazyflie's estimator has NO absolute origin -- it
 * zeroes its position at the takeoff point. So "(25,25,25) cm" is realised
 * as "0.25 m straight up from the takeoff point, holding that x/y".
 * Placing the drone at the chamber floor centre aligns the estimator frame
 * with the chamber frame. Takeoff and landing hold the captured takeoff
 * x/y; the hover phase instead servos to the middle of the box using the
 * four wall distances (see wallCenterVel below). Nothing here servos to an
 * absolute (25,25) -- there is no external reference for that.
 *
 * This is app_cf_brushless_slam_tunnel.c with the forward-tunnel "advance"
 * phase replaced by a fixed-point hover; the arming/takeoff/landing/
 * logging/trigger scaffolding is unchanged -- see that file (and
 * app_cf_brushless_hover_move.c) for detail on those parts. Brushless
 * motors require explicit arming before they spin; this app requests it
 * and waits for it to succeed before takeoff, and disarms after landing.
 *
 * Ranges must NEVER be read over a live radio LogConfig for the vl53l8cx
 * deck (watchdog-reboot risk) -- this app only touches the up sensor via
 * logGetFloat() (reads the live firmware log table directly, not radio)
 * for its hand trigger, and the uSD logger records the full fan.
 *
 * Trigger: set param `chamber.start` = 1 (radio/USB), or hold a hand
 * within `chamber.handMm` of the up-facing sensor (vl53l8cx.s8) for
 * `chamber.handHoldMs`.
 *
 * uSD-card logging: TWO logical streams in one file (fixedFrequency = the
 * 9 vl53l8cx ranges; pose = position/attitude, triggered once per loop
 * below), decoded into two CSVs by sd_log_to_csv.py. Both gated by
 * `usd.logging`, turned on at takeoff and off at landing. See
 * sdcard/config.txt.
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
#include "tof_wall_angle.h"

#define DEBUG_MODULE "CHAMBER_BL"

/* Per-sensor ToF distance [mm] owned by the vl53l8cx deck driver: system.c
 * fills this with the average of the central 4 zones (5, 6, 9, 10) of each
 * sensor, which is exactly what we want for wall centering. Same values as
 * the vl53l8cx.sN log vars. External linkage so we can read it directly
 * without going through the log table. */
extern float vl53l8cxToFAvg[];

/* Fallback average over the OTHER 12 zones of the same sensor, filled by the
 * same loop in system.c. Used only when the central 4 returned nothing. */
// extern float vl53l8cxToFAvgSub[];

/* Sensor index map for wall centering (per user): front=s1, right=s6,
 * left=s5, back=s7. Up (hand trigger) stays s8. */
#define WALL_SENSOR_FRONT 1
#define WALL_SENSOR_RIGHT 6
#define WALL_SENSOR_LEFT  5
#define WALL_SENSOR_BACK  7

/* Zero-payload custom event: the "pose" stream's variable list is defined
 * entirely in config.txt, not in this payload. */
EVENTTRIGGER(pose)

/* ========================= Mission parameters ========================= */
/* Set to 1 to arm+fly on every boot with no trigger -- only once you trust
 * the mission and the area is clear. */
static uint8_t startMission = 0;

/* Hover height above the takeoff point. 0.25 m = centre of a 50 cm box. */
static float takeoffHeight = 0.25f;   /* m */
static float hoverTimeS    = 10.0f;   /* s, hover duration */

/* Time to keep all props at idle (armed, zero thrust) before takeoff. */
static float armDwellS = 3.0f;

/* Wall centering during hover -- same proportional-on-difference form as
 * the lateral centering in app_cf21_slam_tunnel.c, applied to BOTH body
 * axes: v = clamp(kCenter * (dNear - dFar), +-centerMaxV). No target
 * distance and no deadband: the opposing pair of walls IS the reference,
 * so the command goes to zero exactly when the drone is centred. */
static float kCenter    = 0.7f;    /* centering gain (same as slam_tunnel) */
static float centerMaxV = 0.05f;   /* m/s, centering command clamp (small box) */

/* Deadband on the OPPOSING-PAIR DIFFERENCE [m]. While |dNear - dFar| is at
 * or below this, that axis is considered centred and stops being driven:
 * the difference is twice the offset from the centre, so 0.05 m here means
 * "within 2.5 cm of centred". Set 0 to disable. */
static float holdBandM = 0.05f;

/* What a centred axis does inside the deadband:
 *   0 = command zero body velocity (modeVelocity). Damps motion but has NO
 *       restoring force, so the drone slowly drifts within the band.
 *   1 = hold the position it had when it entered the band (modeAbs), which
 *       gives a real restoring force but reintroduces a dependence on the
 *       drifting flow-deck estimate for as long as it stays in the band.
 * Per axis and independent: one axis can hold while the other still centres. */
static uint8_t holdBandUsePos = 0;

/* No-PC-link trigger: hold a hand this close (mm) over the up-facing
 * vl53l8cx sensor (s8) for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: constant-velocity descent, sized so the whole descent takes
 * landTimeS. The commanded speed is takeoffHeight / landTimeS, so changing
 * the hover height keeps the landing duration the same (mirrors takeoff,
 * which is also specified as a time rather than a speed).
 * Motors cut once estZ drops below landCutoffM, or after landMaxTime_s
 * regardless -- landMaxTime_s stays a generous safety fallback, not the
 * nominal duration. */
static float landTimeS   = 1.0f;    /* s, nominal descent duration */
static float landCutoffM = 0.04f;   /* m */

/* Yaw alignment to the walls during hover. tof_wall_angle.c fits a plane to
 * each vl53l8cx fan and fuses the wall-normal azimuths with a 4x circular
 * mean, giving bodyDeg = the body's yaw offset from square-on to the nearest
 * wall, in [-45,+45), 0 = perpendicular. It is a ToF-only absolute measure
 * (no gyro integration), so unlike the estimator's yaw it does not drift.
 *
 * SIGN: az[] is the wall normal's azimuth expressed in BODY coordinates, and
 * a body yawed by +psi sees that azimuth shift by -psi. So bodyDeg = -psi
 * relative to square, and the estimator-frame heading that squares us up is
 *   desired = yawDeg + bodyDeg
 * yawAlignSign is exposed so this can be flipped in flight if the rig
 * disagrees -- a wrong sign here diverges instead of converging. log73/log74
 * settled this empirically: -1 is correct (log73 flew +1 and diverged from
 * 5 to 20 deg; log74 flew -1 and held a median 5.1 deg).
 *
 * The setpoint is driven by a PROPORTIONAL law on the MEASURED WALL
 * MISALIGNMENT -- wallYaw itself, not the setpoint residual:
 *
 *   rate = clamp(yawKp * yawAlignSign * wallYaw, +-yawRateMaxDps)   [deg/s]
 *   yawCmdDeg += rate * dt
 *
 * With yawKp = 0.2 /s and yawRateMaxDps = 5 deg/s that gives
 *   |wallYaw| >= 25 deg -> 5 deg/s (saturated)
 *   |wallYaw|  = 20 deg -> 4 deg/s
 *   |wallYaw|  = 15 deg -> 3 deg/s
 *   |wallYaw|  = 10 deg -> 2 deg/s
 * and while |wallYaw| < yawDeadbandDeg the setpoint is FROZEN (no correction
 * at all), so it stops chattering once the drone is square to the wall.
 * The deadband cuts in above where the P law would still be asking for
 * 1 deg/s, which is intentional.
 *
 * Driving off wallYaw directly (rather than off "desired - yawCmd") keeps the
 * estimator's noisy yaw out of the loop: wallYaw is an absolute, ToF-only
 * measurement of exactly the quantity we want to zero.
 *
 * dt is measured from the tick counter, not assumed to be loopDt_ms: log74
 * showed this loop actually running at ~29 Hz, which would have made a
 * nominal 5 deg/s come out as ~3 deg/s.
 *
 * Only APP_HOVER aligns; takeoff and landing keep holding holdYawDeg. */
static uint8_t yawAlignEnable = 1;
static float yawAlignSign     = -1.0f;
static float yawRateMaxDps    = 9.0f;   /* deg/s, hard cap on the correction rate */
static float yawKp            = 0.2f;   /* 1/s, correction rate per deg of error */
static float yawDeadbandDeg   = 5.0f;   /* deg, below this the setpoint is frozen */

/* Yaw is held at this absolute heading for the whole mission (takeoff,
 * hover, landing). The centering above is expressed in body axes, so the
 * body/chamber axes must stay aligned for it to mean anything. */
static float holdYawDeg = 0.0f;   /* deg */

static const float takeoffTime_s      = 2.0f;  /* s, 0 -> takeoffHeight ramp */
static const float landMaxTime_s      = 6.0f;
static const uint16_t loopDt_ms       = 20;   /* 50 Hz */
static const uint16_t deckStableMs    = 500;
static const uint16_t prearmMs        = 1000;
static const uint16_t armingTimeoutMs = 3000;

/* ========================= App state machine ========================= */
typedef enum {
  APP_IDLE = 0,
  APP_WAIT_FOR_DECK,
  APP_PREARM,
  APP_ARMING,
  APP_ARM_DWELL,
  APP_TAKEOFF,
  APP_HOVER,
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

/* wall distances used for centering [m]; <0 means "no valid reading" */
static float dFrontLog = -1.0f;
static float dBackLog  = -1.0f;
static float dLeftLog  = -1.0f;
static float dRightLog = -1.0f;

/* commanded body-frame centering velocities [m/s] */
static float vxCmdLog = 0.0f;
static float vyCmdLog = 0.0f;

/* 1 while that axis is inside the deadband (centred, not being driven) */
static uint8_t holdXLog = 0;
static uint8_t holdYLog = 0;

/* yaw alignment: measured wall offset, commanded heading, measurement valid */
static float wallYawDegLog = 0.0f;
static float yawCmdDegLog = 0.0f;
static uint8_t yawAlignOkLog = 0;
/* remaining setpoint error and the rate the P law asked for */
static float yawErrDegLog = 0.0f;
static float yawRateDpsLog = 0.0f;

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* Wrap an angle into [-180, 180) deg. */
static inline float wrap180f(float deg)
{
  while (deg >= 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

/* Driver's central-4-zone average for one sensor, in metres. The driver
 * averages all four zones unconditionally, so a zone with no target drags
 * the result down (or to <=0); treat anything non-positive as invalid. */
static float wallDist(int sensor)
{
  float mm = vl53l8cxToFAvg[sensor];
  if (mm <= 0.0f) {
    return 0.0f;
  }
  return 0.001f * mm;
}

/* Body-frame centering velocity, same shape as the lateral centering in
 * app_cf21_slam_tunnel.c (vy = clamp(kCenter*(rLeft-rRight), +-max)), but
 * applied to both axes: body x from front/back, body y from left/right.
 * front=+x, back=-x, left=+y, right=-y, so a positive difference means the
 * far wall is ahead/left and the command pushes that way.
 *
 * Unlike slam_tunnel, an invalid reading (d<0) does NOT fall back to a
 * large "no detection" distance: in a closed box that would saturate the
 * command straight into the wall opposite the dead sensor. Instead the
 * axis gets no command unless BOTH of its sensors are valid. */
static void wallCenterVel(float dFront, float dBack, float dLeft, float dRight,
                          float gain, float maxV,
                          float *vx, float *vy)
{
  *vx = (dFront >= 0.0f && dBack  >= 0.0f)
            ? clampf(gain * (dFront - dBack), -maxV, maxV) : 0.0f;
  *vy = (dLeft  >= 0.0f && dRight >= 0.0f)
            ? clampf(gain * (dLeft - dRight), -maxV, maxV) : 0.0f;
}

/* Per-axis mix of absolute position hold and body-frame velocity, with
 * absolute height/yaw. An axis inside the deadband is flown as modeAbs to
 * holdX/holdY; an axis still being centred is flown as modeVelocity at
 * vxBody/vyBody.
 *
 * position.x/y are filled in BOTH cases on purpose: positionController()
 * builds its body-frame setpoint as
 *   setp_body_x = position.x*cos(yaw) + position.y*sin(yaw)
 * so the modeAbs axis reads the other axis' position too whenever yaw != 0.
 * Leaving the velocity-mode axis at 0 there would corrupt the held axis. */
static void setHoverMixedSetpoint(setpoint_t *setpoint,
                                  bool holdXAxis, bool holdYAxis,
                                  float holdX, float holdY,
                                  float vxBody, float vyBody,
                                  float z, float yawDeg)
{
  memset(setpoint, 0, sizeof(setpoint_t));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yawDeg;

  setpoint->position.x = holdX;
  setpoint->position.y = holdY;

  setpoint->mode.x = holdXAxis ? modeAbs : modeVelocity;
  setpoint->mode.y = holdYAxis ? modeAbs : modeVelocity;
  setpoint->velocity.x = holdXAxis ? 0.0f : vxBody;
  setpoint->velocity.y = holdYAxis ? 0.0f : vyBody;
  setpoint->velocity_body = true;
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
  logVarId_t idUp = logGetVarId("vl53l8cx", "s8");  /* up sensor, hand trigger */

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

  float takeoffX = 0.0f, takeoffY = 0.0f;
  /* Position each axis holds while inside the deadband. Captured on the way
   * in and left alone until the axis leaves the band again. */
  float bandHoldX = 0.0f, bandHoldY = 0.0f;
  bool inBandX = false, inBandY = false;
  /* Rate-limited absolute yaw setpoint used during hover. */
  float yawCmdDeg = 0.0f;
  uint32_t yawLastTick = 0;   /* for the measured-dt yaw rate limit */
  /* Landing returns to the captured takeoff point, not to wherever the
   * centering left us -- the chamber mission is "take off and land on the
   * same spot". */

  // DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  // DEBUG_PRINT("Brushless chamber hover app started\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    bool armed = supervisorIsArmed();

    float estX = logGetFloat(idX);
    float estY = logGetFloat(idY);
    float estZ = logGetFloat(idZ);
    float yawDeg = logGetFloat(idYaw);
    uint8_t isTumbled = logGetUint(idIsTumbled);
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

    /* Trigger the "pose" uSD stream every tick; usddeck no-ops this when
     * usd.logging is off, so it's cheap to call unconditionally. */
    eventTrigger(&eventTrigger_pose);

    bool inFlightState =
        (appState == APP_ARMING) ||
        (appState == APP_ARM_DWELL) ||
        (appState == APP_TAKEOFF) ||
        (appState == APP_HOVER) ||
        (appState == APP_LAND);

    if (inFlightState && isTumbled) {
      // DEBUG_PRINT("CRASH detected (sys.isTumbled)\n");
      if (usdLoggingActive) {
        // DEBUG_PRINT("Stop uSD logging\n");
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
        vxCmdLog = 0.0f;
        vyCmdLog = 0.0f;
        holdXLog = 0;
        holdYLog = 0;
        wallYawDegLog = 0.0f;
        yawCmdDegLog = 0.0f;
        yawAlignOkLog = 0;
        yawErrDegLog = 0.0f;
        yawRateDpsLog = 0.0f;

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
            // DEBUG_PRINT("Hand-wave trigger -> starting mission\n");
            startMission = 1;
            handSeenStarted = false;
          }
        } else {
          handSeenStarted = false;
        }

        if (startMission) {
          // DEBUG_PRINT("Trigger received\n");
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
            // DEBUG_PRINT("Flow deck stable -> PREARM\n");
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
          // DEBUG_PRINT("Flow deck lost in PREARM\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if ((xTaskGetTickCount() - stateStartTick) > M2T(prearmMs)) {
          // DEBUG_PRINT("PREARM done -> ARMING\n");
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
          // DEBUG_PRINT("Flow deck lost during ARMING\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if (armed) {
          // DEBUG_PRINT("Armed -> ARM_DWELL (%.1fs)\n", (double)armDwellS);
          appState = APP_ARM_DWELL;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        if ((xTaskGetTickCount() - stateStartTick) > M2T(armingTimeoutMs)) {
          // DEBUG_PRINT("Arming timed out -> IDLE\n");
          startMission = 0;
          appState = APP_IDLE;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        supervisorRequestArming(true);
        break;
      }

      case APP_ARM_DWELL: {
        /* All props at idle (armed, zero thrust) for armDwellS. */
        stopSetpoint(&setpoint);
        commanderSetSetpoint(&setpoint, 3);

        if (!startMission) {
          supervisorRequestArming(false);
          appState = APP_IDLE;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          // DEBUG_PRINT("Flow deck lost during arm dwell\n");
          supervisorRequestArming(false);
          appState = APP_WAIT_FOR_DECK;
          break;
        }
        if (!armed) {
          // DEBUG_PRINT("Disarmed during dwell -> ARMING\n");
          appState = APP_ARMING;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= armDwellS) {
          // DEBUG_PRINT("Arm dwell done -> TAKEOFF\n");
          takeoffX = estX;
          takeoffY = estY;
          appState = APP_TAKEOFF;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_TAKEOFF: {
        if (!startMission) {
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          // DEBUG_PRINT("Flow deck lost during takeoff -> LAND\n");
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        if (!usdLoggingActive) {
          // DEBUG_PRINT("Start uSD logging\n");
          paramSetInt(idUsdLogging, 1);
          usdLoggingActive = true;
        }

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        float a = clampf(t / takeoffTime_s, 0.0f, 1.0f);
        phaseElapsedSLog = t;

        setAbsSetpoint(&setpoint, takeoffX, takeoffY, a * takeoffHeight, holdYawDeg);
        commanderSetSetpoint(&setpoint, 3);

        if (a >= 1.0f) {
          // DEBUG_PRINT("Takeoff complete -> HOVER %.1fs\n", (double)hoverTimeS);
          bandHoldX = takeoffX; bandHoldY = takeoffY;
          inBandX = false; inBandY = false;
          yawCmdDeg = holdYawDeg;   /* start from where takeoff left us */
          yawLastTick = xTaskGetTickCount();
          appState = APP_HOVER;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_HOVER: {
        if (!startMission) {
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }
        if (!positioningInit) {
          // DEBUG_PRINT("Flow deck lost during hover -> LAND\n");
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        /* --- wall centering from the vl53l8cx central-4-zone averages --- */
        float dFront = wallDist(WALL_SENSOR_FRONT);
        float dBack  = wallDist(WALL_SENSOR_BACK);
        float dLeft  = wallDist(WALL_SENSOR_LEFT);
        float dRight = wallDist(WALL_SENSOR_RIGHT);
        dFrontLog = dFront; dBackLog = dBack; dLeftLog = dLeft; dRightLog = dRight;

        float vx, vy;
        wallCenterVel(dFront, dBack, dLeft, dRight, kCenter, centerMaxV, &vx, &vy);

        /* Deadband: an axis whose opposing-pair difference is within
         * holdBandM counts as centred and stops being driven. An axis with a
         * dead sensor pair is never treated as centred -- wallCenterVel()
         * already zeroed its command, and pretending it is centred would let
         * it latch a hold point on no evidence. */
        bool centredX = (holdBandM > 0.0f) && (dFront >= 0.0f && dBack  >= 0.0f)
                        && (fabsf(dFront - dBack)  <= holdBandM);
        bool centredY = (holdBandM > 0.0f) && (dLeft  >= 0.0f && dRight >= 0.0f)
                        && (fabsf(dLeft  - dRight) <= holdBandM);

        /* Capture the hold point on the way INTO the band; while outside,
         * keep it tracking the live estimate so it is fresh on entry. */
        if (centredX && !inBandX) { bandHoldX = estX; }
        if (!centredX)            { bandHoldX = estX; }
        if (centredY && !inBandY) { bandHoldY = estY; }
        if (!centredY)            { bandHoldY = estY; }
        inBandX = centredX;
        inBandY = centredY;

        if (centredX) vx = 0.0f;
        if (centredY) vy = 0.0f;
        vxCmdLog = vx;
        vyCmdLog = vy;
        holdXLog = centredX ? 1 : 0;
        holdYLog = centredY ? 1 : 0;

        /* --- yaw: square the sensors up to the walls, slew-rate limited --- */
        /* tofWallAngleIsValid() is the module's own gate: the fused angle has
         * cleared tofwall.confMin (default 0.70) and has been refreshed within
         * the last STALE_FRAMES (10) ranging frames. Without it we would be
         * steering off a stale or low-agreement estimate. */
        bool yawOk = yawAlignEnable && tofWallAngleIsValid();
        float wallYawDeg = yawOk ? tofWallAngleGetBodyDeg() : 0.0f;

        /* Measured loop period, so the deg/s figures below are real. Clamped
         * so a scheduling hiccup or the first pass cannot produce a jump. */
        uint32_t nowTick = xTaskGetTickCount();
        float dtS = (float)(nowTick - yawLastTick) / (float)configTICK_RATE_HZ;
        yawLastTick = nowTick;
        dtS = clampf(dtS, 0.0f, 0.2f);

        float yawErr = 0.0f, yawRate = 0.0f;
        if (yawOk && isfinite(wallYawDeg)) {
          /* The error the loop acts on IS the wall misalignment. yawAlignSign
           * turns it into "which way must the heading setpoint move". */
          yawErr = yawAlignSign * wallYawDeg;

          if (fabsf(wallYawDeg) >= yawDeadbandDeg) {
            /* Proportional, saturating at yawRateMaxDps. */
            yawRate = clampf(yawKp * yawErr, -yawRateMaxDps, yawRateMaxDps);
            yawCmdDeg = wrap180f(yawCmdDeg + yawRate * dtS);
          }
          /* else: within yawDeadbandDeg of square -> hold the heading exactly. */
        } else {
          /* No usable wall estimate (or alignment disabled): freeze the
           * setpoint rather than snapping back to holdYawDeg, so a momentary
           * dropout does not jerk the yaw. */
          wallYawDeg = 0.0f;
        }
        wallYawDegLog = wallYawDeg;
        yawCmdDegLog = yawCmdDeg;
        yawAlignOkLog = yawOk ? 1 : 0;
        yawErrDegLog = yawErr;
        yawRateDpsLog = yawRate;

        /* Body-velocity on the axes still being centred; the centred axes
         * either hold position (modeAbs) or just command zero velocity,
         * depending on holdBandUsePos. Height is absolute; yaw is the
         * rate-limited wall-aligned heading. */
        bool posHoldX = centredX && holdBandUsePos;
        bool posHoldY = centredY && holdBandUsePos;
        setHoverMixedSetpoint(&setpoint, posHoldX, posHoldY,
                              bandHoldX, bandHoldY, vx, vy,
                              takeoffHeight, yawCmdDeg);
        commanderSetSetpoint(&setpoint, 3);

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= hoverTimeS) {
          // DEBUG_PRINT("Hover complete -> LAND\n");
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
        }
        break;
      }

      case APP_LAND: {
        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;
        bool stillDescending = (estZ > landCutoffM) && (t < landMaxTime_s);

        if (stillDescending) {
          /* takeoffHeight in landTimeS seconds. Guard against a zero/negative
           * landTimeS being written over the radio. */
          float landSpeed = (landTimeS > 0.05f) ? (takeoffHeight / landTimeS)
                                                : (takeoffHeight / 0.05f);
          setLandDescentSetpoint(&setpoint, takeoffX, takeoffY, -landSpeed, holdYawDeg);
          commanderSetSetpoint(&setpoint, 3);
        } else {
          stopSetpoint(&setpoint);
          commanderSetSetpoint(&setpoint, 3);
          supervisorRequestArming(false);

          // DEBUG_PRINT("Landing complete -> IDLE\n");
          if (usdLoggingActive) {
            // DEBUG_PRINT("Stop uSD logging\n");
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

PARAM_GROUP_START(chamber)
PARAM_ADD(PARAM_UINT8, start, &startMission)
PARAM_ADD(PARAM_FLOAT, takeoffH, &takeoffHeight)
PARAM_ADD(PARAM_FLOAT, hoverTimeS, &hoverTimeS)
PARAM_ADD(PARAM_FLOAT, armDwellS, &armDwellS)
PARAM_ADD(PARAM_FLOAT, kCenter, &kCenter)
PARAM_ADD(PARAM_FLOAT, centerMaxV, &centerMaxV)
PARAM_ADD(PARAM_FLOAT, holdBandM, &holdBandM)
PARAM_ADD(PARAM_UINT8, holdBandPos, &holdBandUsePos)
PARAM_ADD(PARAM_UINT8, yawAlign, &yawAlignEnable)
PARAM_ADD(PARAM_FLOAT, yawAlignSign, &yawAlignSign)
PARAM_ADD(PARAM_FLOAT, yawRateMax, &yawRateMaxDps)
PARAM_ADD(PARAM_FLOAT, yawKp, &yawKp)
PARAM_ADD(PARAM_FLOAT, yawDbDeg, &yawDeadbandDeg)
PARAM_ADD(PARAM_FLOAT, holdYawDeg, &holdYawDeg)
PARAM_ADD(PARAM_UINT16, handMm, &handTriggerMm)
PARAM_ADD(PARAM_UINT16, handHoldMs, &handTriggerHoldMs)
PARAM_ADD(PARAM_FLOAT, landTimeS, &landTimeS)
PARAM_ADD(PARAM_FLOAT, landCutoffM, &landCutoffM)
PARAM_GROUP_STOP(chamber)

LOG_GROUP_START(chamber)
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
LOG_ADD(LOG_FLOAT, dFront, &dFrontLog)
LOG_ADD(LOG_FLOAT, dBack, &dBackLog)
LOG_ADD(LOG_FLOAT, dLeft, &dLeftLog)
LOG_ADD(LOG_FLOAT, dRight, &dRightLog)
LOG_ADD(LOG_FLOAT, vxCmd, &vxCmdLog)
LOG_ADD(LOG_FLOAT, vyCmd, &vyCmdLog)
LOG_ADD(LOG_UINT8, holdX, &holdXLog)
LOG_ADD(LOG_UINT8, holdY, &holdYLog)
LOG_ADD(LOG_FLOAT, wallYaw, &wallYawDegLog)
LOG_ADD(LOG_FLOAT, yawCmd, &yawCmdDegLog)
LOG_ADD(LOG_UINT8, yawAlignOk, &yawAlignOkLog)
LOG_ADD(LOG_FLOAT, yawErr, &yawErrDegLog)
LOG_ADD(LOG_FLOAT, yawRate, &yawRateDpsLog)
LOG_GROUP_STOP(chamber)
