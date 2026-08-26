/*
 * app_cf_brushless_chamber_without_Pcontrol.c
 *
 * CONTROL EXPERIMENT -- the "no P control" arm of an A/B pair with
 * app_cf_brushless_chamber.c. Everything (arming, takeoff ramp, hover
 * duration, landing, uSD logging, triggers, sensor plumbing, log/param
 * names) is IDENTICAL; the ONLY difference is what APP_HOVER commands:
 *
 *   app_cf_brushless_chamber.c      -> body-velocity wall centering,
 *                                      vx/vy = clamp(kCenter*(dNear-dFar), +-centerMaxV)
 *   THIS FILE                       -> no wall feedback at all. Holds the
 *                                      captured takeoff x/y as an absolute
 *                                      position setpoint, exactly as the
 *                                      takeoff phase does.
 *
 * So the drone just stays where it took off (horizontally) for hoverTimeS
 * and then lands. The wall distances are still read and logged every loop
 * so the two runs can be compared with the same analysis script -- they are
 * simply never fed back into the setpoint. The commanded centering velocity
 * that the P-control build WOULD have produced is also computed and logged
 * (chamber.vxCmd / vyCmd) without being applied, so the counterfactual is
 * visible in the log.
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
 * with the chamber frame. ALL phases -- takeoff, hover and landing -- hold
 * the captured takeoff x/y. Nothing here servos to an absolute (25,25);
 * there is no external reference for that. Note that this makes the hover
 * position only as good as the flow deck's estimate, which drifts with no
 * absolute reference to correct it -- measuring that drift against the wall
 * ranges is the point of this build.
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

#define DEBUG_MODULE "CHAMBER_NOP"

/* Per-sensor ToF distance [mm] owned by the vl53l8cx deck driver: system.c
 * fills this with the average of the central 4 zones (5, 6, 9, 10) of each
 * sensor, which is exactly what we want for wall centering. Same values as
 * the vl53l8cx.sN log vars. External linkage so we can read it directly
 * without going through the log table. */
extern float vl53l8cxToFAvg[];

/* NOTE: the feat#4 branch also had vl53l8cxToFAvgSub[] (average of the outer
 * 12 zones) as a fallback. It does not exist here, and all three chamber
 * builds now read the distance identically, so the fallback is gone. */
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

/* NOT APPLIED in this build -- kept so the centering command that the
 * P-control build would have issued can still be computed and logged for
 * comparison. Same values and same meaning as app_cf_brushless_chamber.c:
 * v = clamp(kCenter * (dNear - dFar), +-centerMaxV). */
static float kCenter    = 0.3f;    /* centering gain (same as slam_tunnel) */
static float centerMaxV = 0.15f;   /* m/s, centering command clamp (small box) */

/* No-PC-link trigger: hold a hand this close (mm) over the up-facing
 * vl53l8cx sensor (s8) for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: constant, gentle descent. Motors cut once estZ drops below
 * landCutoffM, or after landMaxTime_s regardless. */
/* Descent is specified as a DURATION (like takeoff), not a speed: the
 * commanded speed is takeoffHeight / landTimeS, so changing the hover height
 * keeps the landing 1 s. landMaxTime_s stays a generous safety fallback. */
static float landTimeS   = 1.0f;    /* s, nominal descent duration */
static float landCutoffM = 0.04f;   /* m */

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

/* Deadband flags. This build has no deadband logic, so they stay 0 -- kept so
 * the chamber log group is identical across all the A/B builds. */
static uint8_t holdXLog = 0;
static uint8_t holdYLog = 0;

/* Yaw alignment. NOT implemented in this build: wallYaw / yawAlignOk carry the
 * live tofWallAngle measurement (read-only, never fed back) so the wall angle
 * is still recorded for comparison, while yawCmd is simply the constant
 * heading actually being commanded and yawErr / yawRate stay 0 because no
 * correction is applied. Same names and types as app_cf_brushless_chamber.c. */
/* Wall measurements, recorded but never fed back (same names/types as the
 * takeoff_Pcontrol_wall / _hybrid builds). p* = plane fit, z* = zone average. */
static float pFrontLog = -1.0f;
static float pBackLog  = -1.0f;
static float pLeftLog  = -1.0f;
static float pRightLog = -1.0f;
static float zFrontLog = -1.0f;
static float zBackLog  = -1.0f;
static float zLeftLog  = -1.0f;
static float zRightLog = -1.0f;
static uint8_t planeOkLog = 0;   /* bit0=front bit1=back bit2=left bit3=right */
/* This build selects no distance source at all (it never centers off the
 * walls). -2 marks that, and is distinct from the _hybrid meanings
 * (1 = plane, 0 = zone average, -1 = axis left undriven). */
static int8_t srcXLog = -2;
static int8_t srcYLog = -2;

static float wallYawDegLog = 0.0f;
static float yawCmdDegLog = 0.0f;
static uint8_t yawAlignOkLog = 0;
static float yawErrDegLog = 0.0f;
static float yawRateDpsLog = 0.0f;

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* Driver's central-4-zone average for one sensor, in metres. The driver
 * averages all four zones unconditionally, so a zone with no target drags
 * the result down (or to <=0); treat anything non-positive as invalid. */
/* Zone average [m] (the 16-zone mean the driver keeps), <0 if unavailable. */
static float wallDistZoneAvg(int sensor)
{
  float mm = vl53l8cxToFAvg[sensor];
  return (mm > 0.0f) ? (0.001f * mm) : -1.0f;
}

/* Distance from the BODY ORIGIN to the PCA/RANSAC-fitted plane [m], <0 if the
 * fit failed this frame. Read-only here: this build never uses the walls for
 * control, but recording it keeps the measurement environment identical to
 * the takeoff_Pcontrol_* builds so the runs can be compared directly. */
static float wallDistPlane(int sensor)
{
  return tofWallAngleGetPlaneDistM((uint8_t)sensor);
}

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
 * axis gets no command unless BOTH of its sensors are valid.
 *
 * In THIS build the result is logged but never commanded. */
static void wallCenterVel(float dFront, float dBack, float dLeft, float dRight,
                          float gain, float maxV,
                          float *vx, float *vy)
{
  *vx = (dFront >= 0.0f && dBack  >= 0.0f)
            ? clampf(gain * (dFront - dBack), -maxV, maxV) : 0.0f;
  *vy = (dLeft  >= 0.0f && dRight >= 0.0f)
            ? clampf(gain * (dLeft - dRight), -maxV, maxV) : 0.0f;
}

/* NOTE: app_cf_brushless_chamber.c has a setHoverVelSetpoint() here, which
 * issues the body-frame velocity command for wall centering. This build
 * never commands horizontal motion, so that helper is deliberately absent
 * (the build treats unused static functions as errors). That omission is
 * the whole difference between the two apps. */

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
  /* Landing returns to the captured takeoff point, not to wherever the
   * centering left us -- the chamber mission is "take off and land on the
   * same spot". */

  DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  DEBUG_PRINT("Brushless chamber hover app started (NO P control)\n");

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
        vxCmdLog = 0.0f;
        vyCmdLog = 0.0f;

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
            DEBUG_PRINT("Flow deck stable -> PREARM\n");
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
          DEBUG_PRINT("Flow deck lost in PREARM\n");
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
          DEBUG_PRINT("Flow deck lost during ARMING\n");
          appState = APP_WAIT_FOR_DECK;
          break;
        }

        if (armed) {
          DEBUG_PRINT("Armed -> ARM_DWELL (%.1fs)\n", (double)armDwellS);
          appState = APP_ARM_DWELL;
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
          DEBUG_PRINT("Flow deck lost during arm dwell\n");
          supervisorRequestArming(false);
          appState = APP_WAIT_FOR_DECK;
          break;
        }
        if (!armed) {
          DEBUG_PRINT("Disarmed during dwell -> ARMING\n");
          appState = APP_ARMING;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= armDwellS) {
          DEBUG_PRINT("Arm dwell done -> TAKEOFF\n");
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
          DEBUG_PRINT("Flow deck lost during takeoff -> LAND\n");
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
        phaseElapsedSLog = t;

        setAbsSetpoint(&setpoint, takeoffX, takeoffY, a * takeoffHeight, holdYawDeg);
        commanderSetSetpoint(&setpoint, 3);

        if (a >= 1.0f) {
          DEBUG_PRINT("Takeoff complete -> HOVER %.1fs\n", (double)hoverTimeS);
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
          DEBUG_PRINT("Flow deck lost during hover -> LAND\n");
          appState = APP_LAND;
          stateStartTick = xTaskGetTickCount();
          break;
        }

        /* --- wall distances: read and logged, but NOT fed back --- */
        float dFront = wallDist(WALL_SENSOR_FRONT);
        float dBack  = wallDist(WALL_SENSOR_BACK);
        float dLeft  = wallDist(WALL_SENSOR_LEFT);
        float dRight = wallDist(WALL_SENSOR_RIGHT);
        dFrontLog = dFront; dBackLog = dBack; dLeftLog = dLeft; dRightLog = dRight;

        /* Counterfactual only: what the P-control build would have commanded
         * right now. Logged so the A/B comparison can show what the centering
         * loop was "asking for" while this build ignored it. */
        float vx, vy;
        wallCenterVel(dFront, dBack, dLeft, dRight, kCenter, centerMaxV, &vx, &vy);
        vxCmdLog = vx;
        vyCmdLog = vy;

        /* THE EXPERIMENT: no horizontal motion commanded at all. Hold the
         * captured takeoff x/y as an absolute position setpoint, exactly as
         * APP_TAKEOFF does, so the only thing keeping the drone in place is
         * the flow deck's position estimate. */
        setAbsSetpoint(&setpoint, takeoffX, takeoffY, takeoffHeight, holdYawDeg);
        commanderSetSetpoint(&setpoint, 3);


            /* --- wall distances: measured only, never applied in this build --- */
        pFrontLog = wallDistPlane(WALL_SENSOR_FRONT);
        pBackLog  = wallDistPlane(WALL_SENSOR_BACK);
        pLeftLog  = wallDistPlane(WALL_SENSOR_LEFT);
        pRightLog = wallDistPlane(WALL_SENSOR_RIGHT);
        zFrontLog = wallDistZoneAvg(WALL_SENSOR_FRONT);
        zBackLog  = wallDistZoneAvg(WALL_SENSOR_BACK);
        zLeftLog  = wallDistZoneAvg(WALL_SENSOR_LEFT);
        zRightLog = wallDistZoneAvg(WALL_SENSOR_RIGHT);
        planeOkLog = (uint8_t)(((pFrontLog > 0.0f) ? 1u : 0u) |
                               ((pBackLog  > 0.0f) ? 2u : 0u) |
                               ((pLeftLog  > 0.0f) ? 4u : 0u) |
                               ((pRightLog > 0.0f) ? 8u : 0u));

    /* --- yaw: measured only, never applied in this build --- */
        bool yawOk = tofWallAngleIsValid();
        float wallYawDeg = yawOk ? tofWallAngleGetBodyDeg() : 0.0f;
        if (!isfinite(wallYawDeg)) { wallYawDeg = 0.0f; }
        wallYawDegLog = wallYawDeg;
        yawAlignOkLog = yawOk ? 1 : 0;
        yawCmdDegLog = holdYawDeg;
        yawErrDegLog = 0.0f;
        yawRateDpsLog = 0.0f;

        float t = (float)(xTaskGetTickCount() - stateStartTick) / (float)configTICK_RATE_HZ;
        phaseElapsedSLog = t;

        if (t >= hoverTimeS) {
          DEBUG_PRINT("Hover complete -> LAND\n");
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

PARAM_GROUP_START(chamber)
PARAM_ADD(PARAM_UINT8, start, &startMission)
PARAM_ADD(PARAM_FLOAT, takeoffH, &takeoffHeight)
PARAM_ADD(PARAM_FLOAT, hoverTimeS, &hoverTimeS)
PARAM_ADD(PARAM_FLOAT, armDwellS, &armDwellS)
PARAM_ADD(PARAM_FLOAT, kCenter, &kCenter)
PARAM_ADD(PARAM_FLOAT, centerMaxV, &centerMaxV)
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
LOG_ADD(LOG_FLOAT, pFront, &pFrontLog)
LOG_ADD(LOG_FLOAT, pBack, &pBackLog)
LOG_ADD(LOG_FLOAT, pLeft, &pLeftLog)
LOG_ADD(LOG_FLOAT, pRight, &pRightLog)
LOG_ADD(LOG_FLOAT, zFront, &zFrontLog)
LOG_ADD(LOG_FLOAT, zBack, &zBackLog)
LOG_ADD(LOG_FLOAT, zLeft, &zLeftLog)
LOG_ADD(LOG_FLOAT, zRight, &zRightLog)
LOG_ADD(LOG_UINT8, planeOk, &planeOkLog)
LOG_ADD(LOG_INT8, srcX, &srcXLog)
LOG_ADD(LOG_INT8, srcY, &srcYLog)
LOG_GROUP_STOP(chamber)
