/*
 * app_cf_brushless_chamber_abscenter.c
 *
 * Improved variant of app_cf_brushless_chamber_without_Pcontrol.c. Same
 * scaffolding as the rest of the chamber family (arming, takeoff ramp,
 * hover duration, landing, uSD logging, triggers, log/param names); the
 * only thing that differs is what APP_HOVER commands:
 *
 *   app_cf_brushless_chamber.c            -> body-VELOCITY wall centering,
 *                                            vx/vy = clamp(kCenter*(dNear-dFar), +-centerMaxV)
 *   app_cf_brushless_chamber_without_Pcontrol.c
 *                                         -> modeAbs hold of the takeoff x/y,
 *                                            i.e. servo to estimator (0,0)
 *   THIS FILE                             -> modeAbs hold of the BOX CENTRE,
 *                                            re-derived from the ToF ranges
 *                                            every loop
 *
 * The idea: the takeoff point is a poor target, because the flow deck's
 * estimate of it drifts (measured: 0.2-0.6 m over a 10 s hover) and there
 * is no absolute reference to correct it. The four wall ranges DO give an
 * absolute reference, so instead of holding estimator (0,0) we work out
 * where the centre of the box currently is IN ESTIMATOR COORDINATES and
 * hold that:
 *
 *   offX = (dFront - dBack) / 2      <- signed distance from the drone to
 *   offY = (dLeft  - dRight) / 2        the box centre, along each axis
 *
 *   setpoint.position.x = estX + offX   (modeAbs)
 *   setpoint.position.y = estY + offY   (modeAbs)
 *
 * Sign convention (same as wallCenterVel() in app_cf_brushless_chamber.c):
 * front = +x, back = -x, left = +y, right = -y. With half-box h and the
 * drone at x relative to the centre, dFront = h - x and dBack = h + x, so
 * (dFront - dBack)/2 = -x. That is the vector FROM the drone TO the centre,
 * which is exactly what has to be added to the current estimate. Note this
 * is the sign-flipped half-difference when the half-difference is read as
 * "where the drone sits relative to the centre".
 *
 * Because the setpoint is modeAbs, the position error the controller sees
 * is exactly offX/offY, so the firmware's position PID (kp, ki, kd, with
 * its own velocity limit) does the work instead of the hand-rolled
 * +-centerMaxV velocity clamp. That is the improvement: far more control
 * authority, an integral term against steady disturbances, and no
 * dependence on the drifting estimator origin.
 *
 * Validity: an axis is only re-targeted when BOTH of its sensors return a
 * usable reading. When they do not, the PREVIOUS target is held rather
 * than re-sampling the live estimate -- re-sampling would make the
 * setpoint chase the drone and provide no restoring force at all.
 *
 * Frames: offX/offY are body-frame, estX/estY are world-frame. Yaw is held
 * at holdYawDeg (0 deg) throughout, so the two coincide and the offsets are
 * added axis-wise with no rotation. Measured yaw excursions of +-10 deg
 * leak ~17% between the axes; that is accepted here.
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

#define DEBUG_MODULE "CHAMBER_ABS"

/* Live per-zone ToF distances [mm] owned by the vl53l8cx deck driver
 * (system.c fills vl53l8cxToFDist[sensor][zone]). Read directly so this app
 * needs no driver-side changes: the exposed vl53l8cx.sN log var is a
 * whole-fan average, whose outer zones look at the floor and ceiling in a
 * 50 cm box and are useless for wall centering. */
extern int16_t vl53l8cxToFDist[][16];

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

/* How much of the measured offset to apply per loop. 1.0 = command the full
 * box-centre position every loop (the intended behaviour); lower values
 * de-rate the correction if the ToF turns out too noisy to trust fully.
 * There is deliberately no velocity clamp here -- the firmware position PID
 * already limits its own output. */
static float centerTrim = 1.0f;

/* Ignore an offset smaller than this [m] (sensor noise floor), so the
 * setpoint stops jittering once the drone is centred. 0 disables. */
static float centerDeadbandM = 0.05f;

/* Horizontal speed cap [m/s] while the position controller flies to a
 * modeAbs setpoint. The firmware caps the position PID output at
 * posCtlPid.xVelMax * velMaxOverhead (velMaxOverhead = 1.10, not exposed as
 * a param), and the cf21bl default xVelMax is 1.0 m/s -- i.e. 1.1 m/s, far
 * too fast for a 50 cm box. This app pushes posCtlPid.x/yVelMax down so the
 * ACTUAL cap equals xyVelMax exactly. Applies to takeoff and landing too,
 * since those also hold x/y with modeAbs. Z is unaffected (zVelMax has its
 * own 0.5 m/s floor in the firmware). */
static float xyVelMax = 0.05f;   /* m/s */

/* Must match velMaxOverhead in position_controller_pid.c. */
#define POSCTL_VEL_MAX_OVERHEAD 1.10f

/* No-PC-link trigger: hold a hand this close (mm) over the up-facing
 * vl53l8cx sensor (s8) for this long (ms) to start the mission. */
static uint16_t handTriggerMm     = 300;
static uint16_t handTriggerHoldMs = 500;

/* Landing: constant, gentle descent. Motors cut once estZ drops below
 * landCutoffM, or after landMaxTime_s regardless. */
static float landSpeed   = 0.15f;   /* m/s */
static float landCutoffM = 0.04f;   /* m */

/* Yaw is held at this absolute heading for the whole mission (takeoff,
 * hover, landing). The centering above is expressed in body axes, so the
 * body/chamber axes must stay aligned for it to mean anything. */
static float holdYawDeg = 0.0f;   /* deg */

static const float takeoffTime_s      = 3.0f;
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

/* measured offset from the drone to the box centre [m] (0 if unmeasurable) */
static float offXLog = 0.0f;
static float offYLog = 0.0f;
/* commanded box-centre position, estimator frame [m] */
static float targetXLog = 0.0f;
static float targetYLog = 0.0f;

/* ========================= Helpers ========================= */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* Average of the central 4 zones (5, 6, 9, 10) of one vl53l8cx sensor, in
 * metres. Only zones that actually returned a target (>0) are averaged;
 * returns <0 if none of the four did. Those four zones are the ones aimed
 * straight at the wall -- the outer zones of the 45 deg fan hit the floor
 * and ceiling at this range. */
static float wallDist(int sensor)
{
  static const int zones[4] = {5, 6, 9, 10};
  int32_t sum = 0;
  int n = 0;
  for (int i = 0; i < 4; i++) {
    int16_t d = vl53l8cxToFDist[sensor][zones[i]];
    if (d > 0) { sum += d; n++; }
  }
  if (n == 0) return -1.0f;
  return 0.001f * (float)sum / (float)n;
}

/* Signed distance from the drone to the box centre along one axis [m], from
 * the opposing pair of wall ranges. dNear is the sensor pointing along the
 * POSITIVE axis (front for x, left for y), dFar the negative one.
 *
 *   dNear = h - p, dFar = h + p   (h = half box, p = drone offset from centre)
 *   => (dNear - dFar) / 2 = -p    = the vector from the drone to the centre
 *
 * Returns false when either sensor has no usable reading, in which case the
 * caller must keep its previous target rather than inventing one. */
static bool centreOffset(float dNear, float dFar, float deadband, float *off)
{
  if (dNear < 0.0f || dFar < 0.0f) {
    return false;
  }
  float o = 0.5f * (dNear - dFar);
  if (deadband > 0.0f && o > -deadband && o < deadband) {
    o = 0.0f;
  }
  *off = o;
  return true;
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
  paramVarId_t idXVelMax = paramGetVarId("posCtlPid", "xVelMax");
  paramVarId_t idYVelMax = paramGetVarId("posCtlPid", "yVelMax");

  /* Forces the cap to be (re)applied on the first loop. */
  float appliedXyVelMax = -1.0f;

  uint32_t stateStartTick = xTaskGetTickCount();
  uint32_t deckSeenTick = 0;
  bool deckSeenStarted = false;
  bool usdLoggingActive = false;

  uint32_t handSeenTick = 0;
  bool handSeenStarted = false;

  float takeoffX = 0.0f, takeoffY = 0.0f;
  /* Box-centre target in estimator coordinates, recomputed each hover loop
   * from the wall ranges. Seeded at the takeoff point on HOVER entry and
   * held unchanged on any axis whose sensor pair is momentarily invalid. */
  float targetX = 0.0f, targetY = 0.0f;
  /* Landing returns to the captured takeoff point, not to wherever the
   * centering left us -- the chamber mission is "take off and land on the
   * same spot". */

  DEBUG_PRINT("Set estimator to Kalman\n");
  paramSetInt(idEstimator, 2);

  DEBUG_PRINT("Brushless chamber hover app started (abs box-centre hold)\n");

  while (1) {
    vTaskDelay(M2T(loopDt_ms));

    /* Keep the firmware's horizontal speed cap in sync with chamber.xyVelMax,
     * so it can be retuned live over the radio without a reflash. */
    if (xyVelMax != appliedXyVelMax) {
      float lim = xyVelMax / POSCTL_VEL_MAX_OVERHEAD;
      paramSetFloat(idXVelMax, lim);
      paramSetFloat(idYVelMax, lim);
      appliedXyVelMax = xyVelMax;
      DEBUG_PRINT("Horizontal speed cap -> %.3f m/s\n", (double)xyVelMax);
    }

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
        offXLog = 0.0f;
        offYLog = 0.0f;
        targetXLog = 0.0f;
        targetYLog = 0.0f;

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
          targetX = takeoffX;
          targetY = takeoffY;
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

        /* --- wall distances (central 4 zones of each sensor) --- */
        float dFront = wallDist(WALL_SENSOR_FRONT);
        float dBack  = wallDist(WALL_SENSOR_BACK);
        float dLeft  = wallDist(WALL_SENSOR_LEFT);
        float dRight = wallDist(WALL_SENSOR_RIGHT);
        dFrontLog = dFront; dBackLog = dBack; dLeftLog = dLeft; dRightLog = dRight;

        /* Where is the box centre, relative to us, right now? */
        float offX, offY;
        bool okX = centreOffset(dFront, dBack,  centerDeadbandM, &offX);
        bool okY = centreOffset(dLeft,  dRight, centerDeadbandM, &offY);

        /* Re-target only the axes we can actually measure. An axis with a
         * dead sensor keeps its previous target: re-seeding it from the live
         * estimate would make the setpoint follow the drone and kill the
         * restoring force entirely. */
        if (okX) {
          targetX = estX + centerTrim * offX;
        } else {
          offX = 0.0f;
        }
        if (okY) {
          targetY = estY + centerTrim * offY;
        } else {
          offY = 0.0f;
        }
        offXLog = okX ? offX : 0.0f;
        offYLog = okY ? offY : 0.0f;
        targetXLog = targetX;
        targetYLog = targetY;

        /* Absolute position hold on the measured box centre. The position
         * error the controller sees is exactly offX/offY, so the firmware
         * position PID -- not a hand-tuned velocity clamp -- sets the
         * response, and the drifting estimator origin drops out. */
        setAbsSetpoint(&setpoint, targetX, targetY, takeoffHeight, holdYawDeg);
        commanderSetSetpoint(&setpoint, 3);

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
PARAM_ADD(PARAM_FLOAT, xyVelMax, &xyVelMax)
PARAM_ADD(PARAM_FLOAT, centerTrim, &centerTrim)
PARAM_ADD(PARAM_FLOAT, centerDbM, &centerDeadbandM)
PARAM_ADD(PARAM_FLOAT, holdYawDeg, &holdYawDeg)
PARAM_ADD(PARAM_UINT16, handMm, &handTriggerMm)
PARAM_ADD(PARAM_UINT16, handHoldMs, &handTriggerHoldMs)
PARAM_ADD(PARAM_FLOAT, landSpeed, &landSpeed)
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
LOG_ADD(LOG_FLOAT, offX, &offXLog)
LOG_ADD(LOG_FLOAT, offY, &offYLog)
LOG_ADD(LOG_FLOAT, tgtX, &targetXLog)
LOG_ADD(LOG_FLOAT, tgtY, &targetYLog)
LOG_GROUP_STOP(chamber)
