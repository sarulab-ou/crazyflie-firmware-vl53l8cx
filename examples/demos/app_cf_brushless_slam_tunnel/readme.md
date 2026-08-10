# Brushless / vl53l8cx SLAM-tunnel mission (app layer)

Standalone app-layer mission for the Crazyflie Brushless with the
vl53l8cx deck (9-sensor build) and a flow deck: arm, take off, then
creep forward down a tunnel -- only forward motion, no corner
negotiation or wall-following -- gated purely on whether the front fan
(frontleft/center/frontright) currently reports it safe to keep going,
with light lateral centering from the left/right beams. Lands
automatically on detecting the tunnel has opened up (exit) or if
progress stalls too long (stuck). This is the Brushless/vl53l8cx
counterpart to `app_cf21_slam_tunnel.c` -- see that file's readme for
the shared design rationale.

**This app does not run SLAM itself** -- same limitation/design as the
CF2.1 version: it flies on reactive range-gating alone and logs the raw
pose+range ingredients to the uSD card; the actual point cloud / mapping
quality / localization-drift analysis happens OFFLINE via
`build_pointcloud_from_log.m` in the companion MATLAB project.

Range data (`vl53l8cx.s0`..`s8`) and position/attitude are recorded to
the onboard microSD card as two separate logical streams -- see
`sdcard/config.txt` and `sdcard/readme.md` for the file to copy to the
card and the exact sensor index -> physical direction mapping. Ranges
are **never** read over the radio link for this deck (risk of a
watchdog reboot, per the firmware author) -- only over uSD, and via
`logGetFloat()` locally on the Crazyflie itself for this app's own
go/no-go decision (also not over radio).

## Trigger

Set param `slamTunnel.start = 1` (e.g. via cfclient), or hold a hand
within `slamTunnel.handMm` of the up-facing sensor (`vl53l8cx.s8`) for
`slamTunnel.handHoldMs` -- no PC link needed for the hand-wave path.
Clears itself automatically after landing or on abort/crash detection
(`sys.isTumbled`). Does **not** auto-start on boot by default.

## Arming

Same as `app_cf_brushless_hover_move.c`: explicitly arms via
`supervisorRequestArming`/`supervisorIsArmed` before takeoff, with a
3 s timeout (`APP_ARMING` state) -- if arming doesn't succeed in time,
the mission aborts back to idle rather than retrying forever.

## How it decides to move forward / reached the end

Same logic as the CF2.1 version, just fed from the front fan's
conservative minimum (`min(frontleft, center, frontright)`) instead of
a single beam -- see that app's readme for the forward-gating and exit-
detection details.

## Tunable params (no reflash needed)

- `slamTunnel.takeoffH` -- takeoff height, m (default 0.3)
- `slamTunnel.vCruise` / `dStop` / `dSlow` -- forward-speed gate
- `slamTunnel.kCenter` / `yCenterMax` -- lateral centering
- `slamTunnel.exitFrontMm` / `exitSideMm` / `exitUpMm` / `exitHoldS` -- exit detection
- `slamTunnel.stuckTimeoutS` / `maxAdvanceS` -- safety aborts
- `slamTunnel.handMm` / `handHoldMs` -- hand-wave trigger tuning
- `slamTunnel.landSpeed` / `landCutoffM` -- landing descent rate and motor-cut height

## Build

You must have the required tools to build the
[Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

```
git submodule update --init --recursive
make -j$(nproc)
make cload
```
