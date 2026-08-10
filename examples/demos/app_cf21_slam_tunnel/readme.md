# CF2.1 SLAM-tunnel mission (app layer)

Standalone app-layer mission for the Crazyflie 2.1 with a Flow deck v2
and a Multiranger deck attached: take off, then creep forward down a
tunnel -- only forward motion, no corner negotiation or wall-following
-- gated purely on whether the front sensor currently reports it safe
to keep going, with light lateral centering from the left/right beams.
Lands automatically on detecting the tunnel has opened up (exit) or if
progress stalls too long (stuck). Runs entirely on-board via
`commanderSetSetpoint` -- no radio connection needed once triggered,
only to set the start param and (optionally) retrieve logs afterward.

**This app does not run SLAM itself.** The STM32 doesn't have the
RAM/compute for real-time loop closure or map optimization. It flies
using only the flow deck's live (relative, drifting) position estimate
and the multiranger's raw ranges for its immediate go/no-go decision,
and logs both, at rate, to the uSD card for the whole flight. Turning
that into a point cloud, evaluating mapping quality, or checking
localization drift against ground truth happens OFFLINE afterward --
see `build_pointcloud_from_log.m` in the companion MATLAB project.

Position/attitude and the multiranger's 5 ranges are recorded to the
onboard microSD card for the duration of the flight (`usd.logging` is
turned on right as takeoff begins and off once landed) -- see
`sdcard/config.txt` in this folder, already set up with "enable on
startup" = `0` since this app controls start/stop itself.

## Trigger

Set param `slamTunnel.start = 1` (e.g. via cfclient) once decks are
attached and the aircraft is on the ground, or hold a hand within
`slamTunnel.handMm` of the multiranger's top sensor for
`slamTunnel.handHoldMs` -- no PC link needed for the hand-wave path. It
clears itself automatically after landing or on abort/crash detection
(`sys.isTumbled`). It does **not** auto-start on boot by default -- see
the comment above `startMission` in the source if you want that.

## How it decides to move forward

Forward speed ramps from 0 at `slamTunnel.dStop` (front distance) up to
`slamTunnel.vCruise` at `slamTunnel.dSlow` -- the same ramp shape as the
`local_gate` helper in this project's `app_tunnel_bug.c` reference.
Lateral centering nudges sideways using `kCenter * (rLeft - rRight)`,
clamped to `yCenterMax`. There is no corner-turning or wall-following:
if the front stays blocked, the UAV just holds/stops, and after
`stuckTimeoutS` of no progress it lands rather than trying to maneuver
around anything.

## How it decides it has reached the end

Same "openCount >= 3 of 4" idea as `app_tunnel_bug.c`: front, left,
right, and up are each checked against `exitFrontMm`/`exitSideMm`/
`exitUpMm`, and if at least 3 of the 4 read open for `exitHoldS`
seconds continuously, the tunnel is considered exited and it lands.
`maxAdvanceS` is a hard time cap regardless, in case neither condition
ever triggers.

## Tunable params (no reflash needed)

- `slamTunnel.takeoffH` -- takeoff height, m (default 0.5)
- `slamTunnel.vCruise` / `dStop` / `dSlow` -- forward-speed gate
- `slamTunnel.kCenter` / `yCenterMax` -- lateral centering
- `slamTunnel.exitFrontMm` / `exitSideMm` / `exitUpMm` / `exitHoldS` -- exit detection
- `slamTunnel.stuckTimeoutS` / `maxAdvanceS` -- safety aborts
- `slamTunnel.handMm` / `handHoldMs` -- hand-wave trigger tuning
- `slamTunnel.landSpeed` / `landCutoffM` -- landing descent rate and motor-cut height

## Build

You must have the required tools to build the
[Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

Clone the repo with `--recursive`. If you didn't, pull submodules with:
```
git submodule update --init --recursive
```

Then build and bootload:
```
make -j$(nproc)
make cload
```
