# Brushless / vl53l8cx hover/move/hover mission (app layer)

Standalone app-layer mission for the Crazyflie Brushless with the
vl53l8cx deck (9-sensor build) and a flow deck: arm, take off to 0.3 m,
hover in place for 5 s, move 0.5 m forward, hover for another 5 s, then
land -- all on-board, no radio connection needed once triggered.

Range data (`vl53l8cx.s0`..`s8`) and position/attitude are recorded to
the onboard microSD card as two separate logical streams -- see
`sdcard/config.txt` and `sdcard/readme.md` for the file to copy to the
card and the exact sensor index -> physical direction mapping. Ranges
are **never** read over the radio link for this deck (risk of a
watchdog reboot, per the firmware author) -- only over uSD.

## Trigger

Set param `hoverMove.start = 1` (e.g. via cfclient), or hold a hand
within `hoverMove.handMm` of the up-facing sensor (`vl53l8cx.s8`) for
`hoverMove.handHoldMs` -- no PC link needed for the hand-wave path.
Clears itself automatically after landing or on abort/crash detection
(`sys.isTumbled`). Does **not** auto-start on boot by default.

## Arming

Unlike the CF2.1 app in this project, this one explicitly arms via
`supervisorRequestArming`/`supervisorIsArmed` before takeoff, with a
3 s timeout (`APP_ARMING` state) -- if arming doesn't succeed in time,
the mission aborts back to idle rather than retrying forever.

## Tunable params (no reflash needed)

- `hoverMove.takeoffH` -- takeoff height, m (default 0.3)
- `hoverMove.moveDistM` -- forward move distance, m (default 0.5)
- `hoverMove.hoverTimeS` -- duration of each hover phase, s (default 5)
- `hoverMove.moveTimeS` -- duration of the forward move, s (default 5)
- `hoverMove.handMm` / `hoverMove.handHoldMs` -- hand-wave trigger tuning
- `hoverMove.landSpeed` / `hoverMove.landCutoffM` -- landing descent
  rate and motor-cut height

## Build

You must have the required tools to build the
[Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

```
git submodule update --init --recursive
make -j$(nproc)
make cload
```
