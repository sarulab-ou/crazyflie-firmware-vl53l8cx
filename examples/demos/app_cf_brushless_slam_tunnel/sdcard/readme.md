# microSD card config -- Brushless / vl53l8cx

Copy `config.txt` in this folder to the **root** of the microSD card,
keeping the name `config.txt` exactly. It is not read from this project
folder -- it only takes effect once it's on the card itself.

## Two streams, one file

`usddeck`'s file format multiplexes named "event" streams into a single
binary file per session -- so genuinely separate files on the card
isn't something the driver supports. What this config does instead:

- `fixedFrequency`: the 9 vl53l8cx ranges, sampled every control-loop
  tick the firmware's synchronous-stabilizer hook fires (50 Hz here).
- `pose`: position/attitude, triggered explicitly by
  `app_cf_brushless_slam_tunnel.c` calling `eventTrigger(&eventTrigger_pose)`
  once per app loop tick (also ~50 Hz).

`sd_log_to_csv.py` decodes a file with multiple event types into
**separate CSVs automatically** (one per stream name) -- so you still
end up with two files, `..._fixedFrequency.csv` and `..._pose.csv`,
just derived from one recording rather than two separate on-card files.

This is the raw data build_pointcloud_from_log.m (in the companion
MATLAB project) turns into a point cloud + trajectory afterward -- the
firmware doesn't build the map itself, it only logs the ingredients.

## Sensor index -> physical position (vl53l8cx, 9-sensor build)

| index | direction  |
|-------|------------|
| s0    | front-left |
| s1    | center     |
| s2    | front-right|
| s3    | front-up   |
| s4    | front-down |
| s5    | left       |
| s6    | right      |
| s7    | back       |
| s8    | up         |

The decoded CSV columns come out named literally `vl53l8cx.s0` ...
`vl53l8cx.s8` (that's what's stored in the file's own header) --
`sd_log_to_csv.py` renames these to the descriptive labels above
automatically when decoding.
