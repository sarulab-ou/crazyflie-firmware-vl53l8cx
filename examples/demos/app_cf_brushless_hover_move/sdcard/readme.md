# microSD card config -- Brushless / vl53l8cx

Copy `config.txt` in this folder to the **root** of the microSD card,
keeping the name `config.txt` exactly. It is not read from this project
folder -- it only takes effect once it's on the card itself.

## Two streams, one file

`usddeck`'s file format multiplexes named "event" streams into a single
binary file per session (the filename/buffer are per-session, not
per-stream) -- so genuinely separate files on the card isn't something
the driver supports. What this config does instead:

- `fixedFrequency`: the 9 vl53l8cx ranges, sampled every control-loop
  tick the firmware's synchronous-stabilizer hook fires (50 Hz here).
- `pose`: position/attitude, triggered explicitly by
  `app_cf_brushless_hover_move.c` calling `eventTrigger(&eventTrigger_pose)`
  once per app loop tick (also ~50 Hz).

`sd_log_to_csv.py` decodes a file with multiple event types into
**separate CSVs automatically** (one per stream name) -- so you still
end up with two files, `..._fixedFrequency.csv` and `..._pose.csv`,
just derived from one recording rather than two separate on-card files.

## Buffer size

Bitcraze's own reference point (10 variables at 1 kHz needs only a
512-byte buffer) implies far more headroom than this setup needs:
9 ranges (float, 4 B each) + ~10 B/row overhead $\approx$ 46 B/row, plus
6 pose vars $\approx$ 34 B/row, both at 50 Hz $\approx$ 4 KB/s combined --
roughly 1/12th the throughput of Bitcraze's reference case. 2048 bytes
here is generous headroom, not a requirement; 512 would likely be fine
too if RAM on the target build is tight.

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
