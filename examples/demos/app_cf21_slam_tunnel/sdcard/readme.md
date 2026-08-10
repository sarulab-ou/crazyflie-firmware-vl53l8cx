# microSD card config

`config.txt` in this folder is a template for the Crazyflie's onboard
uSD-card logger. Copy it to the **root** of the microSD card, keeping
the name `config.txt` exactly. It is not read from this project
folder -- it only takes effect once it's on the card itself.

Logs position/attitude + the multiranger's 5 ranges at 50 Hz, synced to
the stabilizer loop. "Enable on startup" is `0` because
`app_cf21_slam_tunnel.c` turns `usd.logging` on/off itself, right around
takeoff and landing.

This is the raw data build_pointcloud_from_log.m (in the companion
MATLAB project) turns into a point cloud + trajectory afterward -- the
firmware doesn't build the map itself, it only logs the ingredients.
