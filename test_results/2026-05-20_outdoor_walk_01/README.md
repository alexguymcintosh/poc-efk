# 2026-05-20 — First Outdoor Walk Test

First successful end-to-end outdoor walk with the magnetometer fused into the EKF.

## Hardware as tested
- Raspberry Pi Pico 2 (RP2350)
- MPU-6050 IMU (gyro bias calibrated on boot)
- HMC5883L magnetometer (hard-iron bias from 100 stationary samples)
- NEO-M9N GPS (auto-baud locked at 38400)

## Test sequence
Followed `COMMANDS.md` test protocol. Both bags captured cleanly:

| Bag | Duration | Total msgs | Purpose |
|---|---|---|---|
| `bags/stationary_01` | 31.7 s | 5,778 | Smoke-test, confirm all five topics flowing before walking |
| `bags/walk_01` | 70.8 s | 12,893 | Out-and-back walk, ~74 m actual distance |

All topics at expected rates during the walk: IMU raw 100 Hz, IMU+mag 50 Hz, EKF 30 Hz, GPS 1 Hz.

## Result — analyse_bag.py output on walk_01
```
EKF total distance travelled : 175.12 m
EKF start->end displacement  :   5.66 m
GPS-raw total path length    :  73.93 m
GPS-raw start->end displacement: 5.44 m
```

(The "max drift in first 10s" stat assumes stationary; the rig was walking, so ignore.)

## Interpretation
**Working:**
- Macro trajectory matches GPS (same out-and-back arc — see `analysis/combined_paths.png`)
- EKF endpoint within 0.2 m of GPS endpoint
- Full pipeline alive end-to-end for 70+ seconds, no dropouts

**Broken:**
- EKF over-counts distance 2.4× (175 m vs 74 m)
- Sawtooth zigzag on EKF path, ~2 m peak-to-peak — visible in both `combined_paths.png` and `ekf_xy_time.png`

## Root cause
The EKF has no velocity source. Between 1 Hz GPS updates the EKF propagates position using its internal velocity estimate, which is updated to fit GPS deltas rotated by the noisy mag heading.

Raw post-calibration magnetometer values: roughly ±2 with frame-to-frame jumps of similar magnitude. Hard-iron-only calibration (100 stationary samples) does not capture the orientation-dependent soft-iron effects. atan2 yaw bounces → velocity vector rotates → fake lateral motion in the EKF.

## Incidents during session
- `test_mag_01`: empty bag — recorder started before pipeline launched. Mitigation: COMMANDS.md now requires a smoke-test bag (steps 26-27) before any walk record.
- `test_mag_02`: only `/odometry/filtered` captured; Pico GPS UART had silently stopped emitting `GPS:` lines. Pico replug + reboot fixed it. Pipeline-level health checks won't catch this — Pico-side watchdog would.

## Next session direction
Bring up a Pixhawk 6C as a higher-quality comparison baseline. The 6C's onboard EKF2/EKF3 plus aerospace-grade IMUs skip the `robot_localization` tuning problem entirely. Once the 6C path is up, come back and tune the raw-hardware stack against the 6C output:

1. Figure-8 magnetometer calibration (sphere fit for hard + soft iron) — biggest lever
2. Bump mag orientation covariance in `mag_node.py` from 0.1 → ~1.0
3. Consider re-enabling linear acceleration fusion now that mag heading exists

## Files

```
2026-05-20_outdoor_walk_01/
├── README.md              # this file
├── analysis/
│   ├── combined_paths.png   # GPS raw, GPS odom, EKF — same trajectory, EKF noisy
│   ├── ekf_xy_time.png      # EKF x and y vs time
│   ├── imu_gyro_z_time.png  # gyro z (yaw rate)
│   └── summary.txt          # raw analyse_bag.py stdout
└── bags/
    ├── walk_01/             # the walk recording — copy to ~/ros2_ws/bags/walk_01/ to replay
    └── stationary_01/       # the smoke-test recording
```

To replay analysis from these bags:
```bash
cp -r bags/walk_01 ~/ros2_ws/bags/
python3 ../../analyse_bag.py ~/ros2_ws/bags/walk_01
```
