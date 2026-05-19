# EKF Test Commands

One step at a time. Each numbered block is ONE command with ONE purpose. Run in order. If a step fails, stop and debug — don't skip ahead.

All steps run on the laptop host. No docker.

---

## 1. Is the Pico connected?
```bash
lsusb | grep -i "2e8a\|raspberry\|pico"
```
- `2e8a:0009` → running normally → go to step 2
- `2e8a:000f` → bootloader mode → go to step 1b
- nothing → check USB cable, replug

## 1b. Flash firmware (only if step 1 showed bootloader)
```bash
cp ~/system/alex/agent/rob/rover/p1/loc/ekf/pico_firmware/build/imu.uf2 /media/$USER/RP2350/
```
Pico reboots automatically. Go back to step 1.

## 2. Does the serial port exist?
```bash
ls /dev/ttyACM*
```
- `/dev/ttyACM0` → proceed
- `No such file` → Pico isn't running firmware, go back to step 1

## 3. Verify raw serial data
```bash
head -20 < /dev/ttyACM0
```
Expect lines starting with `IMU:`, `MAG:`, `GPS:`. If it stalls, Ctrl-C and retry.

## 4. Check for already-running ROS2 nodes
```bash
ros2 node list
```
- empty output → go to step 5
- any nodes listed → go to step 4b first

## 4b. Kill stale nodes (only if step 4 listed any)
```bash
pkill -f serial_parser_node; pkill -f imu_node; pkill -f mag_node; pkill -f gps_node; pkill -f ekf_node; pkill -f gps_to_odom_node; pkill -f static_transform_publisher; pkill -f foxglove_bridge; sleep 0.5
```
Then re-run step 4 to confirm it's empty.

## 5. Launch the pipeline — Terminal 1 (stays open)
```bash
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pico_bridge pico_bridge.launch.py
```
Expect log lines from `serial_parser`, `imu_node`, `mag_node`, `gps_node`, `gps_to_odom_node`, `ekf_filter_node`. Leave this terminal running. **Open a new terminal for the next steps.**

---

## 6. Source ROS2 in the new terminal (once per new terminal)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
```

## 7. Confirm all nodes are running
```bash
ros2 node list
```
Expect: `/serial_parser`, `/imu_node`, `/mag_node`, `/gps_node`, `/gps_to_odom`, `/ekf_filter_node`, `/tf_base_to_imu`, `/tf_base_to_gps`.

## 8. Check raw IMU stream
```bash
ros2 topic echo /pico/imu_raw --once
```
Expect a `std_msgs/String` line with comma-separated floats.

## 9. Check raw MAG stream
```bash
ros2 topic echo /pico/mag_raw --once
```
Expect a `std_msgs/String` starting `MAG:` with three floats.

## 10. Check parsed IMU
```bash
ros2 topic echo /imu/data_raw --once
```
Expect a `sensor_msgs/Imu` with non-zero `angular_velocity` and `linear_acceleration`.

## 11. Check magnetometer-derived heading
```bash
ros2 topic echo /imu/data --once
```
Expect a `sensor_msgs/Imu` with non-zero `orientation.z` / `orientation.w` (yaw quaternion).

## 12. Check GPS fix (needs outdoor sky view)
```bash
ros2 topic echo /fix --once
```
Expect a `sensor_msgs/NavSatFix` with valid lat/lon. No fix → take the rig outside and wait ~30s.

## 13. Check GPS odometry
```bash
ros2 topic echo /odometry/gps --once
```
Expect a `nav_msgs/Odometry`. `position.x/y` should be near 0 at the datum (first GPS fix).

## 14. Check fused EKF output
```bash
ros2 topic echo /odometry/filtered --once
```
Expect a `nav_msgs/Odometry` — fused pose and orientation.

---

## 15. IMU rate
```bash
ros2 topic hz /imu/data_raw
```
Expect ~100Hz. Ctrl-C to stop.

## 16. Mag-heading rate
```bash
ros2 topic hz /imu/data
```
Expect ~50Hz. Ctrl-C to stop.

## 17. EKF output rate
```bash
ros2 topic hz /odometry/filtered
```
Expect ~30Hz. Ctrl-C to stop.

---

## 18. (Optional) Visualise topology
```bash
ros2 run rqt_graph rqt_graph
```

## 19. Foxglove bridge — Terminal 3 (stays open)
```bash
source /opt/ros/jazzy/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
Connect Foxglove Studio to `ws://localhost:8765`.

Panels to add in Foxglove:
- Map → `/fix`
- Plot → `/odometry/filtered` pose.x and pose.y
- Plot → `/imu/data_raw` angular_velocity.z
- Raw Messages → `/imu/data` (orientation quaternion)

---

## 20. Make bags directory (only first time)
```bash
mkdir -p ~/ros2_ws/bags
```

---

# === TEST PROTOCOL ===

Goal: never start a walk bag without proof the pipeline is actually publishing. Steps 21–25 run live rate checks. Step 26 is a short stationary smoke-test bag — claude verifies all topics captured non-zero messages BEFORE you walk. The walk only starts at step 28.

Sequence: 21 → 22 → 23 → 24 → 25 (paste all five outputs to claude) → 26 → 27 (paste to claude) → 28 → 29 → 30.

---

## Pre-bag health checks (paste each output to claude)

Each command: let it run ~10 seconds, Ctrl-C, move to the next.

## 21. IMU raw rate (expect ~100 Hz)
```bash
ros2 topic hz /imu/data_raw
```

## 22. IMU + mag heading rate (expect ~50 Hz)
```bash
ros2 topic hz /imu/data
```

## 23. GPS rate (expect ~1 Hz — use window of 5 so average appears in ~5s)
```bash
ros2 topic hz /fix -w 5
```

## 24. EKF output rate (expect ~30 Hz)
```bash
ros2 topic hz /odometry/filtered
```

## 25. Confirm GPS has a real fix
```bash
ros2 topic echo /fix --once
```
Look for `status.status: 0` (or higher) and non-zero `latitude`/`longitude`. If `status: -1` → no fix yet, wait outdoors.

**Paste 21–25 outputs to claude. Claude confirms green-light before step 26.**

---

## Stationary smoke test (verify the bag actually captures)

## 26. Record a 30-second stationary bag
```bash
ros2 bag record /fix /imu/data_raw /imu/data /odometry/filtered /odometry/gps -o ~/ros2_ws/bags/stationary_01
```
Hold the rig still. Ctrl-C after ~30 seconds. Increment the number (`stationary_02`, …) for repeats.

## 27. Show smoke-test bag message counts
```bash
ros2 bag info ~/ros2_ws/bags/stationary_01
```
**Paste output to claude.** Every topic must show a non-zero count proportional to its rate (~3000 IMU raw, ~1500 IMU data, ~30 fix, ~900 filtered, ~30 odom gps for 30s).

If any topic = 0 → pipeline broken upstream. DO NOT walk. Debug first (`head -20 < /dev/ttyACM0`, `ros2 node list`, restart pipeline).

---

## Walk test

## 28. Record the walk
```bash
ros2 bag record /fix /imu/data_raw /imu/data /odometry/filtered /odometry/gps -o ~/ros2_ws/bags/walk_01
```
Walk the planned route. Ctrl-C to stop. Increment the number for repeats.

## 29. Verify walk bag captured
```bash
ros2 bag info ~/ros2_ws/bags/walk_01
```
Paste to claude. Same non-zero-counts rule as step 27.

## 30. Analyse the walk
```bash
python3 ~/system/alex/agent/rob/rover/p1/loc/ekf/analyse_bag.py ~/ros2_ws/bags/walk_01
```
Produces `combined_paths.png`, `ekf_xy_time.png`, `imu_gyro_z_time.png` in your current directory. Paste the summary stats to claude.

---

## When to rebuild

`~/ros2_ws/src/pico_bridge` is a symlink to the project repo, built with `--symlink-install`. Python edits in `ekf/ros2_ws/pico_bridge/*.py` are picked up automatically — no rebuild needed.

**Rebuild only if you change:** `setup.py` (entry points), `package.xml`, `launch/*.py`, or `config/*.yaml`.

### Rebuild ROS2 package
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select pico_bridge
```
Then `source ~/ros2_ws/install/setup.bash` in any open terminals.

### Rebuild Pico firmware (only if `pico_firmware/*.cpp` changed)
```bash
cd ~/system/alex/agent/rob/rover/p1/loc/ekf/pico_firmware/build && make -j
```
Then put the Pico in bootloader mode (hold BOOTSEL while replugging USB) and flash per step 1b.
