# Terminal Command Reference — EKF Localisation Session

Commands in the order you run them. Every session follows this sequence.

---

## Stage 1 — Check Hardware State

### Check what the Pico looks like to the laptop
```bash
lsusb | grep -i 2e8a
```
**What it is:** `lsusb` lists all USB devices. We filter for Raspberry Pi's vendor ID `2e8a`.
**What you're looking for:**
- `2e8a:000f` = Pico in **boot mode** (BOOTSEL was held — ready to receive new firmware)
- `2e8a:0009` = Pico in **running mode** (firmware loaded, streaming data)
- Nothing = not plugged in or not recognised

**When to use:** First thing every session. Also after flashing to confirm it switched from `000f` → `0009`.

---

## Stage 2 — Build New Firmware

Run these once per session when firmware has changed.

### Delete the stale build directory
```bash
rm -rf /workspace/alex/agent/rob/rover/p1/loc/ekf/pico_firmware/build
```
**What it is:** `rm -rf` removes a directory and everything inside it. Required because CMake bakes absolute paths into its cache — if the build dir exists from a different machine or path, cmake fails silently.
**When to use:** Before every cmake run. Safe to run even if build/ doesn't exist.

### Configure the build
```bash
cd /workspace/alex/agent/rob/rover/p1/loc/ekf/pico_firmware && mkdir build && cd build
cmake -DPICO_BOARD=pico2 -DCMAKE_TOOLCHAIN_FILE=$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake ..
```
**What it is:** `cmake` reads `CMakeLists.txt` and generates the actual Makefile. The flags tell it: target is Pico 2 (RP2350), use the ARM cross-compiler (compiles code for the Pico's CPU, not your laptop's CPU). The `..` means "CMakeLists.txt is one directory up."
**When to use:** Once after deleting build/.

### Compile the firmware
```bash
make -j
```
**What it is:** Runs the compiler. `-j` uses all CPU cores in parallel — faster. Produces `imu.uf2` in the build directory.
**When to use:** After cmake. Takes ~30 seconds.

---

## Stage 3 — Flash Firmware to Pico

### Put Pico in boot mode
1. Hold the **BOOTSEL** button on the Pico
2. Unplug the USB cable
3. Plug the USB cable back in
4. Release BOOTSEL

The Pico mounts as a USB drive called `RP2350`. Run `lsusb | grep 2e8a` — should show `000f`.

### Copy the firmware file to the Pico
```bash
cp /workspace/alex/agent/rob/rover/p1/loc/ekf/pico_firmware/build/imu.uf2 /media/$USER/RP2350/
```
**What it is:** Copies the compiled firmware file to the Pico's USB drive. The Pico automatically reboots into the new firmware and unmounts the drive.
**`$USER`** is automatically replaced with your Linux username (e.g. `knucky-rover-team`).
**When to use:** After `make -j` succeeds and Pico is in boot mode.

### Confirm Pico is running the new firmware
```bash
lsusb | grep -i 2e8a
```
Should now show `0009` (running). The drive disappears — that's normal.

---

## Stage 4 — Verify Raw Serial Output

### Read raw lines from the Pico
```bash
head -20 < /dev/ttyACM0
```
**What it is:** Opens the USB serial port and prints the first 20 lines. The Pico streams CSV lines here before any ROS software is involved.
**What you're looking for:**
```
IMU: MPU-6050 awake.
IMU: calibrating gyro...
IMU: gyro bias gx=+0.0012 ...
MAG: HMC5883L found at 0x1E     ← or QMC5883L at 0x0D
GPS: locked baud 57600
IMU:0.1234,0.0012,...
MAG:245.3,88.1,-412.7
GPS:$GNGGA,...
```
**When to use:** Immediately after firmware flashes and Pico switches to `0009`. This is your first sanity check — before launching any ROS nodes.

---

## Stage 5 — Build the ROS2 Package

Run this when Python nodes have changed (not every session — only after code edits).

### Build the pico_bridge package
```bash
cd ~/ros2_ws && colcon build --packages-select pico_bridge
```
**What it is:** `colcon` is ROS2's build tool. It compiles/installs the Python nodes in `ros2_ws/`. `--packages-select pico_bridge` only rebuilds our package, not everything.
**When to use:** After editing any `.py` file in `ros2_ws/pico_bridge/` or after changing `ekf.yaml`.

### Load the new build into your shell
```bash
source ~/ros2_ws/install/setup.bash
```
**What it is:** Adds the freshly built package to your PATH so ROS2 can find it. Must run in any new terminal before using `ros2` commands.
**When to use:** After every `colcon build`, and at the start of every new terminal window.

---

## Stage 6 — Launch the Full Pipeline

### Kill any leftover nodes from a previous session
```bash
pkill -f serial_parser_node; pkill -f imu_node; pkill -f gps_node
pkill -f ekf_node; pkill -f gps_to_odom_node; pkill -f static_transform_publisher
pkill screen; pkill -f foxglove_bridge
```
**What it is:** `pkill -f` kills any process whose command line matches that string. Cleans up stale nodes so the new launch doesn't fight with old ones.
**When to use:** Start of every session before launching.

### Launch the sensor pipeline
```bash
cd ~/ros2_ws && source install/setup.bash && ros2 launch pico_bridge pico_bridge.launch.py
```
**What it is:** Starts all nodes at once — serial_parser, imu_node, gps_node, mag_node, gps_to_odom_node, ekf_filter_node. Leave this terminal open and watch for errors.
**When to use:** After hardware is confirmed working (Stage 4).

---

## Stage 7 — Verify Each Topic is Alive

Run these in a **second terminal** while the pipeline is running.

### Check a single message on each topic (in this order)
```bash
source ~/ros2_ws/install/setup.bash

ros2 topic echo /pico/imu_raw --once       # raw IMU CSV from serial parser
ros2 topic echo /pico/mag_raw --once       # raw MAG CSV from serial parser
ros2 topic echo /imu/data_raw --once       # processed IMU (Imu message)
ros2 topic echo /imu/data --once           # IMU with orientation from mag
ros2 topic echo /fix --once                # GPS fix (needs outdoor signal)
ros2 topic echo /odometry/gps --once       # GPS converted to local x/y
ros2 topic echo /odometry/filtered --once  # EKF fused output
```
**What it is:** `ros2 topic echo` prints one message from a topic and exits. `--once` stops after one message instead of streaming forever.
**What you're looking for:** Data at each stage. If a topic is silent, the node feeding it has a problem.

### Check topic rates
```bash
ros2 topic hz /imu/data_raw        # expect ~100Hz
ros2 topic hz /imu/data            # expect ~100Hz
ros2 topic hz /fix                 # expect ~1Hz (GPS)
ros2 topic hz /odometry/filtered   # expect ~30Hz (EKF)
```
**What it is:** Prints how many messages per second are arriving. A rate of 0 means the node is not publishing.

---

## Stage 8 — Visualise

### Draw the live node/topic graph
```bash
ros2 run rqt_graph rqt_graph
```
**What it is:** Opens a GUI window showing every running ROS node as a box and every topic as an arrow between boxes. This is your "did everything connect correctly" picture. Refresh it by clicking the circular arrow button in the window.
**When to use:** After launching the pipeline to confirm the graph matches the architecture diagram.

### Start the Foxglove bridge
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
**What it is:** Starts a WebSocket server on port 8765. Foxglove Studio (the desktop app) connects to it and shows live sensor data as plots, maps, and 3D views.
**When to use:** After verifying topics are alive. Open Foxglove Studio → Connect → `ws://localhost:8765`.

**Foxglove panels to add:**
- Map → `/fix` — GPS dot on satellite map
- Plot → `/odometry/filtered` position x and y — EKF path
- Plot → `/imu/data_raw` angular_velocity.z — yaw rate
- Raw Messages → `/imu/data` — check orientation quaternion

---

## Stage 9 — Record and Analyse Data

### Record a bag (run for 60s stationary, then 40m walk)
```bash
mkdir -p ~/ros2_ws/bags
ros2 bag record /fix /imu/data_raw /imu/data /odometry/filtered /odometry/gps \
  -o ~/ros2_ws/bags/test_mag_01
```
**What it is:** Records all messages on those topics to a file. `Ctrl+C` to stop recording.
**When to use:** Once pipeline is confirmed working — stationary test first, then walk test.

### Analyse the bag
```bash
source /opt/ros/jazzy/setup.bash
python3 /workspace/alex/agent/rob/rover/p1/loc/ekf/analyse_bag.py ~/ros2_ws/bags/test_mag_01
```
**What it is:** Reads the bag file and produces path plots as PNG images — GPS path vs EKF path. The goal is both lines overlapping closely.

---

## Quick Reference — Pico States

| `lsusb` output | State | Action needed |
|---|---|---|
| `2e8a:000f` | Boot mode | Ready to flash `.uf2` |
| `2e8a:0009` | Running | Firmware active, serial live |
| Nothing | Not connected | Check USB cable |

## Quick Reference — Common Failures

| Symptom | Likely cause | Fix |
|---|---|---|
| `/dev/ttyACM0` not found | Pico not in running mode | Check `lsusb` |
| `MAG:` lines missing | Mag not wired or wrong address | Check wiring, check boot output |
| `/imu/data` orientation all zeros | mag_node not publishing | Check mag_node logs |
| EKF not publishing | Topic rate 0 on `/odometry/gps` | GPS needs outdoor fix |
| cmake fails | Stale build dir | Delete `pico_firmware/build/` and rerun |
