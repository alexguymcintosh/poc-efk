# poc-ekf — Autonomous Agricultural Navigation Prototype

**RMIT Capstone P004044Eng — Knuckey Agricultural Engineering**
*Autonomous rover navigation stack — sensor fusion prototype*

---

## What This Is

This repo is a working proof-of-concept sensor fusion stack for autonomous agricultural navigation. It fuses a GPS receiver and IMU into a real-time Extended Kalman Filter (EKF) that produces a clean, fused odometry estimate suitable for feeding into ROS2 Nav2 for autonomous waypoint navigation.

The immediate goal: take a GPS waypoint, use the EKF for localisation, and have the system tell you which compass heading to walk toward. No motor control yet — just the sensing, fusion, and guidance pipeline proven end to end.

The long-term goal: a swarm of autonomous rovers navigating broadacre paddocks using GPS tramlines, obstacle detection, and aerial survey data from a DJI Mavic 3M multispectral drone.

---

## Hardware Stack

| Component | Model | Role |
|---|---|---|
| Microcontroller | Raspberry Pi Pico 2 (RP2350) | Bare metal sensor bridge |
| IMU | GY-521 (MPU-6050) | 6-axis accel + gyro over I2C |
| GPS | GY-GPSV3 (NEO-M9N) | Multi-GNSS, 1Hz NMEA over UART |
| Magnetometer | HMC5883L / QMC5883L | Absolute heading — **next hardware addition** |
| Laptop | Pop!_OS 24.04, ThinkPad | ROS2 Jazzy, EKF, visualisation |

**Why the Pico 2 as a dumb bridge and not a Pi Zero 2W:**
The Pi Zero 2W runs Linux which introduces scheduler jitter, 30 second boot time, and USB gadget mode complexity. The Pico 2 is bare metal — deterministic microsecond timing. All ROS2 logic runs on the laptop. The Pico just reads I2C and UART and forwards raw bytes over USB CDC serial. Simple, fast, reliable.

**Why C++ not MicroPython:**
MicroPython has a Global Interpreter Lock and non-deterministic timing. C++ with the Pico SDK gives deadline-based 100Hz IMU polling that never drifts regardless of how long the I2C transaction takes.

---

## Software Stack

| Layer | Technology |
|---|---|
| Firmware | C++ / Pico SDK (bare metal) |
| Transport | USB CDC serial — appears as /dev/ttyACM0 |
| Middleware | ROS2 Jazzy |
| Sensor fusion | robot_localization (EKF) |
| Visualisation | Foxglove Studio |
| Analysis | Python / rosbag2_py |

---

## Repo Structure

```
poc-ekf/
├── pico_firmware/
│   ├── main.cpp              # Pico firmware — IMU + GPS bridge
│   ├── CMakeLists.txt        # Pico SDK build config
│   └── pico_sdk_import.cmake
├── ros2_ws/                  # ROS2 pico_bridge package
│   ├── pico_bridge/
│   │   ├── serial_parser_node.py   # Opens /dev/ttyACM0, routes by prefix
│   │   ├── imu_node.py             # Publishes /imu/data_raw
│   │   ├── gps_node.py             # Publishes /fix
│   │   └── gps_to_odom_node.py     # Converts /fix → /odometry/gps
│   ├── config/
│   │   └── ekf.yaml          # robot_localization EKF parameters
│   ├── launch/
│   │   └── pico_bridge.launch.py   # Launches full pipeline
│   ├── setup.py
│   └── package.xml
└── analyse_bag.py            # ROS bag analysis and path visualisation
```

---

## System Architecture

```
MPU-6050 (I2C0, GP4/GP5, 100Hz)  ──┐
                                     │
NEO-M9N  (UART0, GP0/GP1, 1Hz)   ──┤
                                     │
         Raspberry Pi Pico 2         │
         - Gyro bias calibration     │
         - Baud auto-detection       │
         - CSV framing               │
         - USB CDC stream           ─┘
                  │
         /dev/ttyACM0 (115200 baud)
                  │
         serial_parser_node
         - Prefix routing
         - IMU: → /pico/imu_raw
         - GPS: → /pico/gps_raw
              │              │
         imu_node        gps_node
              │              │
     /imu/data_raw        /fix
              │              │
              │      gps_to_odom_node
              │              │
              │      /odometry/gps
              │              │
              └──────────────┘
                      │
                 ekf_filter_node
                      │
            /odometry/filtered ← fused pose at 30Hz
```

---

## Key Decisions and Why

### 1. One Pico for both sensors
IMU uses I2C0 (GP4/GP5) and GPS uses UART0 (GP0/GP1). Completely separate pins, no conflict. One USB CDC port, one serial_parser_node, prefix-based routing. Simpler than two Picos and two serial ports.

### 2. GPS baud auto-detection
The NEO-M9N defaults to 57600 baud — not 9600 like older u-blox modules. Rather than hardcode and get garbled output, the firmware sweeps 9600, 38400, 57600, 115200 and checks for the NMEA '$' character. Locked at 57600 on this module.

### 3. Gyro bias calibration on boot
The MPU-6050 Y axis reads ~125 deg/s while completely stationary — a known hardware characteristic of uncalibrated MEMS gyros. On startup, 500 samples are collected and averaged. The resulting bias offset is subtracted from every subsequent reading. Without this the EKF receives wildly incorrect angular velocity data.

### 4. Custom gps_to_odom_node instead of navsat_transform_node
robot_localization includes navsat_transform_node for converting GPS to odometry. However, it requires /odometry/filtered to provide a yaw estimate, and /odometry/filtered requires /odometry/gps from navsat_transform. Without a magnetometer there is no initial yaw to break this circular dependency — complete deadlock.

Solution: a 30-line custom node that converts lat/lon to local x/y metres using equirectangular projection relative to the first valid GPS fix as datum. No external heading required. No deadlock. Works perfectly.

### 5. Linear acceleration NOT fused into EKF (currently)
Fusing ax/ay/az from an IMU without a magnetometer is a critical mistake. Without absolute heading, the EKF double-integrates noisy acceleration in an unknown frame. The walk test showed the EKF reporting 685m travelled when the actual distance was ~40-50m — entirely caused by this. Currently only angular velocity (gyro) is fused. GPS handles all position. The linear acceleration will be re-enabled once the magnetometer provides absolute heading.

---

## What the Data Shows

### Stationary test (60 seconds, sensor not moving):
- EKF drift: ~3m start-to-end — GPS noise floor, expected
- GPS raw drift: ~1.3m in first 10 seconds — ~5m accuracy without DGPS

### Walk test (out and back, ~40-50m actual):
- GPS raw path: correct, tracks the actual walk closely
- EKF path: 685m reported — caused by linear acceleration fusion without heading reference
- After disabling linear acceleration fusion: EKF will track GPS closely

The path plot (combined_paths.png from analyse_bag.py) shows this clearly. GPS and GPS-odom overlay almost perfectly. EKF diverges because of heading drift.

---

## Current Limitations

| Limitation | Impact | Fix |
|---|---|---|
| No magnetometer | No absolute heading — EKF position drifts | **Adding HMC5883L next session** |
| GPS 1Hz | Slow position corrections | Configure NEO-M9N to 10Hz via UBX-CFG-RATE |
| MPU-6050 gyro bias | Startup calibration only — drifts with temperature | Online bias estimation in EKF |
| ~5m GPS accuracy | Position noise floor | RTK GPS for centimetre accuracy |

---

## What's Next

### Immediate — Magnetometer (next session)
Wire HMC5883L to I2C0 bus alongside the MPU-6050 (address 0x1E — no conflict). Add to firmware read sequence. Publish orientation quaternion in imu_node. Re-enable ax/ay/az in EKF imu0_config. Rerun walk test — the 685m triangle should become a tight path matching GPS.

### Short term — System debugging and tuning
- Foxglove dashboard for real-time monitoring of all sensor streams
- EKF covariance tuning — adjust Q and R matrices based on bag analysis
- GPS update rate — configure to 10Hz via UBX protocol
- IMU online bias estimation

### Medium term — Nav2 waypoint guidance
- Input GPS waypoints
- Nav2 uses /odometry/filtered for localisation
- System outputs compass heading to walk toward each waypoint
- Pure pedestrian test — no motors, just heading guidance on screen
- This proves the full autonomous navigation stack end to end

### Integration with team
Three parallel workstreams converging:

**This repo (sensor fusion prototype):**
Status: EKF working, magnetometer next, then Nav2 waypoint guidance

**Teammate 1 (Isaac Sim simulation):**
Status: Isaac Sim 5.1.0 running with Claude Code MCP integration. Building paddock terrain, importing rover URDF, connecting ROS2 Nav2. EKF pipeline starting now.

**Teammate 2 (physical rover):**
Status: Motor control done via TB6612FNG drivers. Next: connect Nav2 to motor control. Needs /odometry/filtered from EKF and /cmd_vel from Nav2 planner.

**Convergence point:** All three teams produce the same ROS2 interface — /odometry/filtered in, /cmd_vel out. The simulation validates the algorithm, the prototype proves the hardware, the full rover integrates both.

---

## Setup Instructions

### Prerequisites
- Pop!_OS 24.04 or Ubuntu 24.04
- ROS2 Jazzy installed
- ARM cross-compiler: `sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git`
- robot_localization: `sudo apt install ros-jazzy-robot-localization`
- Foxglove bridge: `sudo apt install ros-jazzy-foxglove-bridge`

### Pico SDK
```bash
git clone https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
cd ~/pico-sdk && git submodule update --init
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

### Build Pico Firmware
```bash
cd pico_firmware
mkdir build && cd build
cmake -DPICO_BOARD=pico2 -DCMAKE_TOOLCHAIN_FILE=$PICO_SDK_PATH/cmake/preload/toolchains/pico_arm_cortex_m33_gcc.cmake ..
make -j
```

Flash: hold BOOTSEL on Pico, replug USB, release, then:
```bash
cp build/imu.uf2 /media/$USER/RP2350/
```

### Build ROS2 Package
```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_ws ~/ros2_ws/src/pico_bridge
cd ~/ros2_ws
colcon build --packages-select pico_bridge
source install/setup.bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

---

## Every Session Startup

```bash
# Kill any stale nodes
pkill -f serial_parser_node; pkill -f imu_node; pkill -f gps_node
pkill -f ekf_node; pkill -f gps_to_odom_node; pkill -f static_transform_publisher
pkill screen; pkill -f foxglove_bridge

# Verify Pico is running (should show 0009 not 000f)
lsusb | grep -i 2e8a

# Launch pipeline
cd ~/ros2_ws && source install/setup.bash
ros2 launch pico_bridge pico_bridge.launch.py

# Optional — Foxglove visualisation
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# Connect Foxglove Studio to ws://localhost:8765
```

---

## Verify Pipeline Layer by Layer

```bash
# Raw serial data
head -5 < /dev/ttyACM0

# Each topic in order
ros2 topic echo /pico/imu_raw --once
ros2 topic echo /pico/gps_raw --once
ros2 topic echo /imu/data_raw --once
ros2 topic echo /fix --once          # needs outdoor GPS fix
ros2 topic echo /odometry/gps --once
ros2 topic echo /odometry/filtered --once

# Topic rates
ros2 topic hz /imu/data_raw   # expect ~100Hz
ros2 topic hz /fix             # expect ~1Hz
ros2 topic hz /odometry/filtered  # expect ~30Hz
```

---

## Record and Analyse Data

```bash
# Record
mkdir -p ~/ros2_ws/bags
ros2 bag record /fix /imu/data_raw /odometry/filtered /odometry/gps \
  -o ~/ros2_ws/bags/test_name

# Analyse
source /opt/ros/jazzy/setup.bash
python3 analyse_bag.py ~/ros2_ws/bags/test_name
```

Produces: `combined_paths.png`, `ekf_xy_time.png`, `imu_gyro_z_time.png`

---

## Foxglove Dashboard

Connect to `ws://localhost:8765` and add:
- **Map panel** → `/fix` — GPS position on satellite map
- **Plot panel** → `/odometry/filtered.pose.pose.position.x` and `.y` — EKF position
- **Plot panel** → `/imu/data_raw.angular_velocity.z` — yaw rate, spikes on turns
- **Raw Messages** → `/odometry/filtered` — full EKF state with covariance

---

*RMIT Capstone P004044Eng — Knuckey Agricultural Engineering — May 2026*
