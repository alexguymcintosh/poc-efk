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
| Magnetometer | HMC5883L (or QMC5883L variant) | Absolute heading — wired and fused into EKF |
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
├── README.md                       # this file
├── COMMANDS.md                     # step-by-step test protocol (the runbook for every session)
├── HARDWARE.md                     # wiring and component notes
├── STATUS.md / DIRECTION.md        # current state + master-agent direction (auto-maintained)
├── pico_firmware/
│   ├── main.cpp                    # Pico firmware — IMU + MAG + GPS bridge
│   ├── CMakeLists.txt              # Pico SDK build config
│   └── pico_sdk_import.cmake
├── ros2_ws/                        # ROS2 pico_bridge package (symlinked into ~/ros2_ws/src/)
│   ├── pico_bridge/
│   │   ├── serial_parser_node.py   # Opens /dev/ttyACM0, routes by prefix
│   │   ├── imu_node.py             # Publishes /imu/data_raw (gyro + accel)
│   │   ├── mag_node.py             # Publishes /imu/data (orientation quaternion from mag yaw)
│   │   ├── gps_node.py             # Publishes /fix
│   │   └── gps_to_odom_node.py     # Converts /fix → /odometry/gps
│   ├── config/
│   │   └── ekf.yaml                # robot_localization EKF parameters
│   ├── launch/
│   │   └── pico_bridge.launch.py   # Launches full pipeline
│   ├── setup.py
│   └── package.xml
├── analyse_bag.py                  # ROS bag analysis and path visualisation
└── test_results/                   # Bagged sessions + analysis PNGs, dated subdirs
    └── 2026-05-20_outdoor_walk_01/ # First outdoor walk with mag fusion
```

---

## System Architecture

```
MPU-6050  (I2C0 0x68, GP4/GP5,  100Hz)  ──┐
HMC5883L  (I2C0 0x1E, GP4/GP5,   50Hz)  ──┤
NEO-M9N   (UART0,     GP0/GP1,    1Hz)  ──┤
                                            │
         Raspberry Pi Pico 2                │
         - Gyro bias calibration            │
         - Mag hard-iron bias calibration   │
         - GPS UART baud auto-detection     │
         - CSV framing per sensor           │
         - USB CDC stream                  ─┘
                  │
         /dev/ttyACM0 (115200 baud)
                  │
         serial_parser_node
         - Prefix routing
         - IMU: → /pico/imu_raw
         - MAG: → /pico/mag_raw
         - GPS: → /pico/gps_raw
              │           │           │
         imu_node    mag_node    gps_node
              │           │           │
     /imu/data_raw   /imu/data      /fix
       (100Hz)       (50Hz, mag      │
              │       yaw quat)      │
              │           │   gps_to_odom_node
              │           │           │
              │           │     /odometry/gps
              │           │           │
              └───────────┴───────────┘
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

### 5. Linear acceleration NOT fused into EKF (still)
Even with the mag now providing absolute yaw, linear acceleration is deliberately left disabled in `ekf.yaml` (`imu0_config` last three values all `false`). Reason: until the mag heading is shown to be stable end-to-end, fusing ax/ay/az on top of a noisy heading just integrates that noise into position. The first outdoor walk (see [`test_results/2026-05-20_outdoor_walk_01/`](test_results/2026-05-20_outdoor_walk_01/README.md)) confirmed the mag heading is **not** stable enough yet — the EKF sawtooths between GPS updates and over-counts distance 2.4×. Re-enable accel fusion only after mag calibration is improved.

### 6. Magnetometer wired on the same I2C bus as the IMU
HMC5883L lives at I2C0 0x1E; MPU-6050 at I2C0 0x68. No address conflict, no extra bus. The Pico probes both chips at boot and falls back gracefully if either is missing. Boot-time hard-iron calibration takes 100 stationary samples and subtracts the mean — adequate for a first walk but **insufficient long-term** (no soft-iron correction, no orientation coverage). The first outdoor test made this limitation visible: post-calibration mag values jump frame-to-frame by similar magnitude to the signal itself.

---

## What the Data Shows

### First outdoor walk — 2026-05-20 ([full results here](test_results/2026-05-20_outdoor_walk_01/README.md))

70.8 second out-and-back walk, ~74 m actual distance, hand-held rig.

| Metric | EKF | GPS-raw | Comment |
|---|---|---|---|
| Total distance travelled | 175.12 m | 73.93 m | EKF over-counts 2.4× |
| Start → end displacement | 5.66 m | 5.44 m | Within 0.2 m — endpoint excellent |

**Working:** macro trajectory matches GPS, endpoint within 0.2 m, full pipeline alive 70+ s with no dropouts, all topic rates correct (IMU 100 Hz, mag 50 Hz, EKF 30 Hz, GPS 1 Hz).

**Broken:** EKF path is a sawtooth ~2 m peak-to-peak overlaid on the true path — visible in `combined_paths.png`. Root cause: no velocity source, plus noisy mag heading rotating the velocity vector between 1 Hz GPS corrections.

### Stationary smoke-test (31 s)
All five topics captured at expected counts. Mandatory pre-walk check after `test_mag_02` recorded `/odometry/filtered` only — the rest had silently died because the Pico's GPS UART had stopped emitting. See [`COMMANDS.md`](COMMANDS.md) steps 26–27 for the protocol.

---

## Current Limitations

| Limitation | Impact | Status / Fix |
|---|---|---|
| Mag hard-iron-only calibration | Heading noisy (~±a few degrees), EKF sawtooths between GPS samples | Need figure-8 / sphere-fit calibration |
| No velocity source in EKF | Position propagation between GPS updates is unconstrained | Either re-enable accel fusion (after mag is stable) or add wheel odometry once we have a chassis |
| GPS 1 Hz | Slow position corrections | Configure NEO-M9N to 10 Hz via UBX-CFG-RATE |
| MPU-6050 gyro bias | Startup calibration only — drifts with temperature | Online bias estimation in EKF |
| ~5 m GPS accuracy | Position noise floor | RTK GPS for centimetre accuracy (deferred — Pixhawk 6C path may use this) |

---

## What's Next

### Immediate — Pixhawk 6C as accuracy baseline
The first outdoor walk showed the raw stack (MPU-6050 + HMC5883L + NEO-M9N + robot_localization) gets the macro trajectory right but adds ~100 m of fake path through heading-noise integration. Rather than tune-and-retest the raw stack blind, bring up a Pixhawk 6C in parallel:

- 6C has aerospace-grade IMUs (ICM-42688-P + BMI088, redundant), a better magnetometer (IST8310 with proper soft-iron calibration in PX4/ArduPilot), and onboard EKF2/EKF3 doing the fusion correctly.
- Surface its fused odometry to ROS2 via `mavros` (or `micro-XRCE-DDS` for PX4 v1.14+).
- Use the 6C output as the reference to validate raw-hardware tuning against — same walk, both rigs co-mounted, compare paths.

### After 6C baseline — tune the raw stack
1. Figure-8 magnetometer calibration in firmware (sphere fit captures hard + soft iron). Biggest single lever.
2. Bump mag orientation covariance in `mag_node.py` (currently `0.1`, try `~1.0`) so the EKF trusts the mag less.
3. Re-enable linear acceleration fusion in `ekf.yaml` once mag heading is stable.
4. GPS update rate to 10 Hz via UBX-CFG-RATE.

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
The repo's `ros2_ws/pico_bridge` directory is the actual package — symlink it into a colcon workspace and build with `--symlink-install` so Python edits in this repo are picked up without rebuilding:
```bash
mkdir -p ~/ros2_ws/src
ln -s ~/system/alex/agent/rob/rover/p1/loc/ekf/ros2_ws ~/ros2_ws/src/pico_bridge
cd ~/ros2_ws
colcon build --symlink-install --packages-select pico_bridge
source install/setup.bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```
A rebuild is only required when `setup.py`, `package.xml`, `launch/*.py`, or `config/*.yaml` changes — not for `*.py` edits inside `pico_bridge/`.

---

## Running a Test Session

**Always follow [`COMMANDS.md`](COMMANDS.md)** — it's the canonical step-by-step runbook. Every command is single-purpose; the file is updated each session as the protocol evolves.

Summary of phases:
1. **Steps 1–4** — pre-flight: confirm Pico is running, /dev/ttyACM0 exists, raw serial flowing, no stale nodes
2. **Step 5** — launch the full pipeline in Terminal 1 and leave it open
3. **Steps 6–19** — sanity-check each topic, rates, and Foxglove visualisation
4. **Steps 20–25** — pre-bag health checks (paste outputs to claude for green-light)
5. **Steps 26–27** — mandatory stationary smoke-test bag, verify all topics captured
6. **Steps 28–30** — walk recording + analysis

See [`test_results/`](test_results/) for archived bagged sessions and the analysis PNGs they produced.

---

## Foxglove Dashboard

Connect to `ws://localhost:8765` and add:
- **Map panel** → `/fix` — GPS position on satellite map
- **Plot panel** → `/odometry/filtered.pose.pose.position.x` and `.y` — EKF position
- **Plot panel** → `/imu/data_raw.angular_velocity.z` — yaw rate, spikes on turns
- **Raw Messages** → `/odometry/filtered` — full EKF state with covariance

---

*RMIT Capstone P004044Eng — Knuckey Agricultural Engineering — May 2026*
