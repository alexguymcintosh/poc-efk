# direction
date: 2026-05-19T00:00:00Z

## task
Debug why `mag_node` is not running in the ROS2 pipeline, fix it, and complete the EKF bringup test in `COMMANDS.md`. The mag_node is in `setup.py` and the launch file, and `mag_node.py` exists in the package, but `ros2 run pico_bridge mag_node` returns "No executable found" even after `colcon build --packages-select pico_bridge`. The result is that `/pico/mag_raw` and `/imu/data` are not published, so the EKF runs without magnetometer heading.

## context — what happened this session
Session was driven by alex master (running in docker container) directing alex on the host. Alex now moving claude out of docker and into the ekf project directly — that's you. The pipeline was launched and most of it works; the mag chain is broken at the build step.

### what we verified is working
- Pico 2 firmware running (USB ID `2e8a:0009`), `/dev/ttyACM0` up on host
- Raw serial streams `IMU:`, `MAG:`, `GPS:` lines correctly (`head -20 < /dev/ttyACM0`)
- ROS2 nodes that ARE running: `/serial_parser`, `/imu_node`, `/gps_node`, `/gps_to_odom`, `/ekf_filter_node`, `/tf_base_to_imu`, `/tf_base_to_gps`
- Topics publishing: `/pico/imu_raw`, `/imu/data_raw`, `/fix` (GPS lock at Melbourne), `/odometry/gps`, `/odometry/filtered` (EKF running, position + orientation output, but orientation is unconstrained yaw)

### what is broken
- `/mag_node` does NOT appear in `ros2 node list` even though it's in `pico_bridge.launch.py`
- `ros2 run pico_bridge mag_node` → `No executable found`
- After `colcon build --packages-select pico_bridge && source install/setup.bash` → still `No executable found`
- Consequence: `/pico/mag_raw` never publishes → `/imu/data` (from `mag_node`) never publishes → EKF has no heading reference

### what we did NOT try yet
- `ros2 pkg executables pico_bridge` — would list what's actually registered
- Inspect `~/ros2_ws/install/pico_bridge/lib/pico_bridge/` to see whether the `mag_node` script was written there
- Clean rebuild: `rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log` then `colcon build`
- Check `mag_node.py` for import-time errors (it'd silently fail to register if it crashes during entry-point discovery)

### relevant files
- `ros2_ws/setup.py` — entry point `mag_node = pico_bridge.mag_node:main` IS there
- `ros2_ws/pico_bridge/mag_node.py` — subscribes `/pico/mag_raw` (String), publishes `/imu/data` (Imu) with yaw quaternion from atan2(mx, my) + declination
- `ros2_ws/pico_bridge/serial_parser_node.py` — publishes `/pico/mag_raw` when serial line starts with `MAG:` (this part is fine, just no consumer downstream right now)
- `ros2_ws/launch/pico_bridge.launch.py` — mag_node IS in launch description and event handler
- `COMMANDS.md` — has the test runbook; updates this session: split USB check into two steps (1 + 2), added flash step 1b, added step 4 (`ros2 node list` to check for stale nodes), added bottom "Debug: run mag_node manually" block

## constraints
- One command at a time. Each command has one purpose. Do not chain commands the user has to copy. (Alex was firm about this — see memory.)
- Do not assume host vs container — claude now runs on the host alongside ROS2. Serial port and `~/ros2_ws` are directly accessible.
- Do not invent fixes; diagnose root cause first.
- Use `COMMANDS.md` as the canonical runbook — update it when steps change.

## next steps to fix mag_node (in order)
1. Run `ros2 pkg executables pico_bridge` — see if mag_node is registered at all
2. Run `ls ~/ros2_ws/install/pico_bridge/lib/pico_bridge/` — see if a `mag_node` script was installed
3. Run `python3 -c "from pico_bridge import mag_node"` from `~/ros2_ws/install/pico_bridge/lib/python3.*/site-packages/` — catches import errors that would silently break entry-point install
4. If still broken: clean rebuild `rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log && cd ~/ros2_ws && colcon build --packages-select pico_bridge && source install/setup.bash`
5. Re-verify with `ros2 run pico_bridge mag_node` — should print `mag_node ready. declination=0.0°`

## after mag_node fix — resume COMMANDS.md
- Relaunch pipeline (kill old launch first, step 4b in COMMANDS.md)
- Confirm `/mag_node` appears in `ros2 node list`
- Confirm `/pico/mag_raw` and `/imu/data` publish
- Continue with steps 5–10: topic rates, foxglove, bag record, analyse

## architectural direction (background, not blocking)
Alex is moving claude out of docker. Docker breaks USB passthrough (this is why the master claude couldn't help directly — it lived in a container with no `/dev/ttyACM0`). Going forward:
- ekf project has its own git repo (`https://github.com/alexguymcintosh/poc-efk.git`)
- Lives on host laptop because GPS needs to move with the rover
- Python deps can live in a venv inside this dir if needed
- Per-machine memory at `~/.claude/projects/.../memory/` — that's fine, git handles code sync
- Sync protocol: see master CLAUDE.md at `/workspace/alex/CLAUDE.md` (or `~/system/alex/CLAUDE.md` on laptop)

## return
Update `STATUS.md` in this directory with one of:
- `state: in-progress` while debugging
- `state: blocked` with a one-line blocker if you can't proceed without alex
- `state: done` when mag chain is publishing and EKF has heading
