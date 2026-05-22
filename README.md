# g1_api_ws

Python-only bringup for Unitree G1 SLAM + Nav2 on ROS 2 Foxy. Nav2 binaries come from `/opt/ros/foxy`; this repo provides bridges, launch files, maps, and helper scripts. No `colcon build` is required.

## Workflow

### env start: use robotenv1 to activate env.

1. **Start Unitree SLAM relocation**

   ```bash
   python3 tools/g1_slam_relocation.py
   ```

   Edit the map path and initial pose in that script before running.

2. **Start the odom bridge**

   ```bash
   ./run_bridge.sh
   ```

   Or directly:

   ```bash
   python3 nodes/unitree_relocation_odom_bridge.py
   ```

   Subscribes to `rt/unitree/slam_relocation/odom` (DDS on `eth0`) and publishes `/unitree/odom` plus TF.

3. **Start Nav2**

   ```bash
   ./run_nav2.sh
   ```

   Launches map server, static `map → odom` TF, the odom bridge, and the Nav2 navigation stack using `g1_api_nav2/params.yaml` and `g1_api_nav2/map.yaml`.

4. **Send a navigation goal**

   ```bash
   ./send_goal.sh <x> <y> <yaw_degrees>
   ```

   Example:

   ```bash
   ./send_goal.sh 3 3 0
   ```

   For the robot to move, also run `nodes/cmd_vel_bridge.py` (or your usual cmd_vel bridge) in a separate terminal.

## Layout

| Path | Purpose |
|------|---------|
| `g1_api_nav2/` | Nav2 params, map, launch file |
| `nodes/` | Odom bridge, cmd_vel bridge, camera nodes |
| `tools/` | SLAM relocation, map conversion, TTS, etc. |
| `handshake/` | Handshake / inspire hand experiments |

## Planned work

1. Voice chat and microphone integration
2. Handshake testing and optimization
