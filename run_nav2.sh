#!/usr/bin/env bash
ros2 launch "$(dirname "$0")/g1_api_nav2/unitree_slam_bringup.launch.py" "$@"
