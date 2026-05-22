# Handshake Arm Control

Standalone arm control for handshake_server. Replaces xr_teleoperate imports.

xr_teleoperate for arm control
inspire_hand_ws for hand control

## Contents

- **arm_control.py** – `G1_29_ArmController` (motion_mode=True, rt/arm_sdk)
- **arm_ik.py** – `G1_29_ArmIK` (inverse kinematics for left-arm x,y,z)
- **arm_ops.py** – `move_left_arm_to_position`, `move_left_arm_to_home`
- **weighted_moving_filter.py** – filter for IK smoothness
- **grasp_on_palm_trigger.py** – palm-triggered grasp (Inspire Hand)
- **inspire_sdkpy/** – Inspire Hand SDK (Modbus + DDS); copied from inspire_hand_ws

## Dependencies

| Package | Purpose | Install |
|---------|---------|---------|
| unitree_sdk2py | G1 robot DDS communication | (Unitree SDK) |
| pinocchio | Robot kinematics | `pip install pin` |
| casadi | IK optimization | `pip install casadi` |
| numpy | Numerics | `pip install numpy` |
| pymodbus | Inspire Hand Modbus TCP | `pip install pymodbus` |
| cyclonedds | Inspire Hand DDS | `pip install cyclonedds` |
| flask | HTTP server | `pip install flask` |

All at once: `pip install pin casadi numpy pymodbus cyclonedds flask`

## Assets

- **URDF**: `handshake/assets/g1/g1_body29_hand14.urdf` (copied from xr_teleoperate)
- **arm_joints.json**: right-arm default pose (in handshake/)

## Run

From `teleoperate/`: `python handshake/handshake_server.py`
