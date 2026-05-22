#!/usr/bin/env python3
"""
HTTP server replacement for handshake_left_arm_subscriber.
Uses REST API instead of ROS2 topics. Same arm control logic.

Endpoints:
  POST /handshake/handd     Body: "x;y;z" or {"x":0.1,"y":0.2,"z":0.3}  -> move arm to position
  POST /handshape/handover  Body: (any)                                  -> move arm to home
  GET  /status             -> {"can_accept_handd": true/false, "busy": true/false}
"""

import json
import os
import sys
import threading
import time

import numpy as np

# Ensure parent (teleoperate/) is in path so "handshake" package can be imported
_script_dir = os.path.dirname(os.path.abspath(__file__))
_parent = os.path.dirname(_script_dir)
if _parent and _parent not in sys.path:
    sys.path.insert(0, _parent)

from flask import Flask, request, jsonify
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from handshake import (
    G1_29_ArmController,
    G1_29_ArmIK,
    move_left_arm_to_position,
    move_left_arm_to_home,
)

waist_z = 0.77
x_offset = 0.1
y_offset = 0.0


app = Flask(__name__)
# Shared state
_state = {
    "arm_ctrl": None,
    "arm_ik": None,
    "right_arm_q": None,
    "busy": threading.Lock(),
    "can_accept_handd": True,
    "duration": 2.0,
}


def load_joint_positions_from_json(path):
    """Load 14 arm joint positions (rad) from JSON. Returns np.array of shape (14,)."""
    with open(path) as f:
        data = json.load(f)
    q = np.array(data["joint_positions_rad"], dtype=np.float64)
    if q.shape != (14,):
        raise ValueError(f"joint_positions_rad must have 14 elements, got {len(q)}")
    return q


def parse_xyz(data):
    """Parse 'x;y;z' string or {"x":..,"y":..,"z":..} dict into (x,y,z). Returns None on error."""
    if isinstance(data, dict):
        try:
            return (float(data["x"]), float(data["y"]), float(data["z"]))
        except (KeyError, ValueError, TypeError):
            return None
    try:
        parts = str(data).strip().split(";")
        if len(parts) != 3:
            return None
        return (float(parts[0].strip()), float(parts[1].strip()), float(parts[2].strip()))
    except (ValueError, AttributeError):
        return None


@app.route("/handshake/handd", methods=["POST"])
def handle_handd():
    # if not _state["can_accept_handd"]:
    #     print("[handd] Rejected: waiting for handover")
    #     return jsonify({"ok": False, "reason": "Waiting for handover to complete"}), 503

    if not _state["busy"].acquire(blocking=False):
        print("[handd] Rejected: still moving")
        return jsonify({"ok": False, "reason": "Still moving to previous target"}), 503

    # Accept "x;y;z" or JSON {"x":..., "y":..., "z":...}
    print("[handd] Accepted: request data: ", request.get_data(as_text=True))
    data = request.get_data(as_text=True) or ""
    if request.is_json:
        data = request.get_json(silent=True) or {}

    xyz = parse_xyz(data)
    if xyz is None:
        _state["busy"].release()
        return jsonify({"ok": False, "reason": "Invalid format, expected 'x;y;z' or {\"x\":..,\"y\":..,\"z\":..}"}), 400

    x, y, z = xyz
    x = x - x_offset
    y = y - y_offset
    z = z - waist_z

    _state["can_accept_handd"] = False

    def run():
        try:
            move_left_arm_to_position(
                _state["arm_ctrl"],
                _state["arm_ik"],
                _state["right_arm_q"],
                x, y, z,
                duration=_state["duration"],
            )
            print("Left arm moved to target.")
        except Exception as e:
            print(f"Movement failed: {e}")
        finally:
            _state["busy"].release()

    threading.Thread(target=run, daemon=True).start()
    return jsonify({"ok": True, "message": "Moving to target"})


# @app.route("/handshape/handover", methods=["POST"])
# def handle_handover():
#     if not _state["busy"].acquire(blocking=False):
#         return jsonify({"ok": False, "reason": "Movement in progress"}), 503

#     def run():
#         try:
#             move_left_arm_to_home(_state["arm_ctrl"])
#             _state["can_accept_handd"] = True
#             print("Move to home completed. Ready for next handd.")
#         except Exception as e:
#             print(f"Move to home failed: {e}")
#         finally:
#             _state["busy"].release()

#     threading.Thread(target=run, daemon=True).start()
#     return jsonify({"ok": True, "message": "Moving to home"})


@app.route("/status", methods=["GET"])
def status():
    return jsonify({
        "can_accept_handd": _state["can_accept_handd"],
        "busy": _state["busy"].locked(),
    })


def main():
    arm_joints_json = os.path.join(_script_dir, "arm_joints.json")
    if not os.path.isfile(arm_joints_json):
        print(f"ERROR: arm_joints.json not found: {arm_joints_json}")
        sys.exit(1)

    full_q = load_joint_positions_from_json(arm_joints_json)
    _state["right_arm_q"] = full_q[7:14].copy()

    ChannelFactoryInitialize(0)
    print("Connecting to robot arm controller...")
    _state["arm_ctrl"] = G1_29_ArmController(simulation_mode=False)
    _state["arm_ik"] = G1_29_ArmIK()
    # Hold current position on startup so arm doesn't jerk to zeros
    # current_q = _state["arm_ctrl"].get_current_dual_arm_q()
    # _state["arm_ctrl"].ctrl_dual_arm(current_q, np.zeros(14))
    print("Robot ready. HTTP server listening on :5000")
    print("  POST /handshake/handd   body: x;y;z or {\"x\":..,\"y\":..,\"z\":..}")
    print("  POST /handshape/handover")
    print("  GET  /status")

    # use_reloader=False: reloader spawns a child process; with shared _state this can
    # cause duplicate request handling or confusing behavior. Single process = one move per handd.
    app.run(host="0.0.0.0", port=5000, threaded=False, use_reloader=False)


if __name__ == "__main__":
    main()
