"""
Handshake arm operations: move_left_arm_to_position, move_left_arm_to_home.
"""
import numpy as np
import pinocchio as pin
import time
from .grasp_on_palm_trigger import grasp_on_palm_trigger


def compute_right_arm_pose_from_joints(arm_ik, right_arm_q):
    """Compute 4x4 pose of right wrist in waist frame from right arm joint angles."""
    q_full = np.concatenate([np.zeros(7), right_arm_q])
    model = arm_ik.reduced_robot.model
    data = pin.Data(model)
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    return data.oMf[arm_ik.R_hand_id].homogeneous.copy()


def build_left_arm_target_pose(x, y, z):
    """Build 4x4 SE(3) pose for left arm target. Robot: x front, y left, z up."""
    pose = np.eye(4)
    pose[0, 3], pose[1, 3], pose[2, 3] = x, y, z
    return pose


def apply_velocity_limits(target_q, current_q, max_velocity=0.5, control_dt=0.01):
    """Limit step size to avoid too fast movement."""
    delta_q = target_q - current_q
    max_delta = max_velocity * control_dt
    if np.any(np.abs(delta_q) > max_delta):
        scale_factor = max_delta / np.max(np.abs(delta_q))
        delta_q = delta_q * scale_factor
    return current_q + delta_q


def move_left_arm_to_position(arm_ctrl, arm_ik, right_arm_q, x, y, z, duration=3.0, control_dt=0.01):
    """Move left arm to (x, y, z) in waist frame. Right arm stays fixed."""
    from .arm_control import G1_29_JointIndex

    right_arm_pose = compute_right_arm_pose_from_joints(arm_ik, right_arm_q)
    left_target_pose = build_left_arm_target_pose(x, y, z)

    start_q = arm_ctrl.get_current_dual_arm_q()
    current_dq = arm_ctrl.get_current_dual_arm_dq()

    try:
        sol_q, sol_tauff = arm_ik.solve_ik(
            left_target_pose, right_arm_pose, start_q, current_dq
        )
    except Exception as e:
        raise RuntimeError(f"IK solve failed: {e}") from e

    target_q = np.concatenate([sol_q[:7], right_arm_q])
    num_steps = max(1, int(duration / control_dt))
    q = start_q.copy()

    for step in range(num_steps + 1):
        alpha = step / num_steps
        interpolated_q = start_q + alpha * (target_q - start_q)
        interpolated_q[7:14] = right_arm_q
        interpolated_q = apply_velocity_limits(interpolated_q, q, max_velocity=0.5, control_dt=control_dt)
        interpolated_tauff = np.concatenate([
            sol_tauff[:7] * alpha + (1 - alpha) * np.zeros(7),
            np.zeros(7),
        ])
        arm_ctrl.ctrl_dual_arm(interpolated_q, interpolated_tauff)
        q = interpolated_q
        time.sleep(control_dt)

    ## arm action 
    # time.sleep(10.0)
    print("start to grasp on palm trigger--------")
    grasp_on_palm_trigger(ip="192.168.123.210", debug=True)
    print("stop the arm after the palm trigger-------")
    arm_ctrl.stop_arm()


def move_left_arm_to_home(arm_ctrl):
    """
    Move arm to home by ramping arm weight from 1 to 0 over ~2 s.
    Releases arm control to robot's default/standing pose.
    Uses arm_ctrl._arm_weight so _ctrl_motor_state respects the ramp.
    """
    with arm_ctrl.ctrl_lock:
        arm_ctrl._arm_active = False
        arm_ctrl._arm_weight = 1.0
    for weight in np.linspace(1, 0, num=101):
        with arm_ctrl.ctrl_lock:
            arm_ctrl._arm_weight = float(weight)
        time.sleep(0.02)
