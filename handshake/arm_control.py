"""
Handshake arm control: G1_29_ArmController for motion_mode=True only.
Extracted from xr_teleoperate/teleop/robot_control/robot_arm.py.
"""
import numpy as np
import threading
import time
from enum import IntEnum

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"
G1_29_Num_Motors = 35


class MotorState:
    def __init__(self):
        self.q = None
        self.dq = None


class G1_29_LowState:
    def __init__(self):
        self.motor_state = [MotorState() for _ in range(G1_29_Num_Motors)]


class DataBuffer:
    def __init__(self):
        self.data = None
        self.lock = threading.Lock()

    def GetData(self):
        with self.lock:
            return self.data

    def SetData(self, data):
        with self.lock:
            self.data = data


class G1_29_JointArmIndex(IntEnum):
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


class G1_29_JointIndex(IntEnum):
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11
    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


class G1_29_ArmController:
    """G1 dual-arm controller, motion_mode=True (rt/arm_sdk) only."""

    def __init__(self, simulation_mode=False):
        print("Initialize G1_29_ArmController (handshake, motion_mode=True)...")
        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self.simulation_mode = simulation_mode
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5
        self.arm_velocity_limit = 20.0
        self.control_dt = 1.0 / 250.0

        domain_id = 1 if simulation_mode else 0
        ChannelFactoryInitialize(domain_id, "eth0")

        self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        while not self.lowstate_buffer.GetData():
            time.sleep(0.1)
            print("[G1_29_ArmController] Waiting to subscribe dds...")
        print("[G1_29_ArmController] Subscribe dds ok.")

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.lowstate_subscriber.Read().mode_machine

        self.all_motor_q = self.get_current_motor_q()
        arm_indices = set(m.value for m in G1_29_JointArmIndex)

        for id in G1_29_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                if self._is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                if self._is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q = self.all_motor_q[id.value]

        self.ctrl_lock = threading.Lock()
        self._arm_active = False  # True only after ctrl_dual_arm; stop_arm sets False
        self._arm_weight = 0.0   # kNotUsedJoint0.q: 1=our control, 0=sport mode (ramp in move_to_home)
        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()

        print("[G1_29_ArmController] Ready (motion_mode=True).")

    def _subscribe_motor_state(self):
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                for i in range(G1_29_Num_Motors):
                    lowstate.motor_state[i].q = msg.motor_state[i].q
                    lowstate.motor_state[i].dq = msg.motor_state[i].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def _is_weak_motor(self, motor_index):
        weak = [4, 10, 15, 16, 17, 18, 22, 23, 24, 25]
        return motor_index.value in weak

    def _is_wrist_motor(self, motor_index):
        wrist = [19, 20, 21, 26, 27, 28]
        return motor_index.value in wrist

    def clip_arm_q_target(self, target_q, velocity_limit):
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        return current_q + delta / max(motion_scale, 1.0)

    def _ctrl_motor_state(self):
        while True:
            start_time = time.time()
            with self.ctrl_lock:
                arm_active = self._arm_active
                arm_weight = self._arm_weight
                arm_q_target = self.q_target.copy()
                arm_tauff_target = self.tauff_target.copy()

            # weight=1: our control; weight=0: sport mode controls arm
            weight = 1.0 if arm_active else arm_weight
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight

            if arm_active:
                if self.simulation_mode:
                    clipped = arm_q_target
                else:
                    clipped = self.clip_arm_q_target(arm_q_target, self.arm_velocity_limit)
                for idx, mid in enumerate(G1_29_JointArmIndex):
                    self.msg.motor_cmd[mid].q = clipped[idx]
                    self.msg.motor_cmd[mid].dq = 0
                    self.msg.motor_cmd[mid].tau = arm_tauff_target[idx]

            # when not active: do not update arm joints; last values remain (ignored by robot when weight=0)
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            elapsed = time.time() - start_time
            time.sleep(max(0, self.control_dt - elapsed))

    def ctrl_dual_arm(self, q_target, tauff_target):
        """Take over arm control: set weight=1, start applying q_target."""
        with self.ctrl_lock:
            self._arm_active = True
            self._arm_weight = 1.0
            self.q_target = np.array(q_target, dtype=np.float64)
            self.tauff_target = np.array(tauff_target, dtype=np.float64)
            

    def stop_arm(self, ramp=True):
        """Release arm to sport mode: weight=0, stop updating arm joints. Call ctrl_dual_arm to resume.
        If ramp=True (default), smoothly ramp weight 1→0 over ~2 s; if False, instant release."""
        with self.ctrl_lock:
            self._arm_active = False
            self._arm_weight = 1.0
        if ramp:
            for weight in np.linspace(1, 0, num=101):
                with self.ctrl_lock:
                    self._arm_weight = float(weight)
                time.sleep(0.02)
        with self.ctrl_lock:
            self._arm_weight = 0.0

    def get_current_motor_q(self):
        data = self.lowstate_buffer.GetData()
        return np.array([data.motor_state[i].q for i in range(G1_29_Num_Motors)])

    def get_current_dual_arm_q(self):
        data = self.lowstate_buffer.GetData()
        return np.array([data.motor_state[mid].q for mid in G1_29_JointArmIndex])

    def get_current_dual_arm_dq(self):
        data = self.lowstate_buffer.GetData()
        return np.array([data.motor_state[mid].dq for mid in G1_29_JointArmIndex])
