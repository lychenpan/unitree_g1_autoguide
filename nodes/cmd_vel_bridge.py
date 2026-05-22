#!/usr/bin/env python3
"""
Unified ROS2 cmd_vel Bridge for G1 Robot (Single Process, DDS Conflict Resolved)

Usage:
    python3 cmd_vel_bridge_unified.py
"""

import sys
import os
import time
import signal

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

print("sys.path: ", sys.path)

ChannelFactoryInitialize(0, "eth0")

# Create and initialize LocoClient
loco_client = LocoClient()
loco_client.SetTimeout(10.0)
loco_client.Init()


os.environ['ROS_DOMAIN_ID'] = '1'
import rclpy
rclpy.init()
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Define the class at module level - it inherits from Node
class CmdVelBridge(Node):
    """
    Unified ROS2 cmd_vel Bridge Node
    
    Subscribes to /cmd_vel (ROS2 domain 1) and sends commands to G1 robot
    via Unitree SDK (domain 0).
    """
    
    def __init__(self, loco_client):
        super().__init__('cmd_vel_bridge')
        self.loco_client = loco_client
        
        # Subscribe to cmd_vel on ROS2 domain 1
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('✅ Subscribed to /cmd_vel (ROS2 domain 1)')
        self.get_logger().info('✅ Connected to G1 robot (Unitree SDK domain 0)')
        
        # Control state
        self.last_cmd_time = time.time()
        self.cmd_timeout = 0.5  # seconds
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self.last_movement_log = time.time()
        
        # Timer to check for command timeout
        self.create_timer(0.1, self.timeout_check_callback)
    
    def cmd_vel_callback(self, msg):
        """
        Forward cmd_vel to robot via Unitree SDK.
        
        ⚠️  IMPORTANT: Robot hardware expects inverted coordinates!
        Even though TF is fixed, the physical robot frame is inverted
        due to upside-down lidar mounting (180° X-rotation)
        """
        # Update command values (with coordinate inversion)
        self.current_vx = msg.linear.x        # Forward: unchanged
        self.current_vy = msg.linear.y        # Lateral: inverted (usually 0 for differential drive)
        self.current_vyaw = msg.angular.z    # Rotation: MUST invert! (left ↔ right)
        self.last_cmd_time = time.time()
        print(f"cmd_vel_callback: {msg.linear.x}, {msg.linear.y}, {msg.angular.z}")
        
        # Send command to robot via Unitree SDK (domain 0)
        try:
            self.loco_client.Move(self.current_vx, self.current_vy, self.current_vyaw)
            
            # Log movement (throttled to once per second)
            if abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01 or abs(self.current_vyaw) > 0.01:
                if time.time() - self.last_movement_log > 1.0:
                    # self.get_logger().info(
                    #     f"🚶 Moving: vx={self.current_vx:.2f}, "
                    #     f"vy={self.current_vy:.2f}, vyaw={self.current_vyaw:.2f}"
                    # )
                    self.last_movement_log = time.time()
        except Exception as e:
            self.get_logger().error(f"Failed to send command to robot: {e}")
    
    def timeout_check_callback(self):
        """Check for command timeout and stop robot if no new commands."""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.current_vx != 0.0 or self.current_vy != 0.0 or self.current_vyaw != 0.0:
                self.get_logger().warn("⚠️  Command timeout, stopping robot")
                self.current_vx = 0.0
                self.current_vy = 0.0
                self.current_vyaw = 0.0
                try:
                    self.loco_client.Move(0.0, 0.0, 0.0)
                except Exception as e:
                    self.get_logger().error(f"Failed to stop robot: {e}")


def main():
    # Fixed network interface
    print("\n" + "="*70)
    print("🤖 G1 Robot cmd_vel Bridge (Unified - Single Process)")
    print("="*70)
    print()
    print("⚠️  This will enable Nav2 to control the robot!")
    print()
    print("="*70)
    print()
    
    
    # ========================================================================
    # STEP 4: Create bridge node
    # ========================================================================
    print()
    print("[STEP 4] Creating cmd_vel bridge node...")
    node = CmdVelBridge(loco_client)
    print("[STEP 4] ✓ Bridge node created")
    
    print()
    print("="*70)
    print("✅ Bridge is active! Nav2 can now control the robot.")
    print("="*70)
    print()
    print("   ROS2 Domain 1: Subscribing to /cmd_vel")
    print("   Unitree SDK Domain 0: Controlling robot")
    print()
    print("   Press Ctrl+C to stop")
    print()
    
    # ========================================================================
    # STEP 6: Run ROS2 spin
    # ========================================================================
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user!")
    finally:
        print("\n🛑 Shutting down...")
        
        # Stop robot
        try:
            print("   Stopping robot...")
            loco_client.Move(0.0, 0.0, 0.0)
            time.sleep(0.2)
        except Exception as e:
            print(f"   Warning: Could not stop robot: {e}")
        
        # Cleanup ROS2
        node.destroy_node()
        rclpy.shutdown()
        
        print("✅ Bridge stopped")


if __name__ == "__main__":
    sys.exit(main() or 0)

