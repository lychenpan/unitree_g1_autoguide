import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tts_player import RemoteTTSPlayer


GOALS = [
    # (2.04389, 5.45511, -0.75947),
    # (3.06306, 6.62257, -0.63755),            
    (9.59981, 8.07126, -1.55780),
    (4.38853, 9.82912, -1.26172),
    (6.26905, 12.52388, 1.99526),
    (8.43942, 9.42302, -2.23091),
    (10.18071, 6.08362, 2.30097),
    (10.90188, 7.50113, -2.75488),
    (3.53442, 6.35187, -2.30050),
    (5.65731, 17.61633, -2.41657),  #5.65731, -23.61633, -2.41657
    (-3.55489, 17.98313, -0.36193),
]


def nav_status_to_text(status: int) -> str:
    status_map = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return status_map.get(status, f"UNMAPPED({status})")


def yaw_to_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def split_text_into_parts(text: str, parts: int):
    text = text.strip()
    if parts <= 0:
        return []
    if not text:
        return [""] * parts

    length = len(text)
    chunks = []
    for i in range(parts):
        start = round(i * length / parts)
        end = round((i + 1) * length / parts)
        chunks.append(text[start:end].strip())
    return chunks


class NavClient(Node):
    def __init__(self):
        super().__init__("nav_client")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def navigate_blocking(self, x: float, y: float, yaw: float, timeout_sec: float = 300.0):
        # 等待 action server 可用
        self.get_logger().info("Step[NAV]: waiting for /navigate_to_pose action server")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose server not available")
            return None

        # 构造 goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        # 发送并阻塞等待是否接受
        self.get_logger().info(
            f"Step[NAV]: sending goal x={x:.5f}, y={y:.5f}, yaw={yaw:.5f}, qz={qz:.5f}, qw={qw:.5f}"
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            self.get_logger().error("timeout waiting goal acceptance")
            return None
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return None
        self.get_logger().info("Step[NAV]: goal accepted by action server")

        # 阻塞等待最终结果
        self.get_logger().info("Step[NAV]: waiting for navigation result")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error("timeout waiting goal result")
            return None

        wrapped = result_future.result()
        self.get_logger().info(
            f"Step[NAV]: result received status={wrapped.status}({nav_status_to_text(wrapped.status)})"
        )
        return wrapped.status, wrapped.result


def main():
    rclpy.init()
    node = NavClient()
    tts = RemoteTTSPlayer()
    try:
        node.get_logger().info("=== Robot nav + TTS mission started ===")
        with open("/workspace/1.txt", "r", encoding="utf-8") as f:
            full_text = f.read()
        speech_chunks = split_text_into_parts(full_text, 8)
        node.get_logger().info(
            f"Loaded speech text: total_chars={len(full_text.strip())}, chunks={len(speech_chunks)}"
        )

        for idx, (x, y, yaw) in enumerate(GOALS, start=1):
            node.get_logger().info(f"--- Point {idx}/8 begin ---")
            node.get_logger().info(f"Step[NAV]: start navigate to point {idx}: x={x}, y={y}, yaw={yaw}")
            out = node.navigate_blocking(x, y, yaw, timeout_sec=300.0)
            node.get_logger().info(f"Step[NAV]: point {idx} raw result: {out}")

            if out is None or out[0] != GoalStatus.STATUS_SUCCEEDED:
                status_text = "None" if out is None else f"{out[0]}({nav_status_to_text(out[0])})"
                node.get_logger().warn(
                    f"Step[NAV]: point {idx} not successful, status={status_text}, skip speaking and continue."
                )
                continue

            node.get_logger().info(f"Step[NAV]: point {idx} navigation succeeded")
            # 修改点：从第3个点开始播报（idx >= 3），对应语音段索引为 idx - 3
            if idx >= 0:
                chunk_idx = idx - 0  # 第3个点 -> chunk 0，第4个点 -> chunk 1，...
                speak_text = speech_chunks[chunk_idx] if chunk_idx < len(speech_chunks) else ""
                if speak_text:
                    node.get_logger().info("Step[TTS]: wait 1 second before speaking")
                    time.sleep(1.0)
                    node.get_logger().info(
                        f"Step[TTS]: start speaking point {idx}, chunk {chunk_idx + 1}/8, chars={len(speak_text)}"
                    )
                    print(speak_text)
                    tts.playtext(speak_text)
                    tts.wait_done()
                    node.get_logger().info(f"Step[TTS]: end speaking at point {idx}")
                    node.get_logger().info("Step[FLOW]: wait 2 seconds before next navigation")
                    time.sleep(2.0)
                else:
                    node.get_logger().warn(f"Step[TTS]: point {idx} has empty chunk, skip speaking.")
            else:
                node.get_logger().info(f"Step[TTS]: point {idx} is before point 3, no speaking required")
            node.get_logger().info(f"--- Point {idx}/8 end ---")
        node.get_logger().info("=== Mission complete: all points processed ===")
    finally:
        node.get_logger().info("Step[SHUTDOWN]: stopping TTS and shutting down ROS node")
        tts.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()