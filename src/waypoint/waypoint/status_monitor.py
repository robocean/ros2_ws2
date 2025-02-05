import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StatusMonitorNode(Node):
    def __init__(self):
        super().__init__('status_monitor_node')

        # 상태 구독자
        self.subscription = self.create_subscription(String, 'robot_state', self.state_callback, 10)
        self.get_logger().info("Status Monitor Node가 시작되었습니다.")

    def state_callback(self, msg):
        """로봇 상태 출력"""
        state = json.loads(msg.data)
        current_position = state["current_position"]
        goal_position = state["goal_position"]
        delta_y = state["delta_y"]

        self.get_logger().info(
            f"현재 위치: {current_position}, 목표 위치: {goal_position}, delta_y: {delta_y:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

