import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select

class TalkerNode(Node):
    def __init__(self):
        super().__init__('motor_talker')
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.get_logger().info("토커 노드가 시작되었습니다. 'w': 전진, 'b': 정지, 's': 후진, 'a': 왼쪽 회전, 'd': 오른쪽 회전")

    def run(self):
        try:
            while rclpy.ok():
                command = self.get_key_input()
                if command == 'w':
                    self.publish_command('start')
                    self.get_logger().info('Published: start')
                elif command == 'b':
                    self.publish_command('stop')
                    self.get_logger().info('Published: stop')
                elif command == 's':
                    self.publish_command('reverse')
                    self.get_logger().info('Published: reverse')
                elif command == 'a':
                    self.publish_command('left')
                    self.get_logger().info('Published: left')
                elif command == 'd':
                    self.publish_command('right')
                    self.get_logger().info('Published: right')
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("키보드 인터럽트로 종료되었습니다.")

    def get_key_input(self):
        """Non-blocking 키 입력 감지 함수."""
        input_ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        if input_ready:
            return sys.stdin.read(1).strip()
        return None

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
