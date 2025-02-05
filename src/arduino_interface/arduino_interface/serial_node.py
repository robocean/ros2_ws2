import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        # Publisher 설정
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        # Subscriber 설정
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.subscription  # 방어적 할당
        
        # 시리얼 포트 초기화
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
        # 타이머 설정 (1초 간격)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """시리얼 데이터를 읽어와 퍼블리시"""
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()  # 데이터 읽기
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)  # 퍼블리시
            self.get_logger().info(f'Publishing: "{msg.data}"')

    def listener_callback(self, msg):
        """수신된 데이터를 처리"""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

