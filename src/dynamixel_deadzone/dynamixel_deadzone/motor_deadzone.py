import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import time

# 제어 테이블 주소
ADDR_TORQUE_ENABLE = 64       # 토크 활성화 주소
ADDR_OPERATING_MODE = 11      # 동작 모드 주소
ADDR_GOAL_PWM = 100           # 목표 PWM 주소
ADDR_PRESENT_VELOCITY = 128   # 현재 속도 주소

# 프로토콜 버전 및 통신 설정
PROTOCOL_VERSION = 2.0        # Dynamixel 프로토콜 버전
BAUDRATE = 57600              # 통신 속도

# 포트 이름
DEVICENAME_2 = '/dev/right_wheel'  # right Wheel 포트

# 모터 ID
DXL_ID_2 = 2  # right Wheel ID

# 기타 설정 값
TORQUE_ENABLE = 1             # 토크 활성화
TORQUE_DISABLE = 0            # 토크 비활성화
PWM_CONTROL_MODE = 16         # PWM 제어 모드 값


class DynamixelDeadzoneNode(Node):
    def __init__(self):
        super().__init__('dynamixel_deadzone')

        # Left Wheel 포트를 초기화
        self.port_handler_1 = PortHandler(DEVICENAME_2)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # 포트 열기
        if not self.port_handler_1.openPort():
            self.get_logger().error("Failed to open port for Left Wheel")
            rclpy.shutdown()

        # 보드레이트 설정
        if not self.port_handler_1.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate for Left Wheel")
            rclpy.shutdown()

        self.get_logger().info("Port opened and baudrate set successfully for Left Wheel")

        # 모터 초기화
        self.initialize_motor(self.port_handler_1, DXL_ID_2)

        # 데드존 측정 시작
        self.timer = self.create_timer(1.0, self.measure_deadzone)

    def initialize_motor(self, port_handler, dxl_id):
        # Torque Off
        self.packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        # PWM 제어 모드 설정
        result, dxl_error = self.packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set PWM mode for Motor ID {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
        else:
            self.get_logger().info(f"PWM Control Mode set for Motor ID {dxl_id}")

        # Torque On
        result, dxl_error = self.packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque for Motor ID {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
        else:
            self.get_logger().info(f"Torque enabled for Motor ID {dxl_id}")

    def measure_deadzone(self):
        # Left Wheel의 후진 데드존 측정
        self._test_deadzone(self.port_handler_1, DXL_ID_2, "Left Wheel")

        # 완료 후 ROS 종료
        rclpy.shutdown()

    def _test_deadzone(self, port_handler, dxl_id, motor_name):
        pwm_value = 0
        step = -10  # 음수로 설정하여 PWM 값을 감소
        min_pwm = -885  # 후진 방향 최대 음수 PWM 값
        deadzone_detected = False  # Deadzone 탐지 여부

        self.get_logger().info(f"Starting reverse deadzone test for {motor_name}")
        while pwm_value >= min_pwm:
            # PWM 값 설정
            result, dxl_error = self.packet_handler.write2ByteTxRx(port_handler, dxl_id, ADDR_GOAL_PWM, pwm_value)
            if result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to set PWM for {motor_name} (PWM: {pwm_value}): {self.packet_handler.getTxRxResult(result)}")
                break

            time.sleep(2.5)  # PWM 값을 3초마다 감소

            # 현재 속도 읽기
            velocity, result, dxl_error = self.packet_handler.read4ByteTxRx(port_handler, dxl_id, ADDR_PRESENT_VELOCITY)
            if result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to read velocity for {motor_name}: {self.packet_handler.getTxRxResult(result)}")
                break

            # Deadzone 탐지
            if velocity != 0 and not deadzone_detected:
                self.get_logger().info(f"Reverse Deadzone for {motor_name} detected at PWM: {pwm_value}")
                deadzone_detected = True  # Deadzone이 탐지되었음을 기록

            self.get_logger().info(f"Testing reverse {motor_name} PWM: {pwm_value}, Velocity: {velocity}")
            pwm_value += step

        # Torque Off
        result, dxl_error = self.packet_handler.write1ByteTxRx(port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to disable torque for {motor_name}: {self.packet_handler.getTxRxResult(result)}")
        else:
            self.get_logger().info(f"Reverse Deadzone test for {motor_name} complete at min PWM: {min_pwm}")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDeadzoneNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

