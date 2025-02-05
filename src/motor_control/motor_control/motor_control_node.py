import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import termios
import sys
import tty
import time
from threading import Thread, Event

# 제어 테이블 주소
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104

# 프로토콜 버전
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600

# 포트 이름
DEVICENAME_1 = '/dev/left_wheel'   # 왼쪽 바퀴 포트
DEVICENAME_2 = '/dev/right_wheel'  # 오른쪽 바퀴 포트

# 모터 ID
DXL_ID_1 = 1
DXL_ID_2 = 2

# 속도 설정
LEFT_WHEEL_FORWARD = 1023
RIGHT_WHEEL_FORWARD = -1023
LEFT_WHEEL_REVERSE = -1023
RIGHT_WHEEL_REVERSE = 1023

# 회전 속도 설정
LEFT_WHEEL_RIGHT_TURN = int(1023 * 0.7)   # 오른쪽 회전 시 왼쪽 모터 출력: 70%
RIGHT_WHEEL_RIGHT_TURN = int(-1023 * 0.1)  # 오른쪽 회전 시 오른쪽 모터 출력: 10%

LEFT_WHEEL_LEFT_TURN = int(1023 * 0.1)    # 왼쪽 회전 시 왼쪽 모터 출력: 10%
RIGHT_WHEEL_LEFT_TURN = int(-1023 * 0.7)   # 왼쪽 회전 시 오른쪽 모터 출력: 70%

# 제자리 회전 속도 설정
LEFT_WHEEL_SPIN_LEFT = 0                  # 왼쪽 제자리 회전 시 왼쪽 모터 정지
RIGHT_WHEEL_SPIN_LEFT = int(-1023 * 0.5)  # 왼쪽 제자리 회전 시 오른쪽 모터 50% 출력

LEFT_WHEEL_SPIN_RIGHT = int(1023 * 0.5)   # 오른쪽 제자리 회전 시 왼쪽 모터 50% 출력
RIGHT_WHEEL_SPIN_RIGHT = 0                # 오른쪽 제자리 회전 시 오른쪽 모터 정지

# 비율로 설정된 속도 값
LEFT_WHEEL_FORWARD_30RPM = int(1023 * 0.129032)  # 약132
RIGHT_WHEEL_FORWARD_30RPM = int(-1023 * 0.129032)

LEFT_WHEEL_FORWARD_40RPM = int(1023 * 0.172043)  # 약176 
RIGHT_WHEEL_FORWARD_40RPM = int(-1023 * 0.172043)

# 모터 초기화
portHandler_1 = PortHandler(DEVICENAME_1)
portHandler_2 = PortHandler(DEVICENAME_2)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def initialize_motor(port_handler, motor_id, logger):
    """모터 초기화"""
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        logger.error(f"모터 {motor_id} 토크 비활성화 실패: {packetHandler.getTxRxResult(dxl_comm_result)}, {packetHandler.getRxPacketError(dxl_error)}")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port_handler, motor_id, ADDR_OPERATING_MODE, 1)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        logger.error(f"모터 {motor_id} 속도 제어 모드 설정 실패: {packetHandler.getTxRxResult(dxl_comm_result)}, {packetHandler.getRxPacketError(dxl_error)}")

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        logger.error(f"모터 {motor_id} 토크 활성화 실패: {packetHandler.getTxRxResult(dxl_comm_result)}, {packetHandler.getRxPacketError(dxl_error)}")
    else:
        logger.info(f"모터 {motor_id} 초기화 완료")

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.get_logger().info("모터 제어 노드가 시작되었습니다. 키보드 입력을 기다립니다...")

        if not portHandler_1.openPort():
            self.get_logger().error("왼쪽 바퀴 포트를 열지 못했습니다.")
        if not portHandler_2.openPort():
            self.get_logger().error("오른쪽 바퀴 포트를 열지 못했습니다.")

        portHandler_1.setBaudRate(BAUDRATE)
        portHandler_2.setBaudRate(BAUDRATE)

        initialize_motor(portHandler_1, DXL_ID_1, self.get_logger())
        initialize_motor(portHandler_2, DXL_ID_2, self.get_logger())

    def run(self):
        """키보드 입력에 따라 모터를 제어"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':
                    self.start_motors(LEFT_WHEEL_FORWARD, RIGHT_WHEEL_FORWARD)
                elif key == 's':
                    self.start_motors(LEFT_WHEEL_REVERSE, RIGHT_WHEEL_REVERSE)
                elif key == 'j':
                    self.start_motors(LEFT_WHEEL_FORWARD_30RPM, RIGHT_WHEEL_FORWARD_30RPM)
                elif key == 'k':
                    self.start_motors(LEFT_WHEEL_FORWARD_40RPM, RIGHT_WHEEL_FORWARD_40RPM)
                elif key == 'b':
                    self.stop_motors()
                elif key == 'a':
                    self.start_motors(LEFT_WHEEL_LEFT_TURN, RIGHT_WHEEL_LEFT_TURN)
                elif key == 'd':
                    self.start_motors(LEFT_WHEEL_RIGHT_TURN, RIGHT_WHEEL_RIGHT_TURN)
                elif key == 'z':
                    self.start_motors(LEFT_WHEEL_SPIN_LEFT, RIGHT_WHEEL_SPIN_LEFT)
                elif key == 'c':
                    self.start_motors(LEFT_WHEEL_SPIN_RIGHT, RIGHT_WHEEL_SPIN_RIGHT)
                elif key == 'q':
                    self.get_logger().info("종료 키 입력, 프로그램을 종료합니다.")
                    break
        except KeyboardInterrupt:
            self.get_logger().info("키보드 인터럽트로 종료됩니다.")
        finally:
            self.destroy_node()

    def start_motors(self, left_speed, right_speed):
        """모터 속도를 설정"""
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler_1, DXL_ID_1, ADDR_GOAL_VELOCITY, left_speed)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error(f"왼쪽 모터 오류: {packetHandler.getTxRxResult(dxl_comm_result)}, {packetHandler.getRxPacketError(dxl_error)}")

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler_2, DXL_ID_2, ADDR_GOAL_VELOCITY, right_speed)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error(f"오른쪽 모터 오류: {packetHandler.getTxRxResult(dxl_comm_result)}, {packetHandler.getRxPacketError(dxl_error)}")

        self.get_logger().info(f"왼쪽 모터 속도: {left_speed}, 오른쪽 모터 속도: {right_speed}")

    def stop_motors(self):
        """모터 정지"""
        self.start_motors(0, 0)

    def get_key(self):
        """키 입력을 비차단적으로 읽어옵니다"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        portHandler_1.closePort()
        portHandler_2.closePort()
        self.get_logger().info("포트가 닫혔습니다.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

