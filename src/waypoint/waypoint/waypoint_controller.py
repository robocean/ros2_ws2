import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import subprocess
import json
import math
import numpy as np

# Dynamixel 모터 설정
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICENAME_1 = '/dev/left_wheel'
DEVICENAME_2 = '/dev/right_wheel'
DXL_ID_1 = 1
DXL_ID_2 = 2

class WaypointControllerNode(Node):
    def __init__(self):
        super().__init__('waypoint_controller_node')

        # 상태 퍼블리셔
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)

        # Dynamixel 초기화
        self.portHandler_1 = PortHandler(DEVICENAME_1)
        self.portHandler_2 = PortHandler(DEVICENAME_2)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_motor(self.portHandler_1, DXL_ID_1)
        self.init_motor(self.portHandler_2, DXL_ID_2)

        # 좌표 데이터 초기화
        self.coordinates = self.load_coordinates()
        self.origin_lat = self.coordinates[0]['lat']
        self.origin_lng = self.coordinates[0]['lng']
        self.xy_coordinates = self.convert_coordinates(self.coordinates)

        # 로봇 상태 초기화
        self.current_position = {"x": self.xy_coordinates[0][0], "y": self.xy_coordinates[0][1], "yaw": 0.0}
        self.goal_index = 1
        self.goal_radius = 3.0  # 목표 반경 3m
        self.delta_y_threshold = 1.0  # 허용 가능한 delta_y 값

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Waypoint Controller Node가 시작되었습니다.")

    def init_motor(self, port_handler, motor_id):
        """모터 초기화"""
        if not port_handler.openPort():
            self.get_logger().error(f"모터 {motor_id} 포트를 열지 못했습니다.")
        port_handler.setBaudRate(BAUDRATE)
        self.packetHandler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)

    def load_coordinates(self):
        """get_coordinates에서 좌표 데이터를 불러옵니다."""
        coordinates_data = subprocess.check_output(["cat", "get_coordinates"], text=True)
        return json.loads(coordinates_data)["coordinates"]

    def gps_to_xy(self, lat, lng):
        """GPS 좌표를 XY 좌표로 변환"""
        lat_diff = lat - self.origin_lat
        lng_diff = lng - self.origin_lng
        x = lat_diff * 111320  # 위도를 미터로 변환
        y = lng_diff * 40008000 * np.cos(np.radians(self.origin_lat)) / 360  # 경도를 미터로 변환
        return x, y

    def convert_coordinates(self, coordinates):
        """WGS84 좌표를 XY 좌표로 변환"""
        xy_coordinates = []
        for coord in coordinates:
            x, y = self.gps_to_xy(coord["lat"], coord["lng"])
            xy_coordinates.append((x, y))
        return xy_coordinates

    def calculate_delta_y(self, current_position, start_position, end_position):
        """현재 위치와 path(두 좌표 간 직선) 사이의 수직 거리(delta_y) 계산"""
        x0, y0 = current_position["x"], current_position["y"]
        x1, y1 = start_position["x"], start_position["y"]
        x2, y2 = end_position["x"], end_position["y"]

        # 직선과 점 사이 거리 계산
        numerator = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
        denominator = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return numerator / denominator

    def control_loop(self):
        """로봇 제어 루프"""
        if self.goal_index >= len(self.xy_coordinates):
            self.get_logger().info("모든 목표에 도달했습니다. 시뮬레이션을 종료합니다.")
            self.stop_motors()
            self.timer.cancel()
            return

        goal_position = {"x": self.xy_coordinates[self.goal_index][0], "y": self.xy_coordinates[self.goal_index][1]}
        start_position = {"x": self.xy_coordinates[self.goal_index - 1][0], "y": self.xy_coordinates[self.goal_index - 1][1]}

        # delta_y 계산
        delta_y = self.calculate_delta_y(self.current_position, start_position, goal_position)

        # 상태 퍼블리싱
        state_message = json.dumps({
            "current_position": self.current_position,
            "goal_position": goal_position,
            "delta_y": delta_y
        })
        self.state_publisher.publish(String(data=state_message))

        # 경로 벗어남 보정
        if delta_y > self.delta_y_threshold:
            self.get_logger().info(f"경로 벗어남: delta_y={delta_y:.2f}. 경로 복귀 중...")
            self.correct_path(delta_y)
        else:
            self.get_logger().info(f"delta_y={delta_y:.2f}. 직진 중...")
            self.move_forward()

        # 목표 위치에 도달했는지 확인
        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance <= self.goal_radius:
            self.get_logger().info(f"목표 {self.goal_index}에 도달했습니다: {goal_position}")
            self.stop_motors()
            self.goal_index += 1

    def correct_path(self, delta_y):
        """경로로 복귀"""
        if delta_y > 0:  # 오른쪽으로 회전
            self.set_motor_speed(-300, 300)
        else:  # 왼쪽으로 회전
            self.set_motor_speed(300, -300)

    def move_forward(self):
        """로봇 직진"""
        self.set_motor_speed(500, 500)

    def stop_motors(self):
        """모터 정지"""
        self.set_motor_speed(0, 0)

    def set_motor_speed(self, left_speed, right_speed):
        """모터 속도 설정"""
        self.packetHandler.write4ByteTxRx(self.portHandler_1, DXL_ID_1, ADDR_GOAL_VELOCITY, left_speed)
        self.packetHandler.write4ByteTxRx(self.portHandler_2, DXL_ID_2, ADDR_GOAL_VELOCITY, right_speed)

    def destroy_node(self):
        """노드 종료 시 모터 정리"""
        self.stop_motors()
        self.portHandler_1.closePort()
        self.portHandler_2.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

