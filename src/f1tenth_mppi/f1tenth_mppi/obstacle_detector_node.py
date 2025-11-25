#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.ndimage import binary_dilation

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import copy

class SimpleObstacleDetector(Node):
    def __init__(self):
        super().__init__('simple_obstacle_detector')

        # 파라미터 선언
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', '/scan'),
                ('occupancy_topic', '/local_map'),
                ('vehicle_frame', 'laser'),
                ('grid_resolution', 0.05),
                ('grid_width', 100),
                ('grid_height', 100),
                ('obstacle_dilation', 10)
            ])

        # 파라미터 값 가져오기
        self.scan_topic = self.get_parameter('scan_topic').value
        self.occupancy_topic = self.get_parameter('occupancy_topic').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.obstacle_dilation = self.get_parameter('obstacle_dilation').value
        
        # OccupancyGrid 메시지 초기화
        self.og_msg = OccupancyGrid()
        self.og_msg.header.frame_id = self.vehicle_frame
        self.og_msg.info.resolution = self.grid_resolution
        self.og_msg.info.width = self.grid_width
        self.og_msg.info.height = self.grid_height

        # ===== [수정 1: 그리드 원점(Origin) 설정 변경] =====
        # MPPI 노드가 기대하는 '전방향 맵'의 원점과 동일하게 설정합니다.
        # x=0: 차량의 x축 위치가 그리드의 시작점 (왼쪽 가장자리)
        # y=-(height/2)*res: 차량의 y축 위치가 그리드의 중앙
        self.og_msg.info.origin.position.x = 0.0
        self.og_msg.info.origin.position.y = - (self.grid_height / 2.0) * self.grid_resolution
        # ===================================================
        
        # 구독자(Subscriber) 및 발행자(Publisher) 생성
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback, 
            10)
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid,
            self.occupancy_topic,
            10)
        
        self.scan_data = None 
        self.timer = self.create_timer(1/40, self.grid_callback)

        self.get_logger().info(f"Local Occupancy Grid Publisher node started. Publishing to '{self.occupancy_topic}'.")

    def scan_callback(self, scan_msg: LaserScan):
        """
        LaserScan 메시지를 수신하면 Occupancy Grid를 생성하고 즉시 발행합니다.
        (타이머와 threading을 제거하여 코드를 단순화하고 지연 시간을 줄였습니다.)
        """
        self.scan_data = scan_msg
        

    def grid_callback(self):

        scan_msg = copy.copy(self.scan_data)
        if not isinstance(scan_msg, LaserScan):
            return 
        
        occupancy_grid = self.create_occupancy_grid(scan_msg)
        
        self.og_msg.header.stamp = self.get_clock().now().to_msg()
        self.og_msg.data = occupancy_grid.flatten().tolist()
        self.occupancy_pub.publish(self.og_msg)


    def create_occupancy_grid(self, scan_msg: LaserScan) -> np.ndarray:
        """
        LaserScan 데이터를 처리하여 2D Occupancy Grid numpy 배열을 생성합니다.
        """
        occupancy_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)

        angles = scan_msg.angle_min + scan_msg.angle_increment * np.arange(len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)
        
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        # 극좌표계 -> 직교좌표계 (x: 전방, y: 왼쪽)
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # ===== [수정 2: 그리드 인덱스 계산 방식 변경] =====
        # 새로운 원점 정의에 맞게 인덱스 계산 방식을 MPPI 노드와 동일하게 맞춥니다.
        x_indices = (x_coords / self.grid_resolution).astype(int)
        y_indices = (y_coords / self.grid_resolution + self.grid_height / 2.0).astype(int)
        # ===================================================

        valid_mask = (x_indices >= 0) & (x_indices < self.grid_width) & \
                     (y_indices >= 0) & (y_indices < self.grid_height)
        
        x_indices = x_indices[valid_mask]
        y_indices = y_indices[valid_mask]

        # numpy 배열은 (행, 열) 순서이므로 (y, x)로 인덱싱
        occupancy_grid[y_indices, x_indices] = 100

        if self.obstacle_dilation > 0:
            dilation_kernel = np.ones((self.obstacle_dilation, self.obstacle_dilation), dtype=bool)
            occupancy_grid = binary_dilation(occupancy_grid, structure=dilation_kernel).astype(np.int8) * 100

        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
