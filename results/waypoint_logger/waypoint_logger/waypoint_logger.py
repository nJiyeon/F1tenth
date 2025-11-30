#!/usr/bin/env python3
import os 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math
from tf_transformations import euler_from_quaternion
import numpy as np
import time 

class OdometryToCSV(Node):
    def __init__(self):
        super().__init__('odometry_to_csv')
        
        # CSV 파일 설정
        csv_filename = f'wp_{time.asctime(time.localtime()).replace(" ", "_").replace(":", "-")}.csv'
        
        mapdir = os.path.join(os.path.expanduser('~'), "waypoints")
        if not os.path.exists(mapdir):
            os.mkdir(mapdir)
        
        self.csv_path = os.path.join(mapdir, csv_filename)

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file, delimiter=';')
        
        # CSV 헤더 작성
        self.csv_writer.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
        
        # 이전 데이터 저장용 변수들
        self.prev_x = None
        self.prev_y = None
        self.prev_psi = None
        self.prev_vx = None
        self.prev_time = None
        self.total_distance = 0.0
        
        # Odometry 구독
        self.subscription = self.create_subscription(
            Odometry,
            '/pf/pose/odom',  # 토픽 이름을 실제 사용하는 것으로 변경하세요
            self.odometry_callback,
            10
        )

        self.odom_data = None 
        self.row_data = None 
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Odometry to CSV converter started')
        
    def quaternion_to_euler(self, quat):
        """쿼터니언을 오일러 각으로 변환"""
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]  # yaw (psi) 반환
    
    def calculate_curvature(self, x, y, psi, prev_x, prev_y, prev_psi):
        """곡률 계산 (간단한 근사 방법)"""
        if prev_x is None or prev_y is None or prev_psi is None:
            return 0.0
            
        # 거리 계산
        ds = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        
        if ds < 1e-6:  # 너무 작은 거리는 무시
            return 0.0
            
        # 각도 변화량 계산
        dpsi = psi - prev_psi
        
        # 각도를 -π ~ π 범위로 정규화
        while dpsi > math.pi:
            dpsi -= 2 * math.pi
        while dpsi < -math.pi:
            dpsi += 2 * math.pi
            
        # 곡률 계산: κ = dψ/ds
        kappa = dpsi / ds if ds > 0 else 0.0
        
        return kappa
    
    def odometry_callback(self, msg):
        self.odom_data = msg 

    def timer_callback(self):

        if not isinstance(self.odom_data, Odometry):
            return
        
        msg = self.odom_data

        # 현재 시간
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 위치 정보 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # 방향각(psi) 계산 - 쿼터니언에서 yaw 추출
        psi = self.quaternion_to_euler(msg.pose.pose.orientation)
        
        # 속도 정보 추출
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        # 실제 전진 속도 계산 (차량 좌표계 기준)
        velocity_magnitude = math.sqrt(vx**2 + vy**2)
        
        # 가속도 계산 (이전 속도와의 차이로 근사)
        ax = 0.0
        if self.prev_vx is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                ax = (velocity_magnitude - self.prev_vx) / dt
        
        # 거리(s) 계산 - 누적 거리
        if self.prev_x is not None and self.prev_y is not None:
            distance_increment = math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
            self.total_distance += distance_increment
        
        # 곡률(kappa) 계산
        kappa = self.calculate_curvature(x, y, psi, self.prev_x, self.prev_y, self.prev_psi)
        
        # CSV에 데이터 쓰기
        row_data = [
            self.total_distance,  # s_m
            x,                    # x_m  
            y,                    # y_m
            psi,                  # psi_rad
            kappa,                # kappa_radpm
            velocity_magnitude,   # vx_mps
            ax                    # ax_mps2
        ]

        if row_data != self.row_data:
            self.row_data = row_data
        else:
            return
        
        self.csv_writer.writerow(self.row_data)
        self.csv_file.flush()  # 즉시 파일에 쓰기
        
        # 로그 출력
        self.get_logger().info(
            f'Data written: s={self.total_distance:.3f}, '
            f'x={x:.3f}, y={y:.3f}, psi={psi:.3f}, '
            f'kappa={kappa:.6f}, vx={velocity_magnitude:.3f}, ax={ax:.3f}'
        )
        
        # 이전 값들 업데이트
        self.prev_x = x
        self.prev_y = y
        self.prev_psi = psi
        self.prev_vx = velocity_magnitude
        self.prev_time = current_time
        
    def __del__(self):
        """소멸자에서 파일 닫기"""
        if hasattr(self, 'csv_file'):
            self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)

    odometry_to_csv = OdometryToCSV()
    
    try:
        rclpy.spin(odometry_to_csv)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_to_csv.destroy_node()

if __name__ == '__main__':
    main()
