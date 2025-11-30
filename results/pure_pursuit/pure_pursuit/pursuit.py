import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA, Header
import time 
import csv
import os 
import sys
import numpy as np

from .ppp import PurePursuitPlanner


class Racecar(Node):

    def __init__(self):
        namespace = 'pure_pursuit'

        super().__init__(namespace)

        basic_qos = QoSProfile(depth=10)

        ### ROS2 Initalization Parameters
        drive_topic = self.declare_parameter('drive_topic', '/drive').value
        laser_topic = self.declare_parameter('laser_topic', '/scan').value
        odom_topic = self.declare_parameter('odom_topic', '/pf/pose/odom').value
        hz = self.declare_parameter('hz', 40).value
        
        ### Vehicle Model Parameters
        wheelbase = self.declare_parameter('wheelbase', 0.3302).value

        ### Driving Algorithm Parameters
        lookahead = self.declare_parameter('lookahead_distance', 0.82461887897713965).value
        sprint_speed = self.declare_parameter('sprint_speed', 4.0).value
        v_gain = self.declare_parameter('velocity_gain', 1.375).value 
        max_reacquire = self.declare_parameter('max_reacquire', 20.0).value
        waypoint_path = self.declare_parameter('waypoint_path', "").value

        assert isinstance(drive_topic, str), "drive_topic parameter must be a str"
        assert isinstance(laser_topic, str), "laser_topic parameter must be a str"
        assert isinstance(odom_topic, str), "odom_topic parameter must be a str"
        assert isinstance(hz, int), "hz parameter must be a int"
        assert isinstance(wheelbase, float), "wheelbase parameter must be a float"
        assert isinstance(lookahead, float), "lookahead distance must be a float"
        assert isinstance(sprint_speed, float), "sprint speed must be a float"
        assert isinstance(v_gain, float), "velocity gain must be a float"
        assert isinstance(max_reacquire, float), "max reacquire must be a float"
        assert isinstance(waypoint_path, str), "waypoint path must be a str"

        # Check waypoint file is exist
        # try:
        #     for path in sys.path:
        #         if namespace in path:
        #             wpt = os.path.join(path.split("lib")[0], "share", namespace, waypoint_path)
        #             if os.path.isfile(wpt):
        #                 waypoint_path = wpt
        #     self.get_logger().info(f"Grab waypoint: {waypoint_path}")
        # except (FileNotFoundError, Exception):
        #     self.get_logger().error("Cannot load waypoint file.")
        #     exit()

        self.pub = self.create_publisher(AckermannDriveStamped, drive_topic, basic_qos)
        self.marker_publisher = self.create_publisher(MarkerArray, 'waypoint_markers', 10)
        self.sub = {
            'odom': self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data)
        }
        #### changed ########
        # qos_profile = QoSProfile(depth= 10)
        # self.pose_sub = self.create_subscription(
        #     PoseStamped,
        #     '/pf/pose',
        #     self.pose_callback,
        #     qos_profile
        # )
     
        self.wheelbase = wheelbase
        self.Hz = hz
        self.timer = self.create_timer((1/self.Hz), self.publish_callback)

        self.planner = PurePursuitPlanner(
            ld=lookahead,
            wb=wheelbase,
            ss=sprint_speed,
            vg=v_gain,
            mr=max_reacquire,
            wp=waypoint_path
        )

        # viz 
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoint_scale', 0.1)
        self.declare_parameter('arrow_scale', 0.5)
        self.declare_parameter('line_width', 0.1)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.waypoint_scale = self.get_parameter('waypoint_scale').get_parameter_value().double_value
        self.arrow_scale = self.get_parameter('arrow_scale').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value


        self.csv_file = waypoint_path
        self.waypoints = []
        self.load_waypoints()

        self.get_logger().info(f'Waypoint visualizer started with {len(self.waypoints)} waypoints')
        
        self.lookahead_point = None
        self.last_pub = 0.0
        self.odom_data = None
        self.pose_data=None

    def load_waypoints(self):
        """CSV 파일에서 웨이포인트 데이터 로드"""
        try:
            with open(self.csv_file, 'r') as file:
                csv_reader = csv.DictReader(file, delimiter=';')
                for row in csv_reader:
                    waypoint = {
                        's_m': float(row['s_m']),
                        'x_m': float(row['x_m']),
                        'y_m': float(row['y_m']),
                        'psi_rad': float(row['psi_rad']),
                        'kappa_radpm': float(row['kappa_radpm']),
                        'vx_mps': float(row['vx_mps']),
                        'ax_mps2': float((row.get('ax_mps2') or row.get('ax_mps') or 0.0))
                    }
                    self.waypoints.append(waypoint)
                    
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.csv_file}')
            
        except FileNotFoundError:
            self.get_logger().error(f'CSV file not found: {self.csv_file}')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV file: {str(e)}')
    
    def create_waypoint_markers(self):
        """웨이포인트를 구 마커로 생성"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        
        # 마커 스케일 설정
        marker.scale = Vector3(x=self.waypoint_scale, y=self.waypoint_scale, z=self.waypoint_scale)
        
        # 색상 설정 (속도에 따라 색상 변경)
        marker.colors = []
        
        # 최대/최소 속도 찾기 (색상 정규화용)
        velocities = [wp['vx_mps'] for wp in self.waypoints]
        max_vel = max(velocities) if velocities else 1.0
        min_vel = min(velocities) if velocities else 0.0
        vel_range = max_vel - min_vel if max_vel != min_vel else 1.0
        
        for waypoint in self.waypoints:
            # 포인트 추가
            point = Point()
            point.x = waypoint['x_m']
            point.y = waypoint['y_m']
            point.z = 0.1  # 바닥보다 조금 위에
            marker.points.append(point)
            
            # 속도에 따른 색상 설정 (파란색=느림, 빨간색=빠름)
            # normalized_vel = (waypoint['vx_mps'] - min_vel) / vel_range
            # color = ColorRGBA()
            # color.r = normalized_vel
            # color.g = 0.0
            # color.b = 1.0 - normalized_vel
            # color.a = 0.8
            # marker.colors.append(color)
            color = ColorRGBA()
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
            color.a = 0.8
            marker.colors.append(color)
        
        if self.lookahead_point and len(self.lookahead_point) == 3:
            point = Point()
            point.x = self.lookahead_point[0]
            point.y = self.lookahead_point[1]
            point.z = 0.1  # 바닥보다 조금 위에
            marker.points.append(point)

            color = ColorRGBA()
            color.r = 0.0
            color.g = 0.0
            color.b = 1.0
            color.a = 0.8
            marker.colors.append(color)

            
        return marker

    def publish_marker(self):
        if not self.waypoints:
            return
        if self.last_pub + 0.1 >= time.time():
            return 

        marker_array = MarkerArray()
        
        # 웨이포인트 마커 추가
        waypoint_marker = self.create_waypoint_markers()
        marker_array.markers.append(waypoint_marker)
        self.marker_publisher.publish(marker_array)
        self.last_pub = time.time()


    def publish_callback(self, _finally=None):
        """
        Publish to /drive topic.

        :return: None
        """
        self.publish_marker()
        lp = None 

        if not self.odom_data:
            self.get_logger().warn("No Odometry data. Skip this callback")
            return 
        # if not self.pose_data:
        #     return
            
        msg = AckermannDriveStamped()
        obs = self.odom_data
        self.get_logger().info(f"pose: [{obs['pose_x']},{obs['pose_y']},{obs['pose_theta']}]")

        if hasattr(self.planner, 'plan'):
            speed, steer, lp = self.planner.plan(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        elif hasattr(self.planner, 'driving'):
            speed, steer, lp = self.planner.driving(obs['pose_x'], obs['pose_y'], obs['pose_theta'])
        else:
            self.get_logger().error("Planner doesn't have `plan` or `driving` function.")
            speed, steer = 0.0, 0.0

        if _finally:
            self.get_logger().info("SIGINT. stopping the vehicle")
            speed, steer = 0.0, 0.0
        

        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.lookahead_point = list(lp)
        self.get_logger().info(f"speed: {speed}, steer: {steer}, lookahead_point: {lp}")

        
        self.pub.publish(msg)


    def odom_callback(self, msg: Odometry):
        """
        Update self Odomerty data.

        :param msg: nav_msgs.msg/Odometry
        :return: class variable update
        """
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.odom_data = {
            'pose_x': msg.pose.pose.position.x,
            'pose_y': msg.pose.pose.position.y,
            'pose_theta': np.arctan2(siny_cosp, cosy_cosp),
            'linear_vels_x': msg.twist.twist.linear.x,
            'linear_vels_y': msg.twist.twist.linear.y,
            'ang_vels_z': msg.twist.twist.angular.z
        }

    def pose_callback(self, msg: PoseStamped):
        """
        Update self Odomerty data.

        :param msg: nav_msgs.msg/Odometry
        :return: class variable update
        """

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.pose_data = {
            'pose_x': msg.pose.pose.position.x,
            'pose_y': msg.pose.pose.position.y,
            'pose_theta': yaw
        }

def main(args=None):
    rclpy.init(args=args)
    racecar = Racecar()
    try:
        rclpy.spin(racecar)
    except KeyboardInterrupt:
        racecar.publish_callback(_finally=True)
    finally:
        racecar.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
