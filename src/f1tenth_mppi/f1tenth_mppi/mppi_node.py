#!/usr/bin/python3
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from safe_path_msg.msg import SafePath

from f1tenth_mppi.utils import *
from f1tenth_mppi.dynamics_models import KBM

class MPPI(Node):
    def __init__(self):
        super().__init__('MPPI')

        # Initialize Parameters
        self.initialize_parameters()
        self.u_mean = np.array([self.min_throttle, 0.0]) # Mean for sampling trajectories

        # Initialize Dynamics Model
        self.model = KBM(self.wheelbase,
                         self.min_throttle,
                         self.max_throttle,
                         self.max_steer,
                         self.dt)

        # Initialize Cost Map
        self.cost_map = OccupancyGrid()
        self.cost_map.header.frame_id = self.vehicle_frame
        self.cost_map.info.resolution = self.cost_map_res 
        self.cost_map.info.width = self.cost_map_width
        self.cost_map.info.height = self.cost_map_width
        self.cost_map.info.origin.position.x = 0.0  
        self.cost_map.info.origin.position.y = -(self.cost_map_width * self.cost_map_res) / 2

        # Load Waypoints
        try:
            self.waypoints = load_waypoints(self.waypoint_path)
        except Exception as e:
            self.get_logger().error(f"error loading waypoints file: {e}")

        # Setup the TF2 buffer and listener to capture transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create Publishers
        self.trajectory_pub_ = self.create_publisher(
            # Path,
            SafePath,
            self.trajectory_topic,
            10
        )

        if self.visualize:
            self.cost_map_pub_ = self.create_publisher(
                OccupancyGrid,
                self.cost_map_topic,
                10
            )
            self.marker_pub_ = self.create_publisher(
                MarkerArray,
                self.marker_topic,
                10
            )
            self.publish_markers(self.waypoints, np.array([1.0, 0.0, 0.0]), "waypoints")

        self.occupancy_sub_ = self.create_subscription(
            OccupancyGrid,
            self.occupancy_topic,
            self.occupancy_callback,
            10
        )
        self.occupancy_grid = None 

        self.timer_ = self.create_timer(
            1/15,
            self.trajectory_callback
        )
        self.get_logger().info("MPPI node initialized")

    def initialize_parameters(self):
        '''
        Initialize all parameters as None so parameters can be loaded from yaml
        '''
        self.declare_parameters(
            namespace='',
            parameters=[
                ('visualize', True),

                # Strings
                ('waypoint_path', '251025.csv'),
                ('vehicle_frame', 'laser'),

                ('trajectory_topic', '/safe_trajectory'),
                ('cost_map_topic', '/cost_map'),
                ('marker_topic', '/markers'),

                ('occupancy_topic', '/local_map'),
                ('pose_topic', '/pf/pose/odom'),


                # 고정 파라미터
                ('wheelbase', 0.33),
                ('max_steer', 0.8378),

                # 차량
                ('min_throttle', 0.75),
                ('max_throttle', 1.0),

                # MPPI 경로 샘플링 파라미터
                # 예측 경로를 생성할 때 사용되는 시간 간격(단위: 초)
                ('dt', 0.1),
                # 매 스텝 마다 생성할 무작위 예측 경로의 총 갯수
                ('num_trajectories', 500),
                # 한개의 예측 경로에서 n개의 스텝으로 이루어지는지 갯수
                ('steps_trajectories', 30),
                # 속도(v), 조향 각속도(omega)에 대한 무작위 노이즈 표준 편차
                ('v_sigma', 0.05),
                ('omega_sigma', 0.2),

                # 코스트맵 및 비용 함수 파라미터
                # 코스트 맵 그리드 한 칸의 실제 크기(셀 크기) 0.05=5cm
                ('cost_map_res', 0.05),
                # 코스트 맵 넓이
                ('cost_map_width', 100),
                # waypoints에서 벗어났을 때 부여하는 페널티의 강도
                ('raceline_dilation', 3),
                # 차량의 진행 방향(heading)이 주행 라인의 방향과 다를 때 부여하는 페널티
                ('heading_weight', 0.0),
                # waypoints을 몇 개의 셀만큼 두껍게 만들지 결정합니다.
                ('raceline_weight', 1.0),
                # 경로가 장애물에 가까워질 때 부여하는 페널티의 강도입니다.
                ('obstacle_weight', 1.5),
            ])

        # Set all parameters as class variables
        self.visualize = self.get_parameter('visualize').value
        
        # String parameters
        self.waypoint_path = self.get_parameter('waypoint_path').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        self.occupancy_topic = self.get_parameter('occupancy_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value

        self.cost_map_topic = self.get_parameter('cost_map_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value

        
        # Fixed parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steer').value
        
        # Vehicle parameters
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        
        # MPPI sampling parameters
        self.dt = self.get_parameter('dt').value
        self.num_trajectories = self.get_parameter('num_trajectories').value
        self.steps_trajectories = self.get_parameter('steps_trajectories').value
        self.v_sigma = self.get_parameter('v_sigma').value
        self.omega_sigma = self.get_parameter('omega_sigma').value
        
        # Cost map and cost function parameters
        self.cost_map_res = self.get_parameter('cost_map_res').value
        self.cost_map_width = self.get_parameter('cost_map_width').value
        self.raceline_dilation = self.get_parameter('raceline_dilation').value
        self.heading_weight = self.get_parameter('heading_weight').value
        self.raceline_weight = self.get_parameter('raceline_weight').value
        self.obstacle_weight = self.get_parameter('obstacle_weight').value

    
    def occupancy_callback(self, msg: OccupancyGrid):
        if isinstance(msg, OccupancyGrid):
            self.get_logger().info("Received OccupancyGrid msg")
            self.occupancy_grid = msg
        return 

    def trajectory_callback(self):
        
        t0 = self.get_clock().now()
        if not isinstance(self.occupancy_grid, OccupancyGrid):
            return 
        
        # self.get_logger().info("Creating cost map")
        try:
            grid = np.array(self.occupancy_grid.data).reshape(
                (self.occupancy_grid.info.height, 
                 self.occupancy_grid.info.width)
            )
            cost_map = self.update_cost_map(grid)
        except Exception as e:
            self.get_logger().warn("Error updating cost map, skipping iteration")
            print(e)
            return 
        
        # self.get_logger().info("Sampling trajectories")
        trajectories, actions = self.sample_trajectories(self.num_trajectories,
                                                         self.steps_trajectories)

        # Evaluate Trajectories
        # self.get_logger().info("Evaluating trajectories")
        min_cost_idx = self.evaluate_trajectories(cost_map, trajectories, grid)

        safe_path = SafePath()
        safe_path.header.stamp = self.get_clock().now().to_msg()

        if min_cost_idx == -1:
            safe_path.flag = False
            safe_path.path = Path()
            self.trajectory_pub_.publish(safe_path)
            return 
        

        # Transfrom absolute
        # 2. 이동(Translation) 및 회전(Rotation) 정보 추출
        transform = self.tf_buffer.lookup_transform('map', self.vehicle_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # 쿼터니언에서 Yaw 각도 추출 (2D 회전을 위해)
        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        # 2D 회전 행렬(R)과 이동 벡터(T) 생성
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                          [np.sin(yaw),  np.cos(yaw)]])
        T = np.array([translation.x, translation.y])

        # 3. 최적 경로의 상대 좌표(x, y)를 선택
        best_trajectory_relative = trajectories[min_cost_idx]
        xy_points_relative = best_trajectory_relative[:, :2] # (x, y)만 사용

        # 4. 행렬 연산을 통해 모든 점을 한번에 절대 좌표로 변환
        # P_absolute = R * P_relative + T
        xy_points_absolute = np.dot(xy_points_relative, R.T) + T

        # (선택) 절대 좌표계에서의 경로 각도(theta) 계산
        # 상대 경로의 각도에 현재 차량의 yaw 각도를 더해줍니다.
        theta_absolute = best_trajectory_relative[:, 2] + yaw

        # 변환된 절대 좌표 경로 (x, y, theta)
        best_trajectory_absolute = np.hstack([xy_points_absolute, theta_absolute[:, np.newaxis]])
        
        self.u_mean[0] = actions[min_cost_idx, 0, 0]
        safe_trj = best_trajectory_absolute

        safe_path.flag = True
        
        safe_path.path.header.frame_id = self.vehicle_frame
        safe_path.path.header.stamp = self.get_clock().now().to_msg()

        for i in safe_trj:
            pose = PoseStamped()
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            pose.pose.orientation.w = i[2]
            safe_path.path.poses.append(pose)

        self.trajectory_pub_.publish(safe_path)
        
        t1 = self.get_clock().now()
        duration_ns = (t1 - t0).nanoseconds
        duration_s = duration_ns / 1e9  # nanoseconds를 seconds로 변환
        iter_hz = 1.0 / duration_s if duration_s > 0 else 0.0  # Hz 계산 (1/초)
        if (int(duration_s) * 10) % 10 == 0: 
            self.get_logger().info(f"Publish trajectory / iters_per_sec: {round(iter_hz, 3)}Hz")
    
    
    def update_cost_map(self, occupancy_grid: np.ndarray) -> np.ndarray:
        '''
        Update cost map based on current environment.
        We want areas that deviate from the raceline to have higher cost.
        Areas that are closer to obstacles in the occupancy_grid should have higher cost.

        Args:
            occupancy_grid (ndarray): The processed occupancy grid
        Returns:
            cost_map (ndarray): The cost map
        '''

        # Get transform from map to ego frame
        transform = self.tf_buffer.lookup_transform(self.vehicle_frame,
                                                    "map", rclpy.time.Time(), timeout=Duration(seconds=0.1))

        # Get vehicle yaw
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        # Determine translation and rotation
        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw),  np.cos(yaw)]])
        T = np.array([transform.transform.translation.x,
                      transform.transform.translation.y])

        # Transform all waypoints from map to ego frame
        points_transformed = np.dot(self.waypoints[:, :2], R.T) + T

        # Convert the transformed points into grid indices.
        x_idx = np.round((points_transformed[:, 0] / self.cost_map.info.resolution)).astype(int)
        y_idx = np.round((points_transformed[:, 1] / self.cost_map.info.resolution) + self.cost_map.info.height / 2).astype(int)

        raceline_mask = np.zeros_like(occupancy_grid, dtype=int)
        mask_shape = raceline_mask.shape
        valid = (x_idx >= 0) & (x_idx < mask_shape[0]) & (y_idx >= 0) & (y_idx < mask_shape[1])
        raceline_mask[y_idx[valid], x_idx[valid]] = 100

        # Apply binary dilation to expand raceline
        # dilation_kernel = np.array([[0, 1, 0],
        #                             [1, 1, 1],
        #                             [0, 1, 0]], dtype=bool)
        dilation_kernel = np.ones((self.raceline_dilation, self.raceline_dilation), dtype=bool)
        raceline_mask = binary_dilation(raceline_mask, structure=dilation_kernel).astype(int) * 100

        # Compute the distance from raceline, for each grid cell
        raceline_cost = distance_transform_edt(raceline_mask == 0)

        # Final cost map is a weighted sum of the obstacle cost and raceline cost
        # cost_map = self.raceline_weight * raceline_cost
        # cost_map = 100 * (cost_map / cost_map.max()) # normalize so that it's within 0-100
        # cost_map = np.clip(cost_map, 0, 100).astype(int) # NOTE: Should we be clipping or normalizing? Do we need to?

        # 1. 각 비용 요소(장애물, 주행라인)의 가중치를 파라미터에서 가져옵니다.
        obstacle_weight = self.obstacle_weight
        raceline_weight = self.raceline_weight
        
        # 2. 장애물 비용(occupancy_grid)과 주행라인 비용(raceline_cost)을 가중 합산하여 최종 코스트맵을 계산합니다.
        cost_map = (obstacle_weight * occupancy_grid) + (raceline_weight * raceline_cost)
        
        # 3. 계산된 코스트맵의 최대값이 0인 경우(맵이 비어있는 등)를 대비해 0으로 나누는 것을 방지합니다.
        if cost_map.max() > 0:
            # 4. 코스트맵의 모든 값을 0~100 사이로 정규화(normalize)합니다.
            #    이렇게 하면 가장 비용이 높은 곳이 100이 되며, 비용의 상대적인 차이가 유지됩니다.
            cost_map = 100 * (cost_map / cost_map.max())
        
        # 5. 모든 값을 정수(int)로 변환하여 최종 코스트맵을 완성합니다.
        cost_map = cost_map.astype(int)

        if self.visualize:
            # normalize so that it's within 0-100
            cost_map_vis = 100 * (cost_map / cost_map.max())
            self.cost_map.data = cost_map_vis.astype(int).flatten().tolist()
            self.cost_map.header.stamp = self.get_clock().now().to_msg()
            self.cost_map_pub_.publish(self.cost_map)

        # self.get_logger().info(f"{cost_map / cost_map.max()}")
        return cost_map / cost_map.max()

    def sample_trajectories(self, num_trajectories: int, steps_trajectories: int):
        '''
        Sample random actions from distribution and generate trajectories using model

        Args:
            num_trajectories (int): The number of trajectories to sample
            steps_trajectories (int): The number of steps for each trajectory
        Returns:
            trajectories (ndarray): (num_trajectories x steps_trajectories x 3) array of trajectories
            actions (ndarray): (num_trajectories x steps_trajectories - 1) array of actions
        '''

        # Sample control values
        # v = self.u_mean[0] + np.random.randn(num_trajectories, steps_trajectories - 1, 1) * self.v_sigma
        # omega = self.u_mean[1] + np.random.randn(num_trajectories, steps_trajectories - 1, 1) * self.omega_sigma

        v = (self.max_throttle + np.random.randn(num_trajectories, 1, 1) * self.v_sigma)
        omega = (self.u_mean[1] + np.random.randn(num_trajectories, 1, 1) * self.omega_sigma)

        v = np.repeat(v, steps_trajectories - 1, axis=1)
        omega = np.repeat(omega, steps_trajectories - 1, axis=1)

        # Limit control values
        v = np.clip(v, self.min_throttle, self.max_throttle)
        omega = np.clip(omega, -self.max_steer, self.max_steer)

        actions = np.concatenate((v, omega), axis=2)

        # Sample trajectories
        trajectories = np.zeros((num_trajectories, steps_trajectories, 3))
        for i in range(steps_trajectories - 1):
            trajectories[:, i + 1] = self.model.predict_euler(trajectories[:, i], actions[:, i])

        # Publish a subset of trajectories
        self.publish_trajectories(trajectories[:5])

        return trajectories, actions
    
    def evaluate_trajectories(self, cost_map: np.ndarray, trajectories: np.ndarray, occupancy_grid: np.ndarray) -> int:
        '''
        Evaluate trajectories using the cost map

        Args:
            cost_map (ndarray): The cost map
            trajectories (np.ndarray): (num_trajectories x steps_trajectories x 3) Sampled trajectories
            occupancy_grid (ndarray): The processed occupancy grid
        Returns:
            min_cost_idx (int): The index of the trajectory with the lowest cost
        '''

        # Convert each trajectory to the cost map frame
        trajectories_pixels = trajectories / self.cost_map.info.resolution
        trajectories_pixels[:, :, 0] = trajectories_pixels[:, :, 0]
        trajectories_pixels[:, :, 1] = trajectories_pixels[:, :, 1] + self.cost_map.info.height/2

        # Handle trajectories that fall outside of the cost map
        trajectories_pixels = np.clip(trajectories_pixels[:, :, :2], 0, self.cost_map.info.width - 1)

        # if a trajectory touches an obstacle, then just completely ignore it 
        # print(f"shape of traj pixels is {trajectories_pixels.shape}") # NxTx3
        # print(f"Shape of occupancy grid is {occupancy_grid.shape}")

        # bad_traj = [1, 0, 1, 0, 0,.. N]
        check_obs = occupancy_grid.astype(bool)[trajectories_pixels[:, :, 1].astype(int), trajectories_pixels[:, :, 0].astype(int)] # NxT
        bad_trajs = np.any(check_obs==True, axis=1) # (N,)

        for i in range(check_obs.shape[0]):
            try:
                idx = np.where(check_obs[i] == True)[0][0]
                check_obs[i, idx:] = True
            except:
                pass

        # Compute each trajectory's position score and normalize
        traj_scores = np.sum(cost_map[trajectories_pixels[:, :, 1].astype(int), trajectories_pixels[:, :, 0].astype(int)], axis=1)
        traj_scores /= traj_scores.max()

        traj_scores[bad_trajs] = np.inf
        # if np.all(traj_scores == np.inf):
        #     return -1
        min_cost_idx = np.argmin(traj_scores)

        # Publish a lowest cost trajectory
        self.publish_trajectories(np.expand_dims(trajectories[min_cost_idx], 0), np.array([1.0, 0.0, 0.0]))

        return min_cost_idx

    def publish_trajectories(self, points: np.ndarray, color: np.ndarray = np.array([0.0, 0.0, 1.0])):
        '''
        Publish MarkerArray message of lines

        Args:
            points (ndarray): NxMx2 array of points to publish
            color (ndarray): The color of the points
        '''

        if not self.visualize:
            return

        # Generate MarkerArray message
        marker_array = MarkerArray()

        for j in range(points.shape[0]):
            for i in range(points.shape[1] - 1):
                marker = Marker()
                marker.header.frame_id = self.vehicle_frame
                marker.id = j * points.shape[1] + i
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0
                marker.scale.x = 0.01

                p1 = Point()
                p1.x = points[j, i, 0]
                p1.y = points[j, i, 1]
                p2 = Point()
                p2.x = points[j, i+1, 0]
                p2.y = points[j, i+1, 1]

                marker.points = [p1, p2]
                marker_array.markers.append(marker)

        # Publish MarkerArray Message
        self.publish_markers(self.waypoints, np.array([1.0, 0.0, 0.0]), "waypoints")
        self.marker_pub_.publish(marker_array)

    def publish_markers(self, points: np.ndarray, color: np.ndarray = np.array([1.0, 0.0, 0.0]), ns: str = ""):
        '''
        Publish MarkerArray message of points

        Args:
            points (ndarray): Nx2 array of points to publish
            color (ndarray): The color of the points
        '''

        if not self.visualize:
            return
        
        # Generate MarkerArray message
        marker_array = MarkerArray()
        for i in range(len(points)):
            marker = Marker()
            marker.ns = ns
            marker.header.frame_id = "map"
            marker.id = i + 1
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = points[i, 0]
            marker.pose.position.y = points[i, 1]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker_array.markers.append(marker)

        # Publish MarkerArray Message
        self.marker_pub_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    mppi_node = MPPI()
    rclpy.spin(mppi_node)

    mppi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
