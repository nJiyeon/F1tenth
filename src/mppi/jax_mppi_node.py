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
from f1tenth_mppi.dynamics_models import KBM  # 여전히 사용(시각화용/비교용)

# ========== JAX ========== #
import jax
import jax.numpy as jnp
from functools import partial
# ========================= #


# =========================
# JAX BACKEND HELPERS
# =========================
class JaxRNG:
    def __init__(self, seed=0):
        self.key = jax.random.PRNGKey(seed)
    def split(self, n=1):
        self.key, *keys = jax.random.split(self.key, n+1)
        return keys if n > 1 else keys[0]

def _normal_clip(key, mean, sigma, low, high, shape, dtype=jnp.float32):
    z = mean + sigma * jax.random.normal(key, shape, dtype=dtype)
    return jnp.clip(z, low, high)

@partial(jax.jit, static_argnums=())
def _kbm_predict_euler_jax(state_batch, action_batch, dt, wheelbase, max_steer):
    """
    state_batch: (N, 3) [x, y, theta]
    action_batch: (N, 2) [v, omega]  (omega: 조향각속도 근사)
    """
    x, y, th = state_batch[:, 0], state_batch[:, 1], state_batch[:, 2]
    v, omega = action_batch[:, 0], action_batch[:, 1]

    # 간단 근사: steer ≈ omega * dt (필요시 '누적 조향각' 모델로 바꿔도 됨)
    steer_small = jnp.clip(omega * dt, -max_steer, max_steer)

    x_next  = x  + v * jnp.cos(th) * dt
    y_next  = y  + v * jnp.sin(th) * dt
    th_next = th + v * jnp.tan(steer_small) / wheelbase * dt
    return jnp.stack([x_next, y_next, th_next], axis=1)

@partial(jax.jit, static_argnums=(4,5,6,7,8)) #(5,6,7))
def _sample_actions_jax(key, u_mean, v_sigma, omega_sigma, num_traj, T, min_thr, max_thr, max_steer):
    """
    returns actions: (N, T-1, 2)
    u_mean: (2,) array([min_throttle, 0.0])
    """
    k1, k2 = jax.random.split(key)
    v = _normal_clip(k1, u_mean[0], v_sigma, min_thr, max_thr, (num_traj, T-1, 1))
    omega = _normal_clip(k2, u_mean[1], omega_sigma, -max_steer, max_steer, (num_traj, T-1, 1))
    return jnp.concatenate([v, omega], axis=2)

@partial(jax.jit, static_argnums=(4,))
def _rollout_jax(init_states, actions, dt, wheelbase, max_steer):
    """
    init_states: (N, 3)
    actions: (N, T-1, 2)
    return trajectories: (N, T, 3)
    """
    def step(carry, u_t):  # scan over time
        s_t = carry
        s_tp1 = _kbm_predict_euler_jax(s_t, u_t, dt, wheelbase, max_steer)
        return s_tp1, s_tp1

    s0 = init_states
    _, traj_body = jax.lax.scan(step, s0, actions.swapaxes(0,1))  # (T-1, N, 3)
    traj_body = traj_body.swapaxes(0,1)                            # (N, T-1, 3)
    return jnp.concatenate([s0[:, None, :], traj_body], axis=1)    # (N, T, 3)

@partial(jax.jit, static_argnums=())
def _evaluate_trajs_jax(cost_map, occ_grid, trajectories, res, half_h):
    """
    cost_map: (H, W) float/int (0~1 or 0~100)
    occ_grid: (H, W) int/bool (장애물>0)
    trajectories: (N, T, 3)
    res: resolution (float)
    half_h: H/2 (float)
    returns: (min_idx, scores, bad_mask, all_inf)
    """
    H, W = cost_map.shape
    N, T, _ = trajectories.shape

    xs = trajectories[:, :, 0] / res
    ys = trajectories[:, :, 1] / res + half_h

    xs = jnp.clip(xs, 0, W - 1).astype(jnp.int32)
    ys = jnp.clip(ys, 0, H - 1).astype(jnp.int32)

    flat_idx = ys * W + xs
    flat_cost = cost_map.reshape(-1)
    flat_occ  = occ_grid.reshape(-1)

    traj_costs = jnp.take(flat_cost, flat_idx)                # (N, T)
    traj_occ   = (jnp.take(flat_occ, flat_idx) > 0)           # (N, T)

    # “처음 True 이후 전부 True” 마스크
    def or_carry(c, x):
        nx = jnp.logical_or(c, x)
        return nx, nx
    _, occ_prefix = jax.lax.scan(or_carry, jnp.zeros((N,), dtype=bool), traj_occ.swapaxes(0,1))
    occ_prefix = occ_prefix.swapaxes(0,1)  # (N, T)

    scores = jnp.sum(traj_costs, axis=1).astype(jnp.float32)
    scores = scores / (jnp.max(scores) + 1e-8)

    bad = jnp.any(traj_occ, axis=1)
    scores = jnp.where(bad, jnp.inf, scores)

    all_inf = jnp.all(jnp.isinf(scores))
    min_idx = jnp.argmin(scores)
    return min_idx, scores, bad, all_inf
# =========================
# END JAX BACKEND
# =========================


class MPPI(Node):
    def __init__(self):
        super().__init__('MPPI')

        # Initialize Parameters
        self.initialize_parameters()
        self.u_mean = np.array([self.min_throttle, 0.0], dtype=np.float32)  # [v_mean, omega_mean]

        # (선택) 기존 KBM 객체는 남겨두되, 주 롤아웃은 JAX가 담당
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
            self.waypoints = np.zeros((1, 2), dtype=np.float32)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.trajectory_pub_ = self.create_publisher(SafePath, self.trajectory_topic, 10)

        if self.visualize:
            self.cost_map_pub_ = self.create_publisher(OccupancyGrid, self.cost_map_topic, 10)
            self.marker_pub_ = self.create_publisher(MarkerArray, self.marker_topic, 10)
            self.publish_markers(self.waypoints, np.array([1.0, 0.0, 0.0]), "waypoints")

        self.occupancy_sub_ = self.create_subscription(
            OccupancyGrid, self.occupancy_topic, self.occupancy_callback, 10
        )
        self.occupancy_grid = None

        self.timer_ = self.create_timer(1/10, self.trajectory_callback)

        # JAX 상태
        self._jax_rng = JaxRNG(seed=42)
        self._jax_params = (float(self.wheelbase), float(self.max_steer))

        self.get_logger().info("MPPI (JAX) node initialized")

    def initialize_parameters(self):
        '''
        Initialize all parameters as None so parameters can be loaded from yaml
        '''
        self.declare_parameters(
            namespace='',
            parameters=[
                ('visualize', True),

                # Strings
                ('waypoint_path', 'levine_1.csv'),
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
                ('dt', 0.1),
                ('num_trajectories', 100),
                ('steps_trajectories', 25),
                ('v_sigma', 0.05),
                ('omega_sigma', 0.2),

                # 코스트맵 및 비용 함수 파라미터
                ('cost_map_res', 0.05),
                ('cost_map_width', 100),
                ('raceline_dilation', 3),
                ('heading_weight', 0.0),
                ('raceline_weight', 1.0),
                ('obstacle_weight', 1.5),
            ])

        # Set all parameters as class variables
        self.visualize = self.get_parameter('visualize').value

        # String parameters
        self.waypoint_path = self.get_parameter('waypoint_path').value
        self.vehicle_frame  = self.get_parameter('vehicle_frame').value
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

        self.get_logger().info("Creating cost map")
        # try:
        #     grid = np.array(self.occupancy_grid.data).reshape(
        #         (self.occupancy_grid.info.height,
        #          self.occupancy_grid.info.width)
        #     )
        #     cost_map = self.update_cost_map(grid)  # (H,W), normalized [0,1]
        # except Exception as e:
        #     self.get_logger().warn("Error updating cost map, skipping iteration")
        #     print(e)
        #     return
        grid = np.array(self.occupancy_grid.data).reshape(
                (self.occupancy_grid.info.height,
                 self.occupancy_grid.info.width)
            )
        cost_map = self.update_cost_map(grid)  # (H,W), normalized [0,1]

        # ---------- JAX: Sample + Rollout + Evaluate ----------
        self.get_logger().info("Sampling + Rollout + Evaluate (JAX)")

        cost_map_j = jnp.asarray(cost_map, dtype=jnp.float32)
        occ_j = jnp.asarray((grid > 0).astype(np.int32))  # 장애물 1/0

        key = self._jax_rng.split()

        actions_j = _sample_actions_jax(
            key=key,
            u_mean=jnp.asarray(self.u_mean, dtype=jnp.float32),
            v_sigma=self.v_sigma,
            omega_sigma=self.omega_sigma,
            num_traj=int(self.num_trajectories),
            T=int(self.steps_trajectories),
            min_thr=float(self.min_throttle),
            max_thr=float(self.max_throttle),
            max_steer=float(self.max_steer)
        )

        init_states_j = jnp.zeros((int(self.num_trajectories), 3), dtype=jnp.float32)

        wheelbase, max_steer = self._jax_params
        trajs_j = _rollout_jax(
            init_states=init_states_j,
            actions=actions_j,
            dt=float(self.dt),
            wheelbase=float(wheelbase),
            max_steer=float(max_steer)
        )

        min_idx_j, traj_scores_j, bad_mask_j, all_inf_j = _evaluate_trajs_jax(
            cost_map=cost_map_j,
            occ_grid=occ_j,
            trajectories=trajs_j,
            res=float(self.cost_map.info.resolution),
            half_h=float(self.cost_map.info.height) / 2.0
        )

        all_inf = bool(all_inf_j)
        if all_inf:
            # 전부 장애물에 막힌 경우
            safe_path = SafePath()
            safe_path.flag = False
            safe_path.path = Path()
            safe_path.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_pub_.publish(safe_path)
            return

        min_cost_idx = int(min_idx_j)
        trajectories = np.asarray(trajs_j)     # (N,T,3) -> numpy for ROS
        actions = np.asarray(actions_j)        # (N,T-1,2)
        # ------------------------------------------------------

        safe_path = SafePath()
        safe_path.header.stamp = self.get_clock().now().to_msg()

        # Transfrom: ego -> map 절대좌표
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.vehicle_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            safe_path.flag = False
            safe_path.path = Path()
            self.trajectory_pub_.publish(safe_path)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw),  np.cos(yaw)]], dtype=np.float32)
        T = np.array([translation.x, translation.y], dtype=np.float32)

        best_trajectory_relative = trajectories[min_cost_idx]
        xy_points_relative = best_trajectory_relative[:, :2]
        xy_points_absolute = np.dot(xy_points_relative, R.T) + T
        theta_absolute = best_trajectory_relative[:, 2] + yaw
        best_trajectory_absolute = np.hstack([xy_points_absolute, theta_absolute[:, np.newaxis]])

        # 다음 분포 평균 업데이트(속도 평균만 간단히)
        self.u_mean[0] = actions[min_cost_idx, 0, 0]

        safe_path.flag = True
        safe_path.path.header.frame_id = self.vehicle_frame  # 상대 프레임로 퍼블리시 원하면 map로 바꿔도 됨
        safe_path.path.header.stamp = self.get_clock().now().to_msg()

        for i in best_trajectory_absolute:
            pose = PoseStamped()
            pose.pose.position.x = float(i[0])
            pose.pose.position.y = float(i[1])
            # theta를 orientation.w에 바로 넣는 건 일반적이지 않지만, 기존 코드의 관례 유지
            pose.pose.orientation.w = float(i[2])
            safe_path.path.poses.append(pose)

        # 시각화용: 몇 개 샘플만 퍼블리시
        try:
            self.publish_trajectories(trajectories[:5])
            # 최적 경로는 빨강
            self.publish_trajectories(np.expand_dims(best_trajectory_relative, 0), np.array([1.0, 0.0, 0.0]))
        except Exception:
            pass

        self.trajectory_pub_.publish(safe_path)

        t1 = self.get_clock().now()
        duration_ns = (t1 - t0).nanoseconds
        duration_s = duration_ns / 1e9
        iter_hz = 1.0 / duration_s if duration_s > 0 else 0.0
        if (int(duration_s) * 10) % 10 == 0:
            self.get_logger().info(f"Publish trajectory / iters_per_sec: {round(iter_hz, 3)}Hz")

    def update_cost_map(self, occupancy_grid: np.ndarray) -> np.ndarray:
        '''
        Update cost map based on current environment.
        We want areas that deviate from the raceline to have higher cost.
        Areas that are closer to obstacles in the occupancy_grid should have higher cost.
        '''

        # map -> ego
        transform = self.tf_buffer.lookup_transform(
            self.vehicle_frame, "map", rclpy.time.Time(), timeout=Duration(seconds=0.1)
        )

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw),  np.cos(yaw)]], dtype=np.float32)
        T = np.array([transform.transform.translation.x,
                      transform.transform.translation.y], dtype=np.float32)

        # waypoints (map) -> ego
        if self.waypoints.shape[1] >= 2:
            points_transformed = np.dot(self.waypoints[:, :2], R.T) + T
        else:
            points_transformed = np.zeros((1, 2), dtype=np.float32)

        # grid index
        x_idx = np.round((points_transformed[:, 0] / self.cost_map.info.resolution)).astype(int)
        y_idx = np.round((points_transformed[:, 1] / self.cost_map.info.resolution) + self.cost_map.info.height / 2).astype(int)

        raceline_mask = np.zeros_like(occupancy_grid, dtype=int)
        H, W = raceline_mask.shape
        valid = (x_idx >= 0) & (x_idx < W) & (y_idx >= 0) & (y_idx < H)
        raceline_mask[y_idx[valid], x_idx[valid]] = 100

        # 굵기
        # dilation_kernel = np.ones((int(max(1, self.raceline_dilation)), int(max(1, self.raceline_dilation)))) 
        dilation_kernel = np.ones((self.raceline_dilation, self.raceline_dilation), dtype=bool)
        # dilation_kernel = np.ones((int(max(1, self.raceline_dilation)), int(max(1, self.raceline_dilation))))  # safety
        # self.get_logger().info(f"dilation_kernel: {dilation_kernel}")
        # dilation_kernel = dilation_kernel[0]  # 위 한 줄 실수 보정
        raceline_mask = binary_dilation(raceline_mask, structure=dilation_kernel).astype(int) * 100

        # raceline distance
        raceline_cost = distance_transform_edt(raceline_mask == 0)

        # weighted sum
        obstacle_weight = float(self.obstacle_weight)
        raceline_weight = float(self.raceline_weight)
        cost_map = (obstacle_weight * occupancy_grid.astype(np.float32)) + (raceline_weight * raceline_cost.astype(np.float32))

        if cost_map.max() > 0:
            cost_map = cost_map / cost_map.max()
        
        # 시각화 퍼블리시
        if self.visualize:
            vis = (100 * cost_map).astype(int)
            self.cost_map.data = vis.flatten().tolist()
            self.cost_map.header.stamp = self.get_clock().now().to_msg()
            self.cost_map_pub_.publish(self.cost_map)

        # self.get_logger().info(f"{cost_map}")
        return cost_map  # [0,1]

    def publish_trajectories(self, points: np.ndarray, color: np.ndarray = np.array([0.0, 0.0, 1.0])):
        '''
        Publish MarkerArray message of lines
        points: (N, T, 3) or (N, T, 2)
        '''
        if not self.visualize:
            return

        marker_array = MarkerArray()
        for j in range(points.shape[0]):
            for i in range(points.shape[1] - 1):
                marker = Marker()
                marker.header.frame_id = self.vehicle_frame
                marker.id = j * points.shape[1] + i
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.color.r = float(color[0])
                marker.color.g = float(color[1])
                marker.color.b = float(color[2])
                marker.color.a = 1.0
                marker.scale.x = 0.01

                p1 = Point()
                p1.x = float(points[j, i, 0])
                p1.y = float(points[j, i, 1])
                p2 = Point()
                p2.x = float(points[j, i+1, 0])
                p2.y = float(points[j, i+1, 1])

                marker.points = [p1, p2]
                marker_array.markers.append(marker)
                
        self.publish_markers(self.waypoints, np.array([1.0, 0.0, 0.0]), "waypoints")
        self.marker_pub_.publish(marker_array)

    def publish_markers(self, points: np.ndarray, color: np.ndarray = np.array([1.0, 0.0, 0.0]), ns: str = ""):
        '''
        Publish MarkerArray message of points
        '''
        if not self.visualize:
            return

        marker_array = MarkerArray()
        for i in range(len(points)):
            marker = Marker()
            marker.ns = ns
            marker.header.frame_id = "map"
            marker.id = i + 1
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(points[i, 0])
            marker.pose.position.y = float(points[i, 1])
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker_array.markers.append(marker)

        self.marker_pub_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    mppi_node = MPPI()
    rclpy.spin(mppi_node)
    mppi_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
