#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np

from typing import List

from nav_msgs.msg import Odometry, Path 
from safe_path_msg.msg import SafePath
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

from f1tenth_mppi.utils import *


def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1,:] + (t*diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment


def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
    """
    starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

    Assumes that the first segment passes within a single radius of the point

    http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
    """
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    trajectory = np.ascontiguousarray(trajectory)
    for i in range(start_i, trajectory.shape[0]-1):
        start = trajectory[i,:]
        end = trajectory[i+1,:]+1e-6
        V = np.ascontiguousarray(end - start)

        a = np.dot(V,V)
        b = 2.0*np.dot(V, start - point)
        c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
        discriminant = b*b-4*a*c

        if discriminant < 0:
            continue
        #   print "NO INTERSECTION"
        # else:
        # if discriminant >= 0.0:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0*a)
        t2 = (-b + discriminant) / (2.0*a)
        if i == start_i:
            if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif t1 >= 0.0 and t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif t2 >= 0.0 and t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break
    # wrap around to the beginning of the trajectory if no intersection is found1
    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0],:]
            end = trajectory[(i+1) % trajectory.shape[0],:]+1e-6
            V = end - start

            a = np.dot(V,V)
            b = 2.0*np.dot(V, start - point)
            c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
            discriminant = b*b-4*a*c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0*a)
            t2 = (-b + discriminant) / (2.0*a)
            if t1 >= 0.0 and t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif t2 >= 0.0 and t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t


def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
    """
    Returns actuation
    """
    waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2]-position)
    speed = lookahead_point[2]
    if np.abs(waypoint_y) < 1e-6:
        return speed, 0.
    radius = 1/(2.0*waypoint_y/lookahead_distance**2)
    steering_angle = np.arctan(wheelbase/radius)
    return speed, steering_angle



class NavigationControl(Node):
    def __init__(self):
        super().__init__('navigation_control')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/pf/pose/odom')
        self.declare_parameter('trajectory_topic', '/safe_trajectory')
        self.declare_parameter('waypoint_path', 'levine_1.csv')
        self.declare_parameter('hz', 10)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('v_gain', 0.75)
        self.declare_parameter('lookahead_distance', 0.82461887897713965)
        self.declare_parameter('max_reacquire', 20)

        self.declare_parameter("min_speed", 1.5)
        self.declare_parameter("max_speed", 3.0)

        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.trajectory_topic = self.get_parameter('trajectory_topic').value 
        self.wheelbase = self.get_parameter('wheelbase').value
        self.hz = self.get_parameter('hz').value 

        self.waypoint_path = self.get_parameter('waypoint_path').value

        self.wpt_xind = 0
        self.wpt_yind = 1
        self.wpt_vind = 3

        try:
            self.waypoints = load_waypoints(self.waypoint_path) # [x, y, theta, velocity]
        except Exception as e:
            self.get_logger().error(f"error loading waypoints file: {e}")

        self.max_reacquire = self.get_parameter('max_reacquire').value
        self.vgain = self.get_parameter('v_gain').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.trajectory_sub = self.create_subscription(
            SafePath,
            self.trajectory_topic,
            self.trajectory_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.marker_pub_ = self.create_publisher(
                MarkerArray,
                "/lookahead_point",
                10
            )

        self.current_odom = None 
        self.current_safe_trj = None 

        self.publish_timer = self.create_timer(1/self.hz, self.publish_callback)
        self.get_logger().info("Navigation Control node initialized")


    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        """
        gets the current waypoint to follow
        """
        wpts = np.vstack((waypoints[:, self.wpt_xind], waypoints[:, self.wpt_yind])).T
        # print(wpts)
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty((3, ))
            # x, y
            current_waypoint[0:2] = wpts[i2, :]
            # speed
            current_waypoint[2] = waypoints[i, self.wpt_vind]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return np.append(wpts[i, :], waypoints[i, self.wpt_vind])
        else:
            return None
        
    def odom_callback(self, msg: Odometry):
        if isinstance(msg, Odometry):
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

            self.current_odom = {
                'pose_x': msg.pose.pose.position.x,
                'pose_y': msg.pose.pose.position.y,
                'pose_theta': np.arctan2(siny_cosp, cosy_cosp),
                'linear_vels_x': msg.twist.twist.linear.x,
                'linear_vels_y': msg.twist.twist.linear.y,
                'ang_vels_z': msg.twist.twist.angular.z
            }

    def trajectory_callback(self, msg: SafePath):
        if isinstance(msg, SafePath):
            self.current_safe_trj = msg 


    def publish_callback(self):
        if not self.current_odom:
            return 

        current_pose = np.array([self.current_odom.get('pose_x'), self.current_odom.get('pose_y')])
        pose_theta = self.current_odom.get('pose_theta')
        flag = False
        waypoints = []
        if self.current_safe_trj and self.current_safe_trj.flag is True:
            flag = self.current_safe_trj.flag
            poses: List[PoseStamped] = self.current_safe_trj.path.poses
            for pose in poses:
                wpt_row = np.array([pose.pose.position.x, pose.pose.position.y, 0.0, self.get_parameter('max_speed').value])
                waypoints.append(wpt_row)
            waypoints = np.array(waypoints)   
        else:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0

            self.drive_pub.publish(msg)
            return 
            # waypoints = self.waypoints


        lookahead_point = self._get_current_waypoint(
            waypoints, 
            self.lookahead_distance, 
            current_pose,
            pose_theta
        )
        # self.get_logger().info(f"{lookahead_point}")
        

        if lookahead_point is None:
            speed, steer = 0.0, 0.0
        else:
            speed, steer = get_actuation(
                pose_theta,
                lookahead_point,
                current_pose,
                self.lookahead_distance,
                self.wheelbase
            )
            self.publish_markers(lookahead_point)
            

        min_speed = self.get_parameter("min_speed").value
        max_speed = self.get_parameter("max_speed").value

        if np.fabs(steer) > np.pi/8:
            speed = min_speed
        else:
            speed = float(-(3/np.pi)*(max_speed - min_speed)*np.fabs(steer)+max_speed)

        self.get_logger().info("Speed: {0:.2f}, Steer: {1:.4f}, use `{2}`".format(
                speed, steer, 
                "MPPI" if flag else "Waypoint" 
            )
        )
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer

        self.drive_pub.publish(msg)

    def publish_markers(self, points: np.ndarray, color: np.ndarray = np.array([0.0, 1.0, 0.0]), ns: str = ""):
        '''
        Publish MarkerArray message of points

        Args:
            points (ndarray): Nx2 array of points to publish
            color (ndarray): The color of the points
        '''

        # Generate MarkerArray message
        marker_array = MarkerArray()
        points = list(points)
        for i in range(len(points)):
            marker = Marker()
            marker.ns = ns
            marker.header.frame_id = "map"
            marker.id = i + 1
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = points[0]
            marker.pose.position.y = points[1]
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

    def stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        self.drive_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = NavigationControl()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.stop()
        print("Shutting down navigation node.")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
