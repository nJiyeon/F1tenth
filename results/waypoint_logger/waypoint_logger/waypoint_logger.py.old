import csv 
import time
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WaypointLogger(Node):

    def __init__(self):
        super().__init__('waypoint_logger')

        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.delimiter = self.declare_parameter('delimiter', ';').value

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.cache_odom = []
        self.waypoints = []

    def listener_callback(self, msg: Odometry):

        s_m = 0.0
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        psi_rad = 0.0
        kappa_radpm = 0.0
        vx_mps = msg.twist.twist.linear.x
        ax_mps = 0.0
        
        odom_value = [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps]
        
        if self.cache_odom != odom_value:
            self.cache_odom = odom_value
            self.waypoints.append(odom_value)
            self.get_logger().info(f"{self.cache_odom}")
        

    def save_csv(self):
        self.get_logger().info("Saving waypoint...")
        path = os.path.abspath(f'wp_{time.asctime()}.csv')
        f = open(path, 'w', newline='')
        wr = csv.writer(f, delimiter=self.delimiter)
        wr.writerow(['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps'])

        for row in self.waypoints:
            wr.writerow(row)
        
        f.close()
        self.get_logger().info(f"Saved in {path}")


def main(args=None):
    rclpy.init(args=args)

    app = WaypointLogger()
    try:
        rclpy.spin(app)
    except KeyboardInterrupt:
        app.save_csv()
    finally:
        app.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
