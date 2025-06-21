#!/home/cirl/ros2_rl_env/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from pyproj import Proj
import math
import time


class GPSToGazeboNode(Node):
    def __init__(self):
        super().__init__('gps_to_gazebo_node')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fws_robot/gps/fix',
            self.gps_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, '/fws_robot/odometry', 10)

        self.origin_lat = 37.4275
        self.origin_lon = -122.1697
        self.origin_alt = 0.0

        self.proj = Proj(proj='utm', zone=10, ellps='WGS84')
        self.origin_x, self.origin_y = self.proj(self.origin_lon, self.origin_lat)

        self.get_logger().info("GPS to Gazebo node started.")

    def gps_callback(self, msg: NavSatFix):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return

        x, y = self.proj(msg.longitude, msg.latitude)
        gazebo_x = x - self.origin_x
        gazebo_y = y - self.origin_y
        gazebo_z = msg.altitude - self.origin_alt

        # self.get_logger().info(f'Gazebo position: x={gazebo_x:.2f}, y={gazebo_y:.2f}, z={gazebo_z:.2f}')

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position = Point(x=gazebo_x, y=gazebo_y, z=gazebo_z)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        odom_msg.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSToGazeboNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
