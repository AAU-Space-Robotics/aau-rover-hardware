#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import math


class PoseNode(Node):
    def __init__(self):
        super().__init__('pose_node')

        # Setup Publishers
        self.pose_publisher = self.create_publisher(Pose, '/robot_pose', 10)

        self.pose_subscription = self.create_subscription(
            PoseStamped, '/zed2i/tracking/pose', self.pose_callback, 10)

        self.pose_data: np.ndarray = None

        timer_period = 0.2   # seconds (5Hz)
        self.timer = self.create_timer(timer_period, self.pose_callback)

        self.get_logger().info("Inference node started successfully")

    def pose_callback(self, msg: Pose):
        # Temp handling of transform

        def create_transformation_matrix(self, x, y, theta):
            # Define the rotation matrix
            c, s = np.cos(theta), np.sin(theta)
            R = np.array([[c, -s, 0],
                          [s, c, 0],
                          [0, 0, 1]])

            # Define the translation matrix
            T = np.array([[1, 0, x],
                          [0, 1, y],
                          [0, 0, 1]])

            # Combine into full transformation matrix
            return np.dot(T, R)

        def quaternion_to_euler(self, x, y, z, w):
            # Convert a quaternion into euler angles (yaw, pitch, roll)
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            return yaw_z, pitch_y, roll_x

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        ox = msg.pose.orientation.x
        oy = msg.pose.orientation.y
        oz = msg.pose.orientation.z
        ow = msg.pose.orientation.w

        angles = quaternion_to_euler(ox, oy, oz, ow)
        T = create_transformation_matrix(px, py, angles[2])
        point_in_camera_coords = np.array([[0.3], [0.0], [1.0]])
        transformed_point = np.dot(T, point_in_camera_coords)

        self.pose_data = np.array(
            [transformed_point[0], transformed_point[1], transformed_point[2], ow, ox, oy, oz])

    def pose_callback(self):

        if self.pose_data is None:
            self.get_logger().warn("Pose data is not ready")
            return None

        # Publish Pose
        pose = Pose()
        pose.position.x = self.pose_data[0]
        pose.position.y = self.pose_data[1]
        pose.position.z = self.pose_data[2]

        pose.orientation.x = self.pose_data[4]
        pose.orientation.y = self.pose_data[5]
        pose.orientation.z = self.pose_data[6]
        pose.orientation.w = self.pose_data[3]

        self.pose_publisher.publish(pose)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    joy_to_vel_converter = PoseNode()
    # Spin the node so the callback function is called.
    rclpy.spin(joy_to_vel_converter)
    # Destroy the node and shutdown the ROS client
    joy_to_vel_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
