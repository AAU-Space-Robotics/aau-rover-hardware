#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs


class RealSenseTracking(Node):
    """ 
    This node subscribes to the RealSense T265 tracking camera and publishes the pose of the camera.
    """

    def __init__(self):
        """ 
        Initialize the node. 
        """
        super().__init__("T265")
        ## Specify Parameters
        self.declare_parameter("hz", 30)
        self.declare_parameter("serial_number", "224622111375")

        ## Initialize Variables
        hz = self.get_parameter("hz").value
        serial_number = self.get_parameter("serial_number").value
        timer_period = 1.0 / hz

        ## Initialize Publishers
        self.publisher = self.create_publisher(PoseStamped, "pose", 10)
        self.timer = self.create_timer(timer_period, self.callback)

        ## Declare and start the RealSense pipeline for T265.
        self.pipe = rs.pipeline()
        self.config = rs.config()
        if serial_number:
            self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.pose)

        ## Start the pipeline.
        self.pipe.start(self.config)


    def callback(self):
        """
        Publishes the pose of the camera.
        """
        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame()

        if pose:
            pose_data = pose.get_pose_data()
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_pose_frame"
            msg.pose.position.x = pose_data.translation.x
            msg.pose.position.y = pose_data.translation.y
            msg.pose.position.z = pose_data.translation.z
            msg.pose.orientation.x = pose_data.rotation.x
            msg.pose.orientation.y = pose_data.rotation.y
            msg.pose.orientation.z = pose_data.rotation.z
            msg.pose.orientation.w = pose_data.rotation.w
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseTracking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()