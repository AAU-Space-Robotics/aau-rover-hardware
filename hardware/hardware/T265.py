#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyrealsense2 as rs


class RealSenseTracking(Node):

    def __init__(self):
        super().__init__("T265")
        pass

    def callback(self):


def main(args=None):
    pass


if __name__ == "__main__":
    main()