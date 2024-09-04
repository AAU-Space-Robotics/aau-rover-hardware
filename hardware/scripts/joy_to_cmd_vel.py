#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
import numpy as np

class JoyToVelNode(Node):
    def __init__(self):
        super().__init__('joy_to_vel_converter')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy/joy',
            self.listener_callback,
            10)
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        self.get_logger().info("Joy to vel converter node started successfully")

    def listener_callback(self, msg):

        ### Convert the joystick input to linear and angular velocities ###
        
        linear_vel = msg.axes[1] # Left stick up/down
        angular_vel = msg.axes[0] # Left stick left/right

        # Strech the circle to a square
        linear_vel, angular_vel = self.circle_to_square(linear_vel, angular_vel)

        # Create and publish the message
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel

        self.publisher_.publish(twist)

        # If Y button is pressed, stop the robot
        # TODO - Implement a service that stop the robot over can bus
        
        # if msg.buttons[3] == 1:
        #     twist.linear.x = 0
        #     twist.angular.z = 0
        #     self.publisher_.publish(twist)

    def circle_to_square(self, x, y):
        # Ensure the point (x, y) lies within the unit circle
        if x**2 + y**2 > 1:
            x = x / math.sqrt(x**2 + y**2)
            y = y / math.sqrt(x**2 + y**2)

        new_x = x
        new_y = y

        if x**2 > y**2:
            # The point lies in the right half of the circle
            new_x = np.sign(x)* math.sqrt(x**2 + y**2)
        elif y != 0:
            new_x = np.sign(y)* (x / y) * math.sqrt(x**2 + y**2)

        if y**2 > x**2:
            # The point lies in the upper half of the circle
            new_y = np.sign(y)* math.sqrt(x**2 + y**2)
        elif x != 0:
            new_y = np.sign(x)* (y / x) * math.sqrt(x**2 + y**2)

        return new_x*4, new_y*4

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    joy_to_vel_converter = JoyToVelNode()
    # Spin the node so the callback function is called.
    rclpy.spin(joy_to_vel_converter)
    # Destroy the node and shutdown the ROS client
    joy_to_vel_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
