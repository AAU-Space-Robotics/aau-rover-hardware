#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class AckermannNode(Node):
    def __init__(self):
        super().__init__('ackermann_node')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/motor_commands',
            10)

        self.get_logger().info("Ackermann node started successfully")

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Your Ackermann function here, assume it returns steering_angles and wheel_velocities
        steering_angles, wheel_velocities = self.ackermann_steering(linear_vel, angular_vel)
        #self.get_logger().info("Steering angles: " + str(steering_angles))

        # Convert from numpy arrays to lists
        steering_angles = [float(angle) for angle in steering_angles]
        wheel_velocities = [float(vel) for vel in wheel_velocities]
        # Create and publish the message
        motor_commands = Float64MultiArray()
        motor_commands.data = wheel_velocities + steering_angles
        self.publisher_.publish(motor_commands)

    def ackermann_steering(self, lin_vel, ang_vel, L=0.849, d_lr=0.894, d_fr=0.77):
        offset=-0.0135
        # Checking the direction of the linear and angular velocities
        direction = -1 if lin_vel < 0 else 1
        turn_direction = -1 if ang_vel < 0 else 1

        # Taking the absolute values
        lin_vel = abs(lin_vel)
        ang_vel = abs(ang_vel)

        # If the angular velocity is 0, then the steering angles are 0
        if ang_vel == 0:
            return [0] * 6, [lin_vel] * 6

        R = lin_vel / ang_vel
        if R < 0.8:
            R = 0.8

        # Different turning radii for the different wheels
        R_ML = R - (d_lr / 2) * turn_direction
        R_MR = R + (d_lr / 2) * turn_direction
        R_FR = R + (d_fr / 2) * turn_direction
        R_FL = R - (d_fr / 2) * turn_direction
        R_RR = R + (d_fr / 2) * turn_direction
        R_RL = R - (d_fr / 2) * turn_direction

        # Steering angles
        theta_FL = math.atan2((L/2)-offset, R_FL)
        theta_FR = math.atan2((L/2)-offset, R_FR)
        theta_ML = 0  # middle wheels don't steer
        theta_MR = 0  # middle wheels don't steer
        theta_RL = math.atan2((L/2)+offset, R_RL)
        theta_RR = math.atan2((L/2)+offset, R_RR)

        # Array of steering angles, adjusted for direction and turning direction
        steering_angles = np.array([theta_FL, theta_FR, -theta_RL, -theta_RR]) * turn_direction

        # Wheel velocities
        V_FL = (ang_vel * R_FL) * direction
        V_FR = (ang_vel * R_FR) * direction
        V_ML = (ang_vel * R_ML) * direction
        V_MR = (ang_vel * R_MR) * direction
        V_RL = (ang_vel * R_RL) * direction
        V_RR = (ang_vel * R_RR) * direction

        # Array of wheel velocities, adjusted for direction
        wheel_velocities = np.array([V_FL, V_FR, V_ML, V_MR, V_RL, V_RR]) * direction

        return steering_angles, wheel_velocities
    
def main(args=None):
    rclpy.init(args=args)

    node = AckermannNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
