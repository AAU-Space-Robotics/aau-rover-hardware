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

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Your Ackermann function here, assume it returns steering_angles and wheel_velocities
        steering_angles, wheel_velocities = self.ackermann_steering(linear_vel, angular_vel)

        # Create and publish the message
        motor_commands = Float64MultiArray()
        motor_commands.data = steering_angles + wheel_velocities
        self.publisher_.publish(motor_commands)

    def ackermann_steering(self, lin_vel, ang_vel, L=0.849, d_lr=0.894, d_fr=0.77):
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

        # Different turning radii for the different wheels
        R_ML = R - d_lr / 2
        R_MR = R + d_lr / 2
        R_FR = R + d_fr / 2
        R_FL = R - d_fr / 2
        R_RR = R + d_fr / 2
        R_RL = R - d_fr / 2

        # Steering angles
        theta_FL = math.atan2(L, R_FL)
        theta_FR = math.atan2(L, R_FR)
        theta_ML = 0  # middle wheels don't steer
        theta_MR = 0  # middle wheels don't steer
        theta_RL = math.atan2(L, R_RL)
        theta_RR = math.atan2(L, R_RR)

        # Array of steering angles, adjusted for direction and turning direction
        steering_angles = np.array([theta_FL, theta_FR, theta_ML, theta_MR, -theta_RL, -theta_RR]) * direction * turn_direction

        # Wheel velocities
        V_FL = ang_vel * R_FL
        V_FR = ang_vel * R_FR
        V_ML = ang_vel * R_ML
        V_MR = ang_vel * R_MR
        V_RL = ang_vel * R_RL
        V_RR = ang_vel * R_RR

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