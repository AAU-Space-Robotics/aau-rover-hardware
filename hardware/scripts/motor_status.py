#!/usr/bin/env python3
import os
import signal
import Jetson.GPIO as GPIO
import rclpy
import canopen

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Motor IDs
FRONT_LEFT = 1
FRONT_RIGHT = 2
CENTER_LEFT = 3
CENTER_RIGHT = 4
REAR_LEFT = 5
REAR_RIGHT = 6
FRONT_LEFT_ANGLE = 7
FRONT_RIGHT_ANGLE = 8
REAR_LEFT_ANGLE = 9
REAR_RIGHT_ANGLE = 10

MOTOR_IDS = [   FRONT_LEFT, FRONT_RIGHT, CENTER_LEFT,
                CENTER_RIGHT, REAR_LEFT, REAR_RIGHT,
                FRONT_LEFT_ANGLE, FRONT_RIGHT_ANGLE,
                REAR_LEFT_ANGLE, REAR_RIGHT_ANGLE] 

class MotorStatus(Node):

    def __init__(self):
        super().__init__('motor_status')

        self.init_settings()

        self.can_interface = 'can1'
        self.network = canopen.Network()
        self.network.connect(bustype='socketcan', channel=self.can_interface)

        # Publishers include : 'Controlword' and 'Statusword'

        # Controlword publisher
        self.controlword_publisher = self.create_publisher(Float64MultiArray, 'controlword', 10)
        self.controlword_msg = Float64MultiArray()

        # Statusword publisher
        self.statusword_publisher = self.create_publisher(Float64MultiArray, 'statusword', 10)
        self.statusword_msg = Float64MultiArray()

        # Start callback for controlword
        self.controlword_subscriber = self.create_subscription(Float64MultiArray, 'controlword', self.controlword_callback, 10)
        
        self.get_logger().info("Motor status node started successfully")

    def init_settings(self):
        # Motor IDs
        self.motor_ids = MOTOR_IDS
        self.velocity_motor_ids = MOTOR_IDS[:6]
        self.steering_motor_ids = MOTOR_IDS[6:]

        # EDS path
        self.eds_path = 'install/hardware/share/hardware/config/C5-E-2-09.eds'

    def get_controlword(self, motor_id):
        return self.network.sdo[motor_id][0x6040].raw    