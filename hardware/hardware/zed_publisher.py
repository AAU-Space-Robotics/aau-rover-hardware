import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyzed.sl as sl

class ZedPublisher(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)

        timer_period = 0.05
        self.br_rgb = CvBridge()

        # Camera serial numbers
        zed_2_serial    = 37915676
 
        # Create a InitParameters object for ZED2 and set configuration parameters
        init_params = sl.InitParameters()
        init_params.set_from_serial_number(zed_2_serial)
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
        
        #init_params.coordinate_system = sl.COORDINATE_SYSTEM_IMAGE
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        init_params.camera_resolution = sl.RESOLUTION.VGA
        init_params.depth_stabilization = False
        init_params.depth_minimum_distance = 0.15
        init_params.depth_maximum_distance = 5.0

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100


        try:
            self.zed = sl.Camera()
            status = self.zed.open(init_params)

            if status != sl.ERROR_CODE.SUCCESS:
                print("There was an error opening the ZED camera. Is it connected?")
                exit(1)


            self.pipe.start(self.cfg)
            self.timer = self.create_timer(timer_period, self.timer_callback)
        except Exception as e:
            print(e)
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED")

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.get_logger().info("Publishing rgb frame")


def main(args = None):
    rclpy.init(args = None)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)
    intel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()