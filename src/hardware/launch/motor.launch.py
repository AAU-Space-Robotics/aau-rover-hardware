from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    
    motor_node = Node(
            package='hardware',
            executable='motors.py',
            name='motors'
        )

    ackermann_node = Node(
            package='hardware',
            executable='ackermann.py',
            name='ackermann'
        )
    
    
    ld.add_action(ackermann_node)
    ld.add_action(motor_node)
    
    return ld 
