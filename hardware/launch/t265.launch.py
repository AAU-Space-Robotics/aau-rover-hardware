from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    
    tracking_node = Node(
            package='hardware',
            executable='T265.py',
            name='T265',
            parameters=[
                {"hz": 30},
                {"serial_number": "224622111375"}
            ]
        )


    ld.add_action(tracking_node)
    
    return ld 