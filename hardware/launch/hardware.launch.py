from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    ### Add nodes to launch file
    joy_node = Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='joy_node'
        )
    
    ### Include launch files from other packages e.g. zed_wrapper
    zed_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("zed_wrapper"), "/launch/include/zed_camera.launch.py"]),

        launch_arguments={
            "camera_model": "zed2i"
        }.items()
    )

    joy_node = Node(
        package='joy',
        namespace='joy',
        executable='joy_node',
        name='joy_node'
    )
    
    joy_to_vel_node = Node(
            package='hardware',
            executable='joy_to_cmd_vel.py',
            name='joy_to_vel_converter'
        )

    ld.add_action(joy_node)
    ld.add_action(zed_launch_file)
    ld.add_action(joy_node)
    ld.add_action(joy_to_vel_node)
    return ld 
