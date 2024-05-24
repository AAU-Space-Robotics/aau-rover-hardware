from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    zed_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('zed_wrapper'), '/launch/zed2i.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '37915676',
            'zed_id': '0',
            'node_name': 'top_camera'
        }.items()
    )

    zed_tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('zed_wrapper'), '/launch/zed2i.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '35803121',
            'zed_id': '1',
            'node_name': 'tracking'
        }.items()
    )

    camera_pose = Node(
        package='hardware',
        executable='pose_transform.py',
        name='camera_pose',
    )

    ld.add_action(zed_top)
    ld.add_action(zed_tracking)

    return ld
