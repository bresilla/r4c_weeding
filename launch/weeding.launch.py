import os
from launch import LaunchDescription
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    detection_blob = Node(
        package='r4c_weeding',
        executable='detection_blob',
        name='detection_blob',
        # parameters=[params, global_params]
    )

    detection_track = Node(
        package='r4c_weeding',
        executable='detection_track',
        name='detection_track',
        # parameters=[params, global_params]
    )

    # camera_front_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['1.0', '0.0', '1.2', '0.0', '0.258819', '0.0', '0.965926', 'camera_front', 'base_link']
    # )

    # camera_back_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['-1.0', '0.0', '1.2', '-0.258819', '0.0', '0.965926', '0.0', 'camera_back', 'base_link']
    # )

    ld.add_action(detection_blob)
    ld.add_action(detection_track)
    # ld.add_action(camera_front_transform)
    # ld.add_action(camera_back_transform)

    return ld