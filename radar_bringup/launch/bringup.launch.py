import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory("radar_bringup"), "launch"))


def generate_launch_description():

    from launch_ros.actions import Node
    from launch import LaunchDescription

    launch_params = yaml.safe_load(
        open(
            os.path.join(
                get_package_share_directory("radar_bringup"),
                "config",
                "launch_params.yaml",
            )
        )
    )

    detector_params = os.path.join(
        get_package_share_directory("radar_bringup"), "config", "detector_params.yaml"
    )

    camera_params = os.path.join(
        get_package_share_directory("radar_bringup"),
        "config",
        "camera_node_params.yaml",
    )
    video_params = os.path.join(
        get_package_share_directory("radar_bringup"),
        "config",
        "video_node_params.yaml",
    )

    # 图像
    if launch_params['video_play']: 
        image_node  = Node(
            package="hik_camera",
            executable="video_player_node",
            parameters=[video_params],
            output="screen",
        )
    else:
        image_node = Node(
            package="hik_camera",
            executable="hik_camera_node",
            parameters=[camera_params],
            output="screen",
        )
   
    detector_node = Node(
        package="radar_detector",
        executable="radar_detector_node",
        parameters=[detector_params],
        output="screen",
    )


    calibrator_node = Node(
        package="radar_calibrator",
        executable="radar_calibrator_node",
        output="screen",
    )
    
    serial_node = Node(
        package="radar_referee",
        executable="radar_referee",
        output="screen",
    )
    
    return LaunchDescription(
        [
            detector_node,
            image_node,
            calibrator_node,
            # serial_node
        ]
    )
