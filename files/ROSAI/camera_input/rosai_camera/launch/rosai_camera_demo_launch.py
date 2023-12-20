# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosai_camera',
            executable='usbcam_publisher',
        ),
        Node(
            package='rosai_camera',
            executable='rosai_camera_demo',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
        )
    ])
