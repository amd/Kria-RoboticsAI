# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisherNode(Node):
    def __init__(self, image_folder):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer_ = self.create_timer(1.0 / 30, self.publish_frames)  # 30 FPS
        self.cv_bridge = CvBridge()
        self.image_folder = image_folder

    def publish_frames(self):
        image_files = [f for f in os.listdir(self.image_folder) if f.endswith('.jpg')]

        for image_file in image_files:
            image_path = os.path.join(self.image_folder, image_file)
            frame = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

            if frame is not None:
                image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='mono8')
                self.publisher_.publish(image_msg)
            else:
                self.get_logger().warn(f'Failed to read frame from {image_file}.')

def main(args=None):
    rclpy.init(args=args)
    image_folder = '/home/ubuntu/images/rosai_file_demo_test_images'
    camera_publisher = CameraPublisherNode(image_folder)
    rclpy.spin(camera_publisher)

if __name__ == '__main__':
    main()
