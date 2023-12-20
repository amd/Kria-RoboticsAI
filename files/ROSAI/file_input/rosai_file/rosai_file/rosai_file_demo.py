# Copyright 2023 Avnet, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.

import sys
sys.path.append('/usr/lib/python3.10/site-packages')
# the above path is needed by xir and vart
import xir
import vart
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
# the above path is needed by pynq_dpu
from pynq_dpu import DpuOverlay

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
# import CV BRIDGE
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ctypes import *
from typing import List
import pathlib
import time
import argparse
import glob
import subprocess
import re
import os

import mnist
import matplotlib.pyplot as plt

batch_size = 1 #since we have one image at a time
height = 28    #Adjast based on MNIST data set
width = 28     #Adjast based on MNIST data set
channels = 1   #Assuming grayscale image

class RosaiFileDemo(Node):

    def __init__(self):
        super().__init__('rosai_file_demo')
        self.subscriber_ = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscriber_  # prevent unused variable warning
        self.publisher2 = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Parameters (for text overlay)
        self.scale = 1.0
        self.text_fontType = cv2.FONT_HERSHEY_SIMPLEX
        self.text_fontSize = 0.75*self.scale
        self.text_color    = (255,0,0)
        self.text_lineSize = max( 1, int(2*self.scale) )
        self.text_lineType = cv2.LINE_AA
        self.text_x = int(10*self.scale)
        self.text_y = int(30*self.scale)

        # Determine DPU architecture and Vitis-AI .xmodel file
        self.overlay = DpuOverlay("dpu.bit")
        self.model_path = '/home/root/jupyter_notebooks/pynq-dpu/dpu_mnist_classifier.xmodel'
        self.get_logger().info("MODEL="+self.model_path)
        self.overlay.load_model(self.model_path)

        # Create DPU runner
        self.dpu = self.overlay.runner

    def calculate_softmax(self, data):
        result = np.exp(data)
        return result

    def listener_callback(self, msg):
        self.get_logger().info("Starting of listener callback")
        bridge = CvBridge()
        cv2_image_org = bridge.imgmsg_to_cv2(msg)
        self.get_logger().info("converted message to CV2 image")
        cv2_image_normal = np.asarray(cv2_image_org/255, dtype=np.float32)
        self.get_logger().info("normalized the image")
        cv2_image = np.expand_dims(cv2_image_normal, axis=2)
        self.get_logger().info("expanded the normalized image data")
        raw_data = mnist.test_images()
        normalized_data = np.asarray(raw_data/255, dtype=np.float32)
        test_data = np.expand_dims(normalized_data, axis=3)
        test_label = mnist.test_labels()
        inputTensors = self.dpu.get_input_tensors()
        outputTensors = self.dpu.get_output_tensors()
        shapeIn = tuple(inputTensors[0].dims)
        shapeOut = tuple(outputTensors[0].dims)
        outputSize = int(outputTensors[0].get_data_size() / shapeIn[0])

        softmax = np.empty(outputSize)
        output_data = [np.empty(shapeOut, dtype=np.float32, order="C")]
        input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
        image = input_data[0]
        num_pics  = 1
        prediction = 0


        for i in range(num_pics):
                image[0,...] = cv2_image
                are_equal = np.array_equal(test_data[i], cv2_image)
                self.get_logger().info("starting DPU execution..")
                job_id = self.dpu.execute_async(input_data, output_data)
                self.dpu.wait(job_id)
                self.get_logger().info("DPU execution done...")
                temp = [j.reshape(1, outputSize) for j in output_data]
                softmax = self.calculate_softmax(temp[0][0])
                prediction = softmax.argmax()
                self.get_logger().info("predicted digit="+str(prediction))



        ## Display the frame
        frame_bgr = cv2.cvtColor(cv2_image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Video', frame_bgr)

        # Set the window size (you can adjust the width and height)
        cv2.waitKey(1)


        ####################################################
        # Advance
        self.actionDetected = ""
        if prediction == 1:
              self.actionDetected = "A : Advance"
              # Create message to backup (+ve value on x axis)
              msg = Twist()
              msg.linear.x = 2.0
              msg.linear.y = 0.0
              msg.linear.z = 0.0
              msg.angular.x = 0.0
              msg.angular.y = 0.0
              msg.angular.z = 0.0
              self.publisher2.publish(msg)
        # Back
        if prediction == 2:
          self.actionDetected = "B : Back-Up"
          # Create message to backup (-ve value on x axis)
          msg = Twist()
          msg.linear.x = -2.0
          msg.linear.y = 0.0
          msg.linear.z = 0.0
          msg.angular.x = 0.0
          msg.angular.y = 0.0
          msg.angular.z = 0.0
          self.publisher2.publish(msg)
        # Left turn
        if prediction == 3:
          self.actionDetected = "L : Turn Left"
          # Create message to turn left (+ve value on z axis)
          msg = Twist()
          msg.linear.x = 2.0
          msg.linear.y = 0.0
          msg.linear.z = 0.0
          msg.angular.x = 0.0
          msg.angular.y = 0.0
          msg.angular.z = 2.0
          self.publisher2.publish(msg)
        # Right turn
        if prediction == 4:
          self.actionDetected = "R : Turn Right"
          # Create message to turn in right (-ve value on z axis)
          msg = Twist()
          msg.linear.x = 2.0
          msg.linear.y = 0.0
          msg.linear.z = 0.0
          msg.angular.x = 0.0
          msg.angular.y = 0.0
          msg.angular.z = -2.0
          self.publisher2.publish(msg)
        if prediction == 0:
          self.actionDetected = "{del} : Reset Turtle"
          # Create message to reset turtlesim
          try:
              self.cli = self.create_client(Empty, 'reset')
              while not self.cli.wait_for_service(timeout_sec=1.0):
                 self.get_logger().info('[ERROR] service not available, throwing exception ...')
                 raise Exception('[ERROR] service not available')
              self.req = Empty.Request()
              self.future = self.cli.call_async(self.req)
          except:
              self.get_logger().info("Failed to reset turtlesim!")
              self.cli = None


        self.get_logger().info(self.actionDetected)


def main(args=None):
    rclpy.init(args=args)

    rosai_file_demo = RosaiFileDemo()

    rclpy.spin(rosai_file_demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rosai_file_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
