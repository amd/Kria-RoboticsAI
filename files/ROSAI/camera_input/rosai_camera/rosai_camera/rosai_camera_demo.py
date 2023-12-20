# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
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

import sys
sys.path.append('/usr/lib/python3.10/site-packages')
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
# the above path is needed by pynq_dpu
from pynq_dpu import DpuOverlay

import mnist
import matplotlib.pyplot as plt

batch_size = 1 #since we have one image at a time
height = 28    #Adjast based on MNIST data set
width = 28     #Adjast based on MNIST data set
channels = 1   #Assuming grayscale image

class RosaiCameraDemo(Node):

    def __init__(self):
        super().__init__('rosai_camera_demo')
        self.subscriber_ = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.get_logger().info('[INFO] __init__, Create Subscription ...')
        self.subscriber_  # prevent unused variable warning
        self.publisher1 = self.create_publisher(Image, 'vision/asl', 10)
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

        # Overlay the DPU and Vitis-AI .xmodel file
        self.overlay = DpuOverlay("dpu.bit")
        self.model_path = '/home/root/jupyter_notebooks/pynq-dpu/dpu_mnist_classifier.xmodel'
        self.get_logger().info("MODEL="+self.model_path)
        self.overlay.load_model(self.model_path)

        # Create DPU runner
        self.dpu = self.overlay.runner

        self.get_logger().info('[INFO] __init__ exiting...')

    def calculate_softmax(self, data):
        result = np.exp(data)
        return result

    def listener_callback(self, msg):
        self.get_logger().info("Starting of listener callback...")
        bridge = CvBridge()
        cv2_image_org = bridge.imgmsg_to_cv2(msg,desired_encoding="rgb8")
        y1 = (128)
        y2 = (128+280)
        x1 = (208)
        x2 = (208+280)
        roi_img = cv2_image_org[ y1:y2, x1:x2, : ]
        resized_image = cv2.resize(roi_img, (28, 28), interpolation=cv2.INTER_LINEAR)
        roi_img_gray=cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        cv2_image_normal = np.asarray(roi_img_gray/255, dtype=np.float32)
        cv2_image = np.expand_dims(cv2_image_normal, axis=2)

        self.get_logger().info("type of cv2_image"+str(type(cv2_image)))
        self.get_logger().info("Shape of cv2_image"+str(cv2_image.shape))
        #########################################

        inputTensors = self.dpu.get_input_tensors()
        outputTensors = self.dpu.get_output_tensors()
        shapeIn = tuple(inputTensors[0].dims)
        shapeOut = tuple(outputTensors[0].dims)
        outputSize = int(outputTensors[0].get_data_size() / shapeIn[0])

        softmax = np.empty(outputSize)
        output_data = [np.empty(shapeOut, dtype=np.float32, order="C")]
        input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
        image = input_data[0]
        image[0,...] = cv2_image

        prediction = 0
        self.get_logger().info("prediction="+str(prediction))

        job_id = self.dpu.execute_async(input_data, output_data)
        self.dpu.wait(job_id)
        temp = [j.reshape(1, outputSize) for j in output_data]
        softmax = self.calculate_softmax(temp[0][0])
        prediction = softmax.argmax()

        self.get_logger().info("REAL prediction="+str(prediction))

        # DISPLAY
        cv2_bgr_image = cv2.cvtColor(cv2_image_org, cv2.COLOR_RGB2BGR)
        cv2.imshow('rosai_demo',cv2_bgr_image)
        cv2.waitKey(1)

        # CONVERT BACK TO ROS & PUBLISH
        image_ros = bridge.cv2_to_imgmsg(cv2_image)
        self.publisher1.publish(image_ros)
        self.get_logger().info("REAL prediction="+str(prediction))

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
        cv2.putText(cv2_image_org,"prediction: "+str(prediction)+self.actionDetected,
            (208,108-10),self.text_fontType,self.text_fontSize,
            (0,255,0),self.text_lineSize,self.text_lineType)
        cv2.rectangle(cv2_image_org, (x1,y1), (x2,y2), (0, 255, 0), 2)

        # DISPLAY
        cv2.imshow('rosai_demo',cv2_image_org)
        cv2.waitKey(1)

        # CONVERT BACK TO ROS & PUBLISH
        image_ros = bridge.cv2_to_imgmsg(cv2_image_org)
        self.publisher1.publish(image_ros)


def main(args=None):
    rclpy.init(args=args)
    rosai_camera_demo = RosaiCameraDemo()
    rclpy.spin(rosai_camera_demo)

if __name__ == '__main__':
    main()
