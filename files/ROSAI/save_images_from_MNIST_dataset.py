# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT


import cv2
import numpy as np
from ctypes import *
from typing import List
import os
import sys
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')


import mnist
import matplotlib.pyplot as plt


height = 28    #Adjast based on MNIST data set
width = 28     #Adjast based on MNIST data set
channels = 1   #Assuming grayscale image




def save_image():
    raw_data = mnist.test_images()
    normalized_data = np.asarray(raw_data/255, dtype=np.float32)
    test_data = np.expand_dims(normalized_data, axis=3)
    num_pics  = 100

    output_folder = "/home/ubuntu/images/rosai_file_demo_test_images"
    # output_folder = "/home/srahimip/mnist_images"
    if not os.path.exists(output_folder):
            os.makedirs(output_folder)

    for i in range(num_pics):
            image_to_save = (255 * test_data[i]).astype(np.uint8)
            filename = f"{output_folder}/mnist_image_{i}.jpg"
            success = cv2.imwrite(filename, image_to_save)


            # plt.imshow(test_data[i], 'gray')
            # plt.axis('off')
            # plt.show()


def main(args=None):

    save_image()



if __name__ == '__main__':
    main()
