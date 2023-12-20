#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# Author: Daniele Bagni, AMD/Xilinx Inc
# date:   08 Nov 2023

print(" ")
print("ResNet18 CNN (pre-trained with ImageNet Dataset) fine-tuned to CIFAR10 Dataset, in TensorFlow2")
print(" ")

# ***********************************************************************
# Import Packages
# ***********************************************************************
import os
import time
import numpy as np
import cv2
from matplotlib import pyplot as plt

# ***********************************************************************
# input file names
# ***********************************************************************
cnn_xmodel  = os.path.join("./compiled", "kr260_cifar10_tf2_resnet18.xmodel")
labels_file = os.path.join("./"        , "cifar10_labels.dat")
images_dir  = os.path.join("./"        , "test_images")

# ***********************************************************************
# Utility Functions
# ***********************************************************************
def plt_imshow(title, image):
    image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.figure()
    plt.imshow(image)
    plt.title(title)
    plt.grid(False)
    plt.savefig(title)
    plt.show()

def predict_label(softmax):
    with open(labels_file, "r") as f:
        lines = f.readlines()
    return lines[np.argmax(softmax)]

# see https://stackoverflow.com/questions/34968722/how-to-implement-the-softmax-function-in-python
def calculate_softmax(data):
    result = np.exp(data)
    return result

# ***********************************************************************
# Pre-Processing Functions for CiFAR10
# ***********************************************************************
'''
_R_MEAN = 0.0 # 123.68
_G_MEAN = 0.0 # 116.78
_B_MEAN = 0.0 # 103.94
MEANS = [_B_MEAN,_G_MEAN,_R_MEAN]

def resize_shortest_edge(image, size):
    H, W = image.shape[:2]
    if H >= W:
        nW = size
        nH = int(float(H)/W * size)
    else:
        nH = size
        nW = int(float(W)/H * size)
    return cv2.resize(image,(nW,nH))

def mean_image_subtraction(image, means):
    B, G, R = cv2.split(image)
    B = B - means[0]
    G = G - means[1]
    R = R - means[2]
    image = cv2.merge([R, G, B])
    return image

def BGR2RGB(image):
    B, G, R = cv2.split(image)
    image = cv2.merge([R, G, B])
    return image

def central_crop(image, crop_height, crop_width):
    image_height = image.shape[0]
    image_width = image.shape[1]
    offset_height = (image_height - crop_height) // 2
    offset_width = (image_width - crop_width) // 2
    return image[offset_height: offset_height + crop_height,
                 offset_width :  offset_width + crop_width , :]

def normalize(image):
    image=image/255.0
    image=image-0.5
    image=image*2
    return image
'''

def Normalize(image):
    x_test  = np.asarray(image)
    x_test = x_test.astype(np.float32)
    x_test = x_test/255.0
    x_test = x_test -0.5
    out_x_test = x_test *2
    return out_x_test

def preprocess_fn(image_filename):
    image=cv2.imread(image_filename)
    #image = resize_shortest_edge(image, 256)
    #image = mean_image_subtraction(image, MEANS)
    #image = central_crop(image, crop_height, crop_width)
    image = np.asarray(image)
    image2 = Normalize(image) #added by me for ResNet18
    return image2

# ***********************************************************************
# CIFAR10 Labels
# ***********************************************************************
labelNames = { "airplane" : 0, "automobile" : 1, "bird" : 2, "cat" : 3, "deer" : 4, "dog" : 5,
                "frog" : 6, "horse" : 7, "ship" : 8, "truck" : 9}

# ***********************************************************************
# Prepare the Overlay and load the "cnn.xmodel"
# ***********************************************************************
from pynq_dpu import DpuOverlay
overlay = DpuOverlay("dpu.bit")
overlay.load_model(cnn_xmodel)

# ***********************************************************************
# Use VART APIs
# ***********************************************************************
original_images = [i for i in os.listdir(images_dir) if i.endswith("png")]
total_images = len(original_images)

dpu = overlay.runner
inputTensors = dpu.get_input_tensors()
outputTensors = dpu.get_output_tensors()
shapeIn = tuple(inputTensors[0].dims)
shapeOut = tuple(outputTensors[0].dims)
outputSize = int(outputTensors[0].get_data_size() / shapeIn[0])
print("shapeIn   : {}".format(shapeIn))
print("shapeOut  : {}".format(shapeOut))
print("outputSize: {}".format(outputSize))

# allocate some buffers that will be re-used multiple times
predictions = np.empty(total_images)
test_labels = np.empty(total_images)
softmax = np.empty(outputSize)
output_data = [np.empty(shapeOut, dtype=np.float32, order="C")]
input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
image = input_data[0]

# ***********************************************************************
# Run DPU to Make Predictions on ALL the images
# ***********************************************************************
print("Classifying {} CIFAR10 pictures ...".format(total_images))
time1 = time.time()
for image_index in range(total_images):
    filename = os.path.join(images_dir, original_images[image_index])
    preprocessed = preprocess_fn(filename)
    image[0,...] = preprocessed.reshape(shapeIn[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)
    temp = [j.reshape(1, outputSize) for j in output_data]
    softmax = calculate_softmax(temp[0][0])
    predictions[image_index] = softmax.argmax()
time2 = time.time()
execution_time = time2-time1
print("  Execution time: {:.4f}s".format(execution_time))
print("      Throughput: {:.4f}FPS".format(total_images/execution_time))

# ***********************************************************************
# Check Prediction Accuracy
# ***********************************************************************
# filename example: /home/root/jupyter_notebooks/pynq-dpu/cifar10_tf2_target_zcu102/cifar10/test/truck_9938.png

top1_true  = 0
top1_false = 0
for i in range(total_images):
    filename = os.path.join(images_dir, original_images[i])
    short_filename=filename.split("test_images/")[1]     #truck_9938.png
    class_name=short_filename.split("_")[0]    #truck
    #print("short filename: {}".format(short_filename))
    #print("class name    : {}".format(class_name))
    test_labels[i] = labelNames[class_name]
    if predictions[i] == test_labels[i] :
        top1_true += 1
    else:
        top1_false +=1

correct = np.sum(predictions==test_labels)
print("Overall accuracy: {:.4f}".format(correct/total_images))

assert (top1_true+top1_false)  == total_images, "ERROR: top1 true+false not equal to the number of images"
top1_accuracy = float(top1_true)/(top1_true+top1_false)
print("number of top1 false predictions ", top1_false)
print("number of top1 right predictions ", top1_true)
print("top1 accuracy = %.4f" % top1_accuracy)


# ***********************************************************************
# Run DPU to Make Predictions on FEW images
# ***********************************************************************
def run(image_index, display=False):
    filename = os.path.join(images_dir, original_images[image_index])
    preprocessed = preprocess_fn(cv2.imread(filename))
    image[0,...] = preprocessed.reshape(shapeIn[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)
    temp = [j.reshape(1, outputSize) for j in output_data]
    softmax = calculate_softmax(temp[0][0])
    prediction = softmax.argmax()
    short_filename=filename.split("test/")[1]
    class_name=short_filename.split("_")[0]
    print("\ninput image ", short_filename)
    print("Classification method1: ", predict_label(softmax), " ", prediction)
    print("Classification method2: ", class_name            , " ", predictions[image_index])
    if display:
        display_image = cv2.imread(filename)
        #_, ax = plt.subplots(1)
        #_ = ax.imshow(cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB))
        cv2.imshow(short_filename)
        cv2.waitKey(0)
        #plt_imshow(short_filename)

#run(  1, display=True)
#run(101, display=True)
#[run(i) for i in range(10)]

# ***********************************************************************
# Clean up
# ***********************************************************************
del overlay
del dpu
