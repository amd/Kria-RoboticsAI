#!/bin/sh

# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# prepare the entire CIFAR10 dataset images
python3 cifar10_generate_images.py

# remove the unnecessary folders
rm -r ./build
