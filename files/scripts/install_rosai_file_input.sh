#!/bin/bash
#
# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# ***********************************************************************************
# 1. You have already copied the folder "rosai_file_input" from this repository
# to the destination folder ``/home/ubuntu`` of the target KR260 board with
# ``scp`` command, similarly to this:
#
##  ```bash
#  # from host desktop computer to target KR260
#  scp -r rosai_file_input ubuntu@<ip_address>:/home/ubuntu/
##  ```
#
# 2. From now on, all the steps are done from a terminal running directly on the board.
# You have to become super user and set the ``pynq_venv`` environment, as usual:
#
##```bash
#  sudo su
#  source /etc/profile.d/pynq_venv.sh
##```

# ***********************************************************************************
# 3. Create test images with the following commands:
#
##    ```bash
    python3 /home/ubuntu/KR260-Robotics-AI-Challenge/files/ROSAI/save_images_from_MNIST_dataset.py
##    ```

# ***********************************************************************************
# 4. Create the workspace folder ``/home/root/ros2_ws`` and copy there some files
# from this repository with the following commands:
#
##```bash
  cd /home/root
  mkdir -p ros2_ws_fileio
  mkdir -p ros2_ws_fileio/src
  cp -rf /home/ubuntu/KR260-Robotics-AI-Challenge/files/ROSAI/file_input/rosai_file ros2_ws_fileio/src/
  cd ros2_ws_fileio
##```

# ***********************************************************************************
# 5. You have to initialize the ROS2 environment variables and then you can build the design:
#
##```bash
  # initialize
  source /opt/ros/humble/setup.bash
  # build
  colcon build
  ## to remove the build just do
  ## rm -rf build/ install/ log/
 
##```
#
#  You can ignore the warning message like ``Use build and pip and other standards-based tools``.

# ***********************************************************************************
# 6. Then set the environment and execute the demo:
#
###```bash
#  # set env
#  #source /opt/ros/humble/setup.bash
#  source install/local_setup.sh
#  # demo run
#  ros2 launch rosai_file rosai_file_demo_launch.py
###  ```
