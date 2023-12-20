#!/bin/bash
#
# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# ***********************************************************************************
# 1. You have already copied the folder "rosai_camera_input" from this repository
# to the destination folder ``/home/ubuntu`` of the target KR260 board with
# ``scp`` command, similarly to this:
#
##  ```bash
#  # from host desktop computer to target KR260
#  scp -r rosai_camera_input ubuntu@<ip_address>:/home/ubuntu/
##  ```
#
# 2. From now on, all the steps are done from a terminal running directly on the board.
# You have to become super user and set the ``pynq_venv`` environment, as usual:
#
##```bash
#  sudo su
#  source /etc/profile.d/pynq_venv.sh
##```

: '
# ***********************************************************************************
# 3. Move the ``black_background_images`` archive from ``/home/ubuntu/rosai_camera_input``
# to ``/home/ubuntu/images/``, with the following commands:
#
##    ```bash
      cd /home/ubuntu
      # create new folder
      mkdir -p images
      # move zip archive
      mv rosai_camera_input/black_background_images ./images
##    ```
'

# ***********************************************************************************
# 4. Create the workspace folder ``/home/root/ros2_ws`` and copy there some files from this repository with the following commands:
#
##    ```bash
      cd /home/root
      mkdir -p ros2_ws
      mkdir -p ros2_ws/src
      cp -rf /home/ubuntu/KR260-Robotics-AI-Challenge/files/ROSAI/camera_input/rosai_camera ros2_ws/src/
      cd ros2_ws
##    ```

# ***********************************************************************************
# 5. You have to initialize the ROS2 environment variables and then you can build the design:
#
##    ```bash
      # initialize
      source /opt/ros/humble/setup.bash
      # build
      colcon build
##    ```
#
# You can ignore the warning message like ``Use build and pip and other standards-based tools``.

# ***********************************************************************************
# 6. You can execute the demo. Note that you have to use the images from the folder
# ``rosai_camera_input/black_background_images``: either you print them in a paper
# (one image per page) or you put them in an iPad device and the you can put one after
# the other those images in front of the camera.
# Here are the commands:
#
##   ```bash
#    # set the environment
#    source install/local_setup.sh
#    # demo the camera run
#    ros2 launch rosai_camera rosai_camera_demo_launch.py
##   ```
