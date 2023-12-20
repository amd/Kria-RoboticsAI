#!/bin/bash

# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# **********************************************************************
#
# First of all, boot the board and open a terminal and enter in the pynq_env:
#
# sudo su
# source /etc/profile.d/pynq_venv.sh
#
# Then launch the next commands (see the comments):

# **********************************************************************
### 5.1 Set locale

#```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
#```

# **********************************************************************
### 5.2 Setup Sources

# You will need to add the ROS2 apt repository to your system.
#
# First ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.
#
##```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
##```

# First, add "185.199.108.133 raw.githubusercontent.com" to end of file "/etc/hosts". It'll avoid error of connection fail
# Then add the ROS2 GPG key with apt.
#
##```bash
sed -i -e 'a185.199.108.133 raw.githubusercontent.com' /etc/hosts
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
##```

# Then add the repository to your sources list.
#
##```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
##```

# **********************************************************************
### 5.3 Install Packages

# ROS2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages:
#
##```bash
sudo apt update
sudo apt upgrade
##```

# With the next three commands you can install respectively the desktop, the base libraries and the development tools (in reality, the first command seems also to have installed already the base packages, this is why the second command is commented):
#
##```bash
sudo apt install ros-humble-desktop
#sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
##```

# **********************************************************************
### 5.4 Environment Setup

# Set up your environment by sourcing the following file.
#
##```bash
source /opt/ros/humble/setup.bash
##```

# If you don’t want to source the setup file every time you open a new shell, you can add the command to your shell startup script:
#
##```bash
## echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
##```

# **********************************************************************
### 5.5 Install and Test TurtleSim

# Also the TurtleSim package should be already installed from the previous actions. If not, run the following commands:
#
##```bash
sudo apt update
sudo apt install ros-humble-turtlesim
##```


# Check that the ROS package is effectively installed:
#
##```bash
ros2 pkg executables turtlesim
##```
#
# You should see the following output text, which is  a list of TurtleSim executables:
#
##```text
# turtlesim draw_square
# turtlesim mimic
# turtlesim turtle_teleop_key
# turtlesim turtlesim_node
##```

# To start TurtleSim, enter the following command in your terminal:
#
##```bash
#ros2 run turtlesim turtlesim_node
##```

# You can control the turtle with the keyboard:
#
##```bash
#ros2 run turtlesim turtle_teleop_key
##```
