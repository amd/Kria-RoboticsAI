#!/bin/bash

# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# Author:     Sarunas Kalade, AMD
# reviewed by Daniele Bagni, AMD
# Date: 17 Nov 2023

# It is assumed that your KR260 board has IP address 192.168.1.186


# become super user
# sudo su

set -e # exit immediately if a command has a non-zero status

# ***********************************************************************************************
# STEP 1: download the archive vai3.5_kr260.zip from public URL:
# ***********************************************************************************************
echo " "
echo "STEP 1"
echo " "
cd /home/ubuntu
wget -O vai3.5_kr260.zip https://www.xilinx.com/bin/public/openDownload?filename=vai3.5_kr260.zip

# ***********************************************************************************************
# STEP 2: launch the KRIA-PYNQ install on KR260
# (see also https://github.com/Xilinx/Kria-PYNQ#2-install-pynq)
# ***********************************************************************************************
echo " "
echo "STEP 2"
echo " "
cd /home/ubuntu
git clone https://github.com/Xilinx/Kria-PYNQ.git
cd Kria-PYNQ/
# from PYNQ SW stack point of view KV260 and KR260 are the same, so select one of the following two lines
# bash install.sh -b KR260 # this will install less jupyter notebook applications
bash install.sh -b KV260  # this will install more jupyter notebook applications


# ***********************************************************************************************
# STEP 3: add the patches where needed
# ***********************************************************************************************
echo " "
echo "STEP 3"
echo " "
cd /home/ubuntu
unzip vai3.5_kr260.zip
pushd vai3.5_kr260/target/runtime_deb/
bash setup.sh
cd ..
tar -xzf lack_lib.tar.gz
cp -r lack_lib/* /usr/lib
popd
cd vai3.5_kr260
cp ./xbutil_tool/xbutil2 /usr/bin/unwrapped/

# ***********************************************************************************************
# STEP 4: Install DPU PYNQ for 3.5
# ***********************************************************************************************
echo " "
echo "STEP 4"
echo " "
cd /home/ubuntu
git clone https://github.com/Xilinx/DPU-PYNQ -b design_contest_3.5
cd DPU-PYNQ
# make sure you're in pynq-venv so pynq_dpu gets upgraded on the root user
source /etc/profile.d/pynq_venv.sh
python3 -m pip install . --no-build-isolation

# ***********************************************************************************************
# STEP 5: Jupiter Notebooks for 3.5
# ***********************************************************************************************
echo " "
echo "STEP 5"
echo " "
# purge 2.5 models and notebooks, update with new ones from 3.5 install
cd /home/root/jupyter_notebooks
rm -rf pynq-dpu
pynq get-notebooks pynq-dpu -p . --force

# ***********************************************************************************************
# STEP 6: Other patches
# ***********************************************************************************************
echo " "
echo "STEP 6"
echo " "

# need to add /usr/lib for VART to detect libunilog libs
sed -i -e '$aexport LD_LIBRARY_PATH=/usr/lib' /etc/profile.d/pynq_venv.sh

# patch xdputil so it works in pynq-venv
sed -i "s/\/usr\/bin\///g" /usr/bin/xdputil

# if really needed you can uncomment this
#sudo sed -i -e '$aexport PYTHONPATH=/usr/lib/python3.10/site-packages:$PYTHONPATH' /etc/profile.d/pynq_venv.sh

# ***********************************************************************************************
# STEP 7: reboot
# ***********************************************************************************************

#reboot
