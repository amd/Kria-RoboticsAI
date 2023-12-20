# Copyright © 2023 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# Author: Daniele Bagni, Xilinx Inc
# date:   08 Nov 2023

import os
import sys

#from pynq_dpu import DpuOverlay
#overlay = DpuOverlay("dpu.bit")

# ***********************************************************************
# Path names of the various folders
# ***********************************************************************

# base folder
base_dir = "/home/root/jupyter_notebooks/pynq-dpu/cifar10_tf2"

# relative folders
executable      = "code/run_cnn.exe"
xmodel_file     = "compiled/kr260_cifar10_tf2_resnet18.xmodel"
test_images_dir = "test_images"
labels_filename = "cifar10_labels.dat"
logfile         = "cpp_cifar10_predictions.log"

# absolute pathnames
os_exe  = os.path.join(base_dir, executable)
os_xmod = os.path.join(base_dir, xmodel_file)
os_test = os.path.join(base_dir, test_images_dir)
os_lab  = os.path.join(base_dir, labels_filename)
os_log  = os.path.join(base_dir, logfile)

# ***********************************************************************
# launch the C++ compiled executable
# ***********************************************************************
command = os_exe + " " + os_xmod + " " + os_test + "/ " + os_lab + " | tee " + logfile
#+ " 2>&1 /dev/null"

print("exe    : ", os_exe,  file=sys.stderr)
print("xmodel : ", os_xmod, file=sys.stderr)
print("img dir: ", os_test, file=sys.stderr)
print("labels : ", os_lab,  file=sys.stderr)
print("logfile: ", os_log,  file=sys.stderr)
print("command: ", command, file=sys.stderr)

print("\n Launching Inference on MNIST with C++ Compiled Executable", file=sys.stderr)
print(" using C++ VART APIs\n", file=sys.stderr)

res = os.system(command)

if res == 0 :
    print("\n successfull run. Exited with ", res, file=sys.stderr)
else:
    print("\n FAILED run!!!!!! Exited with ", res, file=sys.stderr)


# ***********************************************************************
# Clean up
# ***********************************************************************

#del overlay
#del dpu
