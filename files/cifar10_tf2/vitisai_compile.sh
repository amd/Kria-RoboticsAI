#!/bin/sh

# Copyright (C) 2023, Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT

# USAGE:
# ./vitisai_compile.sh $1 $2 $3 $4

CNN_MODEL=$2
OUTPUT_DIR=$3
NET_NAME=$4


if [ $1 = zcu102 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/ZCU102/arch.json
      TARGET=zcu102
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ZCU102..."
      echo "-----------------------------------------"
elif [ $1 = kr260 ]; then
            ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json
            TARGET=kr260
            echo "-----------------------------------------"
            echo "COMPILING MODEL FOR KR260..."
            echo "-----------------------------------------"
elif [ $1 = kv260 ]; then
            ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/KV260/arch.json
            TARGET=kv260
            echo "-----------------------------------------"
            echo "COMPILING MODEL FOR KV260..."
            echo "-----------------------------------------"
elif [ $1 = zcu104 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCZDX8G/ZCU104/arch.json
      TARGET=zcu104
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ZCU104..."
      echo "-----------------------------------------"
elif [ $1 = vck190 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCVDX8G/VCK190/arch.json
      TARGET=vck190
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR VCK190..."
      echo "-----------------------------------------"
elif [ $1 = v70 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCV2DX8G/V70/arch.json
      TARGET=v70
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR ALVEO V70..."
      echo "-----------------------------------------"
elif [ $1 = vek280 ]; then
    ARCH=/opt/vitis_ai/compiler/arch/DPUCV2DX8G/VEK280/arch.json
    TARGET=vek280
    echo "-----------------------------------------"
    echo "COMPILING MODEL FOR VEK280..."
    echo "-----------------------------------------"
elif [ $1 = vck5000 ]; then
      ARCH=/opt/vitis_ai/compiler/arch/DPUCVDX8H/VCK50004PE/arch.json
      TARGET=vck5000
      echo "-----------------------------------------"
      echo "COMPILING MODEL FOR VCK5000..."
      echo "-----------------------------------------"


else
      echo  "Target not found. Valid choices are: zcu102, zcu104, vck190, vck5000, vek280, v70, kr260... exiting"
      exit 1
fi


compile() {
    vai_c_tensorflow2 \
	--model           ./${CNN_MODEL} \
	--arch            ${ARCH} \
	--output_dir      ${OUTPUT_DIR} \
	--net_name        ${NET_NAME} 
#	--options         "{'mode':'debug'}"
#  --options         '{"input_shape": "1,32,32,3"}' \
}


compile #2>&1 | tee build/log/compile_$TARGET.log


echo "-----------------------------------------"
echo "MODEL COMPILED"
echo "-----------------------------------------"
