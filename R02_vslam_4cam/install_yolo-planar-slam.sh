#!/bin/bash

#export SSH_FLAGS=-oHostKeyAlgorithms=+ssh-rsa
#export TARGET_HOST=kakip-es1
#export TARGET_HOST=rzv2h
#export TARGET_USER=root
#export TARGET_DIR=/home/root/yolo-planar-slam
export TARGET_HOST=192.168.40.206
export TARGET_USER=ubuntu
export TARGET_DIR=/home/ubuntu/yolo-planar-slam
export LIB_DIRS="lib Thirdparty/g2o/lib Thirdparty/DBoW2/lib build/Thirdparty/sioclient build/drp_ai_modules build/drp_modules build/image_load_modules build/image_proc_modules build/opencva_modules build/socket_modules"

shopt -s expand_aliases
alias ssh='sshpass -pubuntu ssh'
alias scp='sshpass -pubuntu scp'

if [ $1 ]; then
  OBJ=$1
else
  OBJ="all"
fi

ssh $SSH_FLAGS ${TARGET_USER}@${TARGET_HOST} mkdir -p ${TARGET_DIR}

if [ $OBJ = "voc" ] || [ $OBJ = "all" ]; then
  scp $SSH_FLAGS -r Vocabulary ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}
  ssh $SSH_FLAGS ${TARGET_USER}@${TARGET_HOST} sh -c "cd\ ${TARGET_DIR}/Vocabulary\&\&tar\ zxf\ ORBvoc.txt.tar.gz\&\&rm\ ORBvoc.txt.tar.gz"
fi

if [ $OBJ = "yolo" ] || [ $OBJ = "all" ]; then
  scp $SSH_FLAGS -r YOLOX_S_dense_640x640_RGB_10271351 configuration_code viewer script ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}
fi

if [ $OBJ = "app" ] || [ $OBJ = "all" ]; then
  scp $SSH_FLAGS -r script Examples ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}
fi

if [ $OBJ = "lib" ] || [ $OBJ = "all" ]; then
  ssh $SSH_FLAGS ${TARGET_USER}@${TARGET_HOST} mkdir -p ${TARGET_DIR}/lib
  for d in $LIB_DIRS; do
    scp $SSH_FLAGS ${d}/*.so.* ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}/lib 
    scp $SSH_FLAGS ${d}/*.so ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}/lib 
  done
fi

if [ $OBJ = "demo" ] || [ $OBJ = "all" ]; then
  ssh $SSH_FLAGS ${TARGET_USER}@${TARGET_HOST} mkdir -p ${TARGET_DIR}/Demo
  scp $SSH_FLAGS Demo/kakip_vslam_demo Demo/kakip_vslam_demo_with_rasp Demo/settings.yaml* ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}/Demo
fi