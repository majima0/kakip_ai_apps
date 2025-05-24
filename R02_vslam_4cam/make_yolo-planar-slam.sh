#!/bin/bash

PROJECT_SOURCE_DIR=$(cd $(dirname $0) && pwd)
export SDKTARGETSYSROOT=/opt/sysroots/arm64-ubuntu
export PKG_CONFIG_SYSROOT_DIR=${SDKTARGETSYSROOT}
export PKG_CONFIG_PATH=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${SDKTARGETSYSROOT}/usr/share/pkgconfig
#export CMAKE_PREFIX_PATH=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu/cmake:${SDKTARGETSYSROOT}/usr/lib/cmake:${SDKTARGETSYSROOT}/usr/share/cmake
#export CMAKE_PREFIX_PATH=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu

#export LD_LIBRARY_PATH=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

#  --debug-find \
#  -DCMAKE_VERBOSE_MAKEFILE='ON' \
cmake -DCMAKE_BUILD_TYPE=Release \
  --graphviz=deps.dot \
  -DCMAKE_TOOLCHAIN_FILE=../toolchain-aarch64.cmake \
  -DENABLE_MEASURE_TIME=ON \
  -DENABLE_DUMP=OFF \
  -DENABLE_DRP=ON \
  -DENABLE_DRP_AI=ON \
  -DENABLE_REALSENSE2=OFF \
  -DENABLE_GOOGLE_PERF=OFF \
  -DENABLE_YOCTO=ON \
  -DENABLE_SLAMFAST=OFF \
  -DENABLE_DRPAI_EXPAND_MEMORY_SPACE=OFF \
  -DENABLE_CALL_OPENCVA_DIRECTLY=ON \
  -DOpenMP_C_FLAGS='-fopenmp' \
  -DOpenMP_CXX_FLAGS='-fopenmp' \
  -DOpenMP_C_LIB_NAMES='gomp;pthread' \
  -DOpenMP_CXX_LIB_NAMES='gomp;pthread' \
  -DOpenMP_gomp_LIBRARY=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu/libgomp.so.1 \
  -DOpenMP_pthread_LIBRARY=${SDKTARGETSYSROOT}/usr/lib/aarch64-linux-gnu/libpthread.so.0 \
  -DYOLO_PLANAR_SLAM_VERSION=${YOLO_PLANAR_SLAM_VERSION} \
  ..
make -j2
