#!/bin/bash

ROOT=$(cd $(dirname $0)/.. && pwd)

export LD_LIBRARY_PATH=$ROOT/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/drp_ai_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/drp_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/image_load_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/image_proc_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/opencva_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/socket_modules:$LD_LIBRARY_PATH

/lib64/ld-linux-aarch64.so.1 --list $ROOT/Demo/kakip_vslam_demo | grep -E 'modules|SLAM|g2o|DBoW2'
