#!/bin/bash

ROOT=$(cd $(dirname $0)/.. && pwd)
APP=Demo/kakip_vslam_demo
#APP=Demo/kakip_vslam_demo_with_rasp

export LD_LIBRARY_PATH=$ROOT/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/Thirdparty/DBoW2/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/drp_ai_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/drp_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/image_load_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/image_proc_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/opencva_modules:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$ROOT/build/socket_modules:$LD_LIBRARY_PATH

/lib64/ld-linux-aarch64.so.1 --list $ROOT/$APP | grep -E 'modules|SLAM|g2o|DBoW2'

cru=$(cat /sys/class/video4linux/video*/name | grep "CRU")
csi2=$(cat /sys/class/video4linux/v4l-subdev*/name | grep "csi2")

if [ -z "$cru" ]
then
    echo "No CRU video device founds"
else
    media-ctl -d /dev/media0 -r
    media-ctl -d /dev/media1 -r
    media-ctl -d /dev/media2 -r
    media-ctl -d /dev/media3 -r

    if [ -z "$csi2" ]
    then
        echo "No MIPI CSI2 sub video device founds"
    else
        media-ctl -d /dev/media0 -V "'imx219 0-0010':0 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media0 -V "'rzg2l_csi2 16000400.csi20':1 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media0 -l "'rzg2l_csi2 16000400.csi20':1 -> 'CRU output':0 [1]"
        v4l2-ctl -d /dev/video0 --set-ctrl=digital_gain=1000
        v4l2-ctl -d /dev/video0 --set-ctrl=analogue_gain=100

        media-ctl -d /dev/media1 -V "'imx219 1-0010':0 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media1 -V "'rzg2l_csi2 16010400.csi21':1 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media1 -l "'rzg2l_csi2 16010400.csi21':1 -> 'CRU output':0 [1]"
        v4l2-ctl -d /dev/video1 --set-ctrl=digital_gain=1000
        v4l2-ctl -d /dev/video1 --set-ctrl=analogue_gain=100

        media-ctl -d /dev/media2 -V "'imx219 2-0010':0 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media2 -V "'rzg2l_csi2 16020400.csi22':1 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media2 -l "'rzg2l_csi2 16020400.csi22':1 -> 'CRU output':0 [1]"
        v4l2-ctl -d /dev/video2 --set-ctrl=digital_gain=1000
        v4l2-ctl -d /dev/video2 --set-ctrl=analogue_gain=100

        media-ctl -d /dev/media3 -V "'imx219 3-0010':0 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media3 -V "'rzg2l_csi2 16030400.csi23':1 [fmt:SRGGB8_1X8/640x480 field:none]"
        media-ctl -d /dev/media3 -l "'rzg2l_csi2 16030400.csi23':1 -> 'CRU output':0 [1]"
        v4l2-ctl -d /dev/video3 --set-ctrl=digital_gain=1000
        v4l2-ctl -d /dev/video3 --set-ctrl=analogue_gain=100
    fi
fi

cd $ROOT
$APP
