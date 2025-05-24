#!/bin/bash

sed -i -e 's/Viewer.Type: SocketViewer/Viewer.Type: None/g' Examples/RGB-D/TUM3.yaml
sed -i -e 's/UseDrp: false/UseDrp: true/g'                  Examples/RGB-D/TUM3.yaml
sed -i -e 's/UseOpenCVA: false/UseOpenCVA: true/g'          Examples/RGB-D/TUM3.yaml

./Examples/RGB-D/rgbd_tum \
  $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
  $HOME/yolo-planar-slam/Examples/RGB-D/TUM3.yaml \
  /opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz \
  /opt/dataset/tum/rgbd_dataset_freiburg3_walking_xyz/associate.txt
