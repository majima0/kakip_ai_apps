#!/bin/bash

sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' Examples/Monocular/TUM3.yaml
sed -i -e 's/UseDrp: false/UseDrp: true/g'                  Examples/Monocular/TUM3.yaml
sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g'          Examples/Monocular/TUM3.yaml

./Examples/Monocular/mono_tum \
  $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
  $HOME/yolo-planar-slam/Examples/Monocular/TUM3.yaml \
  /opt/dataset/tum/GRAY_rgbd_dataset_freiburg3_walking_xyz
