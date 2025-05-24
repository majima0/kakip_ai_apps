#!/bin/bash
sed -i -e 's/Viewer.Type: None/Viewer.Type: SocketViewer/g' Examples/Monocular/ELP_rns-2022-0901.yaml
sed -i -e 's/UseDrp: true/UseDrp: false/g'                  Examples/Monocular/ELP_rns-2022-0901.yaml
sed -i -e 's/UseOpenCVA: true/UseOpenCVA: false/g'          Examples/Monocular/ELP_rns-2022-0901.yaml

./Examples/Monocular/mono_usbcam \
  $HOME/yolo-planar-slam/Vocabulary/ORBvoc.txt \
  $HOME/yolo-planar-slam/Examples/Monocular/ELP_rns-2022-0901.yaml
