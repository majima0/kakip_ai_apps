ビルド
```
python3 packaging/setup.py bdist_wheel
```

インストール
```
pip3 install --break-system-packages dist/pydrpai-0.0.1-cp312-cp312-linux_aarch64.whl
```

実行
```
python3 src/script.py --model-dir /path/to/yolov3_onnx --video-path /path/to/sample_1080p_h264.mp4
```
