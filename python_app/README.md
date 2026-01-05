pydrpaiインストール  
ビルド毎にwheelのハッシュ値が変わるので更新する
```
uv remove pydrpai
uv add ../pydrpai/dist/pydrpai-0.1.0-cp312-cp312-linux_aarch64.whl
```

実行
```
uv run main.py --model-dir /path/to/yolov3_onnx --video-path /path/to/sample_1080p_h264.mp4 [--print(標準出力), --display(画面出力)]
```
