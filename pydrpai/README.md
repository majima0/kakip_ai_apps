uvインストール
[Installation methods](https://docs.astral.sh/uv/getting-started/installation/#installation-methods)を実行する  
その後、ターミナルを再度開く
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```

ビルド
```
cd /path/to/kakip_ai_apps/pydrpai
uv build --config-setting=cmake.args="-DCURRENT_DIR=${PWD}"
```
