#!/bin/bash

ROOT=$(cd $(dirname $0)/.. && pwd)

NODE_PATH=/usr/local/lib/node_modules node $ROOT/viewer/app.js