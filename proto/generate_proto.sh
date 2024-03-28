#!/bin/bash

echo "Generating cpp proto"

CURRENT_PATH=$1
cd ${CURRENT_PATH}
protoc ./detection.proto --cpp_out=./
cd -