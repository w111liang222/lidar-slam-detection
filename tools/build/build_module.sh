#!/bin/bash -e

CUDA_HOME=/usr/local/cuda
PATH=$PATH:$CUDA_HOME/bin
LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
MACHINE=`uname -m`

# build perception
python setup.py bdist_wheel
if [[ "$CI_JOB_NAME" == "commit_build"* ]];then
    pip install -I dist/perception-1.0.0-cp38-cp38-linux_${MACHINE}.whl
else
    mv dist/perception-1.0.0-cp38-cp38-linux_${MACHINE}.whl tools/build/resources/
    rm -rf build/ dist/ *.egg*
fi

# workaround for server docker CI
if [[ "$CI_JOB_NAME" == "commit_build"* ]];then
    echo "$CI_JOB_NAME"
else
    # use prebuilt wheel to overwrite
    if [[ "$MACHINE" == "x86_64" ]];then
        wget http://10.10.80.28:8888/tsari_ad_ipu/Archive/builds/x86_64/perception-1.0.0-cp38-cp38-linux_x86_64.whl
        mv *.whl tools/build/resources/
    fi
fi