#!/bin/bash -e

MACHINE=`uname -m`

# ZeroCM
if [ ! -f "/usr/local/lib/libzcm.so" ]; then
    echo "Install ZeroZCM..."
    cd tools/build/resources/ZeroCM
    ./waf configure --use-ipc --use-zmq --use-elf --libdir=/usr/local/lib
    ./waf build
    ./waf install

    # python extension
    PYTHONPATH=$PYTHONPATH:/home/znqc/.local/lib/python3.8/site-packages python zcm/python/setup.py bdist_wheel
    pip install -I dist/zerocm-1.1.5-cp38-cp38-linux_${MACHINE}.whl
    cd -
fi

ldconfig
