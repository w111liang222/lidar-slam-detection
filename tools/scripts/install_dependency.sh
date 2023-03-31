#!/bin/bash -e

MACHINE=`uname -m`

# gstreamer plugins
cp tools/build/resources/gstreamer/$MACHINE/* /usr/lib/$MACHINE-linux-gnu/gstreamer-1.0/

# libelf-dev
dpkg -i tools/build/resources/$MACHINE/libelf-dev_0.176-1.1build1.deb

# ZeroCM
if [ ! -f "/usr/local/lib/libzcm.so" ]; then
    echo "Install ZeroZCM..."
    cd tools/build/resources/ZeroCM
    ./waf configure --use-ipc --use-zmq --use-elf
    ./waf build
    ./waf install

    # python extension
    PYTHONPATH=$PYTHONPATH:/home/znqc/.local/lib/python3.8/site-packages python zcm/python/setup.py bdist_wheel
    pip install -I dist/zerocm-1.1.5-cp38-cp38-linux_${MACHINE}.whl
    cd -
fi

# pyserial
pip install -I tools/build/resources/pyserial/pyserial-3.4-py2.py3-none-any.whl

ldconfig
