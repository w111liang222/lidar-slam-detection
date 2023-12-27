#!/bin/bash

mkdir -p output/logs

MACHINE=`uname -m`

PRELOAD_LIBS="/usr/lib/$MACHINE-linux-gnu/libgomp.so.1 /lib/$MACHINE-linux-gnu/libGLdispatch.so"
EXT_PYTHONPATH=/home/znqc/.local/lib/python3.8/site-packages
EXT_LIBRARY_PATH=/usr/local/lib/python3.8/dist-packages

if [ -f "server.bin" ]; then
    echo "run server binary"
    run="./server.bin"
else
    echo "run server python script"
    run="python web_backend/server.py"
fi

OPENBLAS_NUM_THREADS=1 LD_PRELOAD="$PRELOAD_LIBS" PYTHONPATH=$PYTHONPATH:$EXT_PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$EXT_LIBRARY_PATH ${run} cfg/board_cfg_all.yaml