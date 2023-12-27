#!/bin/bash
socatExists=$(which socat)
if [ "$socatExists" == "" ]; then
    echo "This script needs socat installed to run"
    echo "sudo apt-get install socat"
    exit 1
fi
socat -d -d pty,raw,echo=0 pty,raw,echo=0
