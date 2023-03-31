#!/bin/bash

cd /tmp/detection_sys

MACHINE=`uname -m`

# copy config file to runtime path
cp -rf /home/znqc/work/cfg/* cfg/

mkdir -m777 -p output/logs

# set max performance
if [[ "$MACHINE" == "aarch64" ]];then
    /usr/bin/jetson_clocks
elif [[ "$MACHINE" == "x86_64" ]];then
    echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
fi

# set realtime timeslice to no limit
# BUG: https://github.com/coreos/bugs/issues/410
sysctl -w kernel.sched_rt_runtime_us=-1

# set udp buffer to 32MB
sysctl -w net.core.rmem_max=33554432
sysctl -w net.core.rmem_default=33554432
sysctl -w net.core.wmem_max=33554432
sysctl -w net.core.wmem_default=33554432
sysctl -w net.core.netdev_max_backlog=2000

# set realtime thread time slice to 10ms
sysctl -w kernel.sched_rr_timeslice_ms=10

# set the memory swappiness to 10%
sysctl -w vm.swappiness=10

# setup network
python util/setup_network.py

# start detection system
tools/scripts/start_system.sh &

# start daemon services
tools/scripts/start_daemon.sh

cd -
