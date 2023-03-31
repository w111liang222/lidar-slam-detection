#!/bin/bash -e

./tools/scripts/install_dependency.sh

MACHINE=`uname -m`

pip uninstall -y sensor_driver_wrapper || true
pip uninstall -y freespace_wrapper || true
pip uninstall -y slam_wrapper || true

# perception
pip install -I tools/build/resources/perception-1.0.0-cp38-cp38-linux_${MACHINE}.whl

# clean
rm -rf tools/build/