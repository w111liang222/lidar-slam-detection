#!/bin/bash

#### Find the script directory regardless of symlinks, etc
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
    # if $SOURCE was a relative symlink, we need to resolve it relative
    # to the path where the symlink file was located
done
THISDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
ROOTDIR=${THISDIR%/scripts}
####


#### We want to exit with an error code if anything goes wrong
set -uo pipefail
function command_failed() {
    errcode=$?
    echo -e "\nERROR: Command on line ${BASH_LINENO[0]} failed ($errcode): $BASH_COMMAND\n"
    exit $errcode
}
trap command_failed ERR

## SETUP

# In this directory we assemble the deb package. Later we call dpkg-deb on it to pack the package.
DEB_PACKAGE_ASSEMBLY_DIR=./build/deb_package_root
mkdir -p $DEB_PACKAGE_ASSEMBLY_DIR/usr/

# Required to find java
if [ -z ${JAVA_HOME+x} ]; then
    export JAVA_HOME=$(readlink -f /usr/bin/javac | sed "s:/bin/javac::")
fi

# Change to the directory containing the source code
cd $ROOTDIR


## BUILD
./waf configure distclean

./waf configure --use-all --use-third-party --prefix=$DEB_PACKAGE_ASSEMBLY_DIR/usr/
./waf build
./waf install


### HACKS TO PREPARE DEBIAN PACKAGE STRUCTURE

# Move the debian control files directory to the temporary $DEB_PACKAGE_ASSEMBLY_DIR
cp -r ./build/DEBIAN $DEB_PACKAGE_ASSEMBLY_DIR


cd $DEB_PACKAGE_ASSEMBLY_DIR
# Unfortunately waf automatically installs to 'pythonX.X/site-packages' as soon as the
# root directory is not contained in the install prefix.
# We need it in 'dist-packages' so we just rename it manually here.
# Note: since this modifies the folder structure that 'find' is iterating, it causes
#       find to print an error such as:
#       "find: ‘./usr/lib/python3.6/site-packages’: No such file or directory".
# It works anyways ...
find -type d -wholename '*python*/site-packages' -execdir mv ./site-packages ./dist-packages \; || true

# There are a number of files in which the install prefix appears such as the java
# launchers in usr/bin and the package-config files.
# This is undesirable since the temporary install prefix in $DEB_PACKAGE_ASSEMBLY_DIR
# is obviously wrong after the files have been installed.
# The following lines replaces all occurences of the $DEB_PACKAGE_ASSEMBLY_DIR as
# path with '/usr' which is our actual install prefix with the debian package.
find -type f -exec sed -i "s+$PWD++g" {} +
cd -

### PACK DEBIAN PACKAGE
##  Debian compliance: fakeroot is required to get correct uids and gids for all installed files
fakeroot dpkg-deb -b $DEB_PACKAGE_ASSEMBLY_DIR
dpkg-name $DEB_PACKAGE_ASSEMBLY_DIR.deb
