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

color_bold=`tput bold`
color_redf=`tput setaf 1`
color_reset=`tput sgr0`

STRICT=true
SINGLE_MODE=false
USE_JULIA=true
JULIA_0_6_MODE=false
USE_NODE=true
USE_PYTHON_2=false
while getopts "ijmsp" opt; do
    case $opt in
        i) STRICT=false ;;
        j) USE_JULIA=false ;;
        m) JULIA_0_6_MODE=true ;;
        n) USE_NODE=false ;;
        s) SINGLE_MODE=true ;;
        p) USE_PYTHON_2=true ;;
        \?) exit 1 ;;
    esac
done

mkdir -p $ROOTDIR/deps

PKGS=''
PIP_PKGS=''

## Dependency dependencies
PKGS+='mlocate wget '

## Waf dependencies
PKGS+='pkg-config zip '

## Basic C compiler dependency
PKGS+='build-essential '

## Lib ZMQ
PKGS+='libzmq3-dev '

## Java
PKGS+='default-jdk default-jre '

## Python
if $USE_PYTHON_2; then
    PKGS+='python python-pip '
else
    PKGS+='python3 python3-pip '
fi
PIP_PKGS+='Cython '
PIP_PKGS+='bitstruct '

# Build cache dep
PIP_PKGS+='urllib3 '

## LibElf
PKGS+='libelf-dev libelf1 '

## Gtk+3
PKGS+='libgtk-3-dev '

## Clang tools for code sanitizers, style checkers, etc.
PKGS+='clang '

echo "Updating apt repos"
sudo apt-get update
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to update"
    exit $ret
fi

echo "Installing from apt"
if $SINGLE_MODE; then
    for pkg in $PKGS; do
        echo "Installing $pkg"
        sudo apt-get install -yq $pkg
        ret=$?
        if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
            echo "Failed to install packages"
            exit $ret
        fi
    done
else
    sudo apt-get install -yq $PKGS
fi

if $USE_PYTHON_2; then
    pip install --user $PIP_PKGS
else
    pip3 install --user $PIP_PKGS
fi
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to install pip packages"
    exit $ret
fi

## Node
if $USE_NODE; then
    bash -c "export NVM_DIR=\$HOME/.nvm; \
        [ -s \"\$NVM_DIR/nvm.sh\" ] && \
        \\. \"\$NVM_DIR/nvm.sh\"; \
        nvm --version" > /dev/null 2>&1
    nvmExists=$?
    if [ $nvmExists -ne 0 ]; then
        echo "Downloading NVM"
        unset NVM_DIR
        outfile=$(mktemp)
        wget -q -O$outfile https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh
        echo "Installing NVM"
        chmod +x $outfile
        $outfile
        rm $outfile
    fi
    echo "Installing node version v10.23.3"
    bash -c "export NVM_DIR=\$HOME/.nvm; \
        [ -s \"\$NVM_DIR/nvm.sh\" ] && \
        \\. \"\$NVM_DIR/nvm.sh\"; \
        nvm install v10.23.3 && \
        nvm alias default v10.23.3"
fi

## Julia
checkJuliaInstall()
{
    juliaVersion=$(julia --version 2>/dev/null)
    juliaExists=$?
    juliaVersion=$(echo "$juliaVersion" | xargs | cut -d ' ' -f 3)

    expectedVersion="1.6.0"
    if $JULIA_0_6_MODE; then
        expectedVersion="0.6.4"
    fi

    if [ $juliaExists -ne 0 ] || [ "$juliaVersion" != "$expectedVersion" ]; then
        return 1
    else
        return 0
    fi
}
if $USE_JULIA; then
    checkJuliaInstall
    ret=$?
    if [ $ret -ne 0 ]; then
        echo "Installing julia"
        tmpdir=$(mktemp -d)
        pushd $tmpdir > /dev/null

        ARCH=$(uname -m | sed -e s/i.86/i686/ -e s/x86_64/x86_64/ -e s/aarch64/aarch64/)
        FOLDER=$(echo "$ARCH" | sed -e s/x86_64/x64/ -e s/i686/x86/ -e s/aarch64/aarch64/)

        echo "Installing julia for $ARCH"

        if $JULIA_0_6_MODE; then
            wget -q https://julialang-s3.julialang.org/bin/linux/$FOLDER/0.6/julia-0.6.4-linux-$ARCH.tar.gz
            if [ $? -ne 0 ]; then >&2 echo "Unable to download julia 0.6"; exit 1; fi
            tar -xaf julia-0.6.4-linux-$ARCH.tar.gz
            rm -rf $ROOTDIR/deps/julia
            mv julia-9d11f62bcb $ROOTDIR/deps/julia
        else
            wget -q https://julialang-s3.julialang.org/bin/linux/$FOLDER/1.6/julia-1.6.0-linux-$ARCH.tar.gz
            if [ $? -ne 0 ]; then >&2 echo "Unable to download julia 1.6"; exit 1; fi
            tar -xaf julia-1.6.0-linux-$ARCH.tar.gz
            rm -rf $ROOTDIR/deps/julia
            mv julia-1.6.0 $ROOTDIR/deps/julia
        fi

        popd > /dev/null
        rm -rf $tmpdir
        echo "Julia has been downloaded to $ROOTDIR/deps"
        echo -n "$color_bold$color_redf"
        echo    "You must add the following to your ~/.bashrc:"
        echo    "    PATH=\$PATH:$ROOTDIR/deps/julia/bin"
        echo -n "$color_reset"
    else
        echo "Found julia on system. Skipping install"
    fi
fi

# Install cxxtest
rm -rf $ROOTDIR/deps/cxxtest
wget -O $ROOTDIR/deps/cxxtest.zip https://github.com/ZeroCM/cxxtest/archive/master.zip
unzip -d $ROOTDIR/deps $ROOTDIR/deps/cxxtest.zip
mv $ROOTDIR/deps/cxxtest{-master,}

echo "Updating db"
sudo updatedb
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to updatedb"
    exit $ret
fi

exit 0
