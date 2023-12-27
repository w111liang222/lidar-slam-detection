#!/bin/bash

cleanup() { true; }
fail() { echo "$1"; cleanup; exit 1; }

## Strict mode
set -uo pipefail
command_failed () {
    errcode=$?
    echo "ERROR: Command on line ${BASH_LINENO[0]} failed ($errcode): $BASH_COMMAND"
    cleanup
    exit $errcode
}
trap command_failed ERR

function usage() {
    echo "Usage: $0 [-hvg]"
    echo "    This script runs all set to zcmtypes through zcm-gen and verifies that"
    echo "    the output matches the pre-generated .ans files"
    echo
    echo "  OPTIONS"
    echo "    -h      Show help"
    echo "    -v      Enable verbose test output"
    echo "    -g      Generate new .ans files"
    echo

    [ $# -eq 0 ] || exit $1
}

VERBOSE=0
GENERATE=0
while getopts "hvg" opt; do
    case $opt in
        v)
            VERBOSE=1
            ;;
        h)
            usage 0
            ;;
        g)
            GENERATE=1
            ;;
        :)
            echo "Option $OPTARG requires an argument." >&2
            usage 1
            ;;
        /?)
            usage 1
            ;;
    esac
done

THISDIR=$(dirname "$(readlink -f "$0")")
BASEDIR=$(dirname $THISDIR)
TESTDIR=$BASEDIR/test/gen
TMPDIR=/tmp/zcmtypes

## Note: this runs in a subshell
gen_zcm() {(
    file=$1
    rm -fr $TMPDIR
    mkdir $TMPDIR
    cd $TMPDIR
    $BASEDIR/build/gen/zcm-gen -c -x -j $file >/dev/null
)}

check_all_ext() {
    bname=$1
    ext=$2

    ## find all files with the requested extention in both this dir and
    candfiles=$(find $TMPDIR | grep "\.$ext"'$')
    origfiles=$(find $TESTDIR/$bname.ans | grep "\.$ext"'$')
    collected=($candfiles $origfiles)
    files=$(echo "${collected[@]}" | xargs -n 1 basename | sort -u)

    for f in $files; do
        if [ "$ext" == "java" ]; then
            cand=$TMPDIR/zcmtypes/$f
            orig=$TESTDIR/$bname.ans/zcmtypes/$f
        else
            cand=$TMPDIR/$f
            orig=$TESTDIR/$bname.ans/$f
        fi
        if [ "$VERBOSE" == "1" ]; then
            echo "CHECK: '$cand' VS '$orig'"
        fi
        if [ "$GENERATE" == "1" ]; then
            cp $cand $orig
        fi
        diff $cand $orig > /tmp/zcm-test.out 2>&1
        if [ "$?" != "0" ]; then
            echo "Error: mismatch btwn $cand and $orig"
            if [ "$VERBOSE" == "1" ]; then
                cat /tmp/zcm-test.out
            fi
        fi
    done
}

compare_c() {
    bname=$1
    check_all_ext $bname h
    check_all_ext $bname c
}

compare_cpp() {
    bname=$1
    check_all_ext $bname hpp
}

compare_java() {
    bname=$1
    check_all_ext $bname java
}

# compare_python() {
#     bname=$1
#     check_all_ext $bname py
# }

## Add new language entries here
compare() {
    bname=$1
    compare_c $bname
    compare_cpp $bname
    #compare_python $bname
    compare_java $bname
}

ZCMFILES=$(ls $TESTDIR/*.zcm)

for f in $ZCMFILES; do
    name=$(basename $f)
    bname=${name%.*}

    if [ "$VERBOSE" == "1" ]; then
        echo "TEST: $bname"
    fi

    gen_zcm $f
    compare $bname
done
