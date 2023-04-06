#!/bin/bash

files=`find $1 -type f`
ret=$?

if [ $ret -ne 0 ]; then
    exit $ret
fi

for file in $files; do
  sed -i '1i #define ZCM_EMBEDDED' "$file"
  sed -i '1i #undef ZCM_EMBEDDED' "$file"
  sed -i '1i \ *\/' "$file"
  sed -i '1i \ * DO NOT MODIFY THIS FILE BY HAND' "$file"
  sed -i '1i \/*' "$file"
done
