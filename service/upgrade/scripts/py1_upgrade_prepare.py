command_1 = '''\
#!/bin/bash

# kill detection_sys backend
./tools/scripts/stop_system.sh

# remove old files
rm -rf service/upgrade/cache/firmware

# check signature
cd service/upgrade/cache/
signature=`openssl rsautl -verify -inkey ../rsapublickey.pub -pubin -keyform PEM -in signature.bin`
hashkey=`openssl sha1 firmware.zip`

if [ "$signature" != "$hashkey" ];then
    echo "$signature" "not match" "$hashkey"
    exit 1
fi

cd -

# unzip the firmware
7za x -pCrvsVM4z5ReV service/upgrade/cache/firmware.zip -oservice/upgrade/cache/firmware
rm -rf service/upgrade/cache/firmware.zip
'''