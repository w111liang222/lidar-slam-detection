command_4 = '''\
#!/bin/bash

cd service/upgrade/cache/firmware

# clear logs
rm -rf detection_sys/output/

# replace the config
rm -rf /home/znqc/work/cfg
cp -r detection_sys/cfg /home/znqc/work/

# generate the new firmware
7za a -tzip -pCrvsVM4z5ReV -mem=AES256 detection_sys.zip detection_sys/*
mv detection_sys.zip /home/znqc/work/
sync

cd -

# clean up
rm -rf service/upgrade/cache
'''