command_3 = '''\
#!/bin/bash -e

cd service/upgrade/cache/firmware/detection_sys

# start system
./tools/scripts/start_system.sh &

i=0
while [ $i -le 60 ]
do
    sleep 1
    let i+=1

    if [ -f "output/logs/status" ]; then
        status=`cat output/logs/status`
        echo $status
        if [ "$status" == "Running" ];then
            ./tools/scripts/stop_system.sh || true
            cd -
            exit 0
        fi
    else
        echo "no status exits"
    fi
done

echo "upgrade verify fail"

cd -

# kill remain process
./tools/scripts/stop_system.sh || true

exit 1
'''