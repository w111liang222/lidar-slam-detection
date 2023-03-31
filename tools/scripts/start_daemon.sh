#!/bin/bash

if [ -f "service/upgrade/upgrade_daemon.py" ]; then
    python service/upgrade/upgrade_daemon.py &
else
    ./service/upgrade/upgrade_daemon.bin &
fi

python service/time_sync/time_sync.py &
python service/broadcast/broadcast.py &
python service/file_server/file_server.py &
python service/cloud_service/cloud_service.py &
python service/audio_service/audio_service.py &
