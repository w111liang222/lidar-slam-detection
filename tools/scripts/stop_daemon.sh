#!/bin/bash

kill -9 `ps aux | grep "service/upgrade/upgrade_daemon.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "service/time_sync/time_sync.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "ptpd" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "chronyd" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "gpsd" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "service/broadcast/broadcast.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "service/file_server/file_server.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "HTTPFileServer.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "frpc" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "service/cloud_service/cloud_service.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "gst-launch-1.0" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "service/audio_service/audio_service.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
