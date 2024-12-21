#!/bin/bash

kill -9 `ps aux | grep "server\.bin" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "web_backend/server.py" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
kill -9 `ps aux | grep "launch_rtsp_server" | grep -v grep | awk '{print $2}'` > /dev/null 2>&1
exit 0