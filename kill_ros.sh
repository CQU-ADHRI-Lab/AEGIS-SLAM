#!/bin/bash
ps aux | grep -E "rosmaster|roscore|roslaunch|rosout|semSLAM" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
sleep 3
echo "done_killing"
