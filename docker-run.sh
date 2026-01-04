#!/bin/bash
screen -S robocap-calib docker run -it --entrypoint="" -v /home/ubuntu/workspace/robocap-tools:/robocap -v /home/ubuntu/workspace/dataset:/data robocap-calib bash /data/run-calibration.sh
