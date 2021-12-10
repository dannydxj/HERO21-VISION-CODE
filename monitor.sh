#!/bin/bash
## Author: Wang Xiaoyan on 2019.10.15
## using:
### Ubuntu 16.04: rc.local
### Ubuntu 18.04: systemd


cd /home/dji/projects/new_hero/build
sudo ./HERORM2021
count = 0
while [ true ]; do
    status=`ps -ef | grep HERORM2021 | grep -v grep | wc -l`
    if [ $status -eq 0 ]; then
        echo "HERORM2021 is not running. Restarting..."
        sudo ./HERORM2021
        count=count+1
        if [ $count -gt 3 ]; then
            make clean && make -j6
        fi
        if [ $count -gt 10 ]; then
            reboot
        fi
    fi
    sleep 3
done
