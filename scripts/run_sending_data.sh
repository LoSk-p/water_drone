#!/bin/bash

date_timestamp=$(date +"%s")
date_time=$(date)
date_time=$(echo "{\"timestamp\": \""$date_timestamp"\", \"date\": \""$date_time"\"}")
date >> /home/ubuntu/log_roslaunch
echo $date_time >> /home/ubuntu/data/gps-sensors.json
sleep 10
roslaunch water_drone writing_file.launch >> /home/ubuntu/log_roslaunch
