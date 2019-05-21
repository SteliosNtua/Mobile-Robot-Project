#!/bin/bash

topics=`rostopic list | grep 'sonar\|log'`
n=`ls bag_logs | wc -l`
echo "creating log$n folder"
mkdir bag_logs/log$n
echo "Logging for ${topics[@]}"

for t in ${topics[@]}; do
	rosbag record -O bag_logs/log$n$t $t &
done

read -p "Press enter to stop recording"
echo "killing recording processes"
kill -SIGINT `pidof record`

for t in ${topics[@]}; do
	rostopic echo -b bag_logs/log$n$t.bag -p $t > bag_logs/log$n$t.csv
done
