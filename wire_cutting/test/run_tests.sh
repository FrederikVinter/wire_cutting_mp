#!/bin/bash

source ../../../devel/setup.bash

declare -a planners=("joint only" "pose" "pose+vel" "pose+vel+accel")
declare -a programs=("rectangle.txt")

for i in "${planners[@]}"
do
	for j in "${programs[@]}"
	do
		roslaunch wire_cutting wire_cutting_norviz.launch testn:="$i" testp:="$j"
	done
done
