#!/bin/bash

source ../../../devel/setup.bash

#declare -a planners=("j" "j_d" "jp" "jp_d" "jpv" "jpv_d" "jpva" "jpva_d")
declare -a planners=("d_j" "d_jp" "d_jpv" "d_jpva")
declare -a programs=("rectangle.txt" "cone.txt" "saddel.txt")

for i in "${planners[@]}"
do
	for j in "${programs[@]}"
	do
		roslaunch wire_cutting wire_cutting_norviz.launch testn:="$i" testp:="$j"
	done
done
