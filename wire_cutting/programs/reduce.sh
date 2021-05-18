#!/bin/bash

filename=$(echo $1 | cut -f1 -d".")
awk 'NR<2|| ((/cut/ && NR%5==0) || (/p2p/)) {print}' $1 > "${filename}_short.txt"
