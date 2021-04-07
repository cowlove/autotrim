#!/bin/bash 

# Grab KBFI ils 14R from over bainbridge island 
cat > tracksim.txt <<EOF
SPEED 100
AP 1
47.59509212379994, -122.38743386638778 1000
47.59233901597324, -122.37080179677619 500
INPUT.ALTBUG 0
INPUT.VLOCBUG 0
INPUT.MODE 5

EOF

make autotrim_ubuntu && \
./autotrim_ubuntu --serial --seconds 178.71 --tracksim ./tracksim.txt  > out.txt && tail -1 out.txt \
    | egrep '(range\s+1\s.*ils-hat\s+1\s)'
 
