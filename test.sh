#!/bin/bash 

# Grab KBFI ILS 14R from over Bainbridge Island.
cat > tracksim.txt <<EOF
SPEED 100
AP 1
INPUT.ALTBUG 0
INPUT.VLOCBUG 0
INPUT.MODE 5
47.59509212379994, -122.38743386638778 1000
47.53789999126084, -122.30912681366567 50

EOF

make BOARD=csim -j2 && \
./autotrim_csim --seconds 163.5 --tracksim ./tracksim.txt > out.txt && \
awk '/TSIM/ { line=$0; mode=$5; range=$19 } END { print line; exit !(mode == 5 && range > 0 && range < 350) }' out.txt
 
