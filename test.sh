#!/bin/bash -x

make autotrim_ubuntu
rm [123].png out.jpg

# Virtual ILS at 140deg, approaching from the left on a track of 100
cat > tracksim.txt <<EOF
SPEED=100
47.59509212379994, -122.38743386638778 2000 100
KNOB=4,2.44
KNOB=2,61
MODE=5
AP=1
EOF
./autotrim_ubuntu --serial --seconds 1000 --tracksim ./tracksim.txt  | grep CDI: > out.txt  && \
gnuplot -e 'f="./out.txt"; set term png; set output "1.png"; set y2tic; set ytic nomir; p [1:*] f u 1:6 w l, f u 1:14 ax x1y2 w l ;'


# Virtual ILS at 140deg, approaching from the right on heading of 180  
cat > tracksim.txt <<EOF
SPEED=100
47.59509212379994, -122.38743386638778 2000 180
KNOB=4,2.44
KNOB=2,61
MODE=5
AP=1
EOF
./autotrim_ubuntu --serial --seconds 1000 --tracksim ./tracksim.txt  | grep CDI: > out.txt  && \
gnuplot -e 'f="./out.txt"; set term png; set output "2.png"; set y2tic; set ytic nomir; p [1:*] f u 1:6 w l, f u 1:14 ax x1y2 w l ;'

# Grab KBFI ils 14R from over bainbridge island 
cat > tracksim.txt <<EOF
SPEED=100
AP=1
47.59509212379994, -122.38743386638778 1000
47.59233901597324, -122.37080179677619 500
KNOB=4,0.0
MODE=5
EOF
./autotrim_ubuntu --serial --seconds 1000 --tracksim ./tracksim.txt  | grep CDI: > out.txt  && \
gnuplot -e 'f="./out.txt"; set term png; set output "3.png"; set y2tic; set ytic nomir; p [1:*] f u 1:6 w l, f u 1:14 ax x1y2 w l ;'


# Graphs should all show heading converging on 140 and altitude decending to ground 
montage *.png -geometry 600x600 out.jpg; feh out.jpg 
