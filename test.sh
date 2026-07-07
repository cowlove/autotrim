#!/bin/bash 

# Grab KBFI ILS 14R from over Bainbridge Island.
cat > tracksim.txt <<EOF
SPEED 100
AP 1
INPUT.ALTBUG 0
INPUT.VLOCBUG 0
INPUT.MODE 5
47.679140933777035, -122.33932295359429 1000
47.53789999126084, -122.30912681366567 50

EOF

make BOARD=csim -j2 && \
./autotrim_csim --seconds 320 --tracksim ./tracksim.txt > out.txt && \
awk '
	/Chose/ { approach=$0 }
	/TSIM/ { line=$0; mode=$5; hd=$7; trk=$17; range=$19 }
	END {
		print approach;
		print line;
		exit !(approach ~ /KBFI 14R/ && mode == 5 && hd > -0.2 && hd < 0.2 && trk > 145 && trk < 156 && range > 500 && range < 800)
	}
' out.txt
 
