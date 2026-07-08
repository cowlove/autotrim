#!/bin/bash 

# Grab KBFI ILS 14R from over Bainbridge Island.
cat > tracksim.txt <<EOF
SPEED 100
AP 1
INPUT.ALTBUG 0
INPUT.VLOCBUG 0
INPUT.MODE 5
47.73712038529761, -122.38808930577605 1000
47.57057237566534, -122.39008597790144 50

EOF

make BOARD=csim -j2 && \
./autotrim_csim --seconds 460 --tracksim ./tracksim.txt > out.txt && \
awk '
	/Chose/ { approach=$0 }
	/TSIM/ { line=$0; mode=$5; hd=$7; trk=$17; xte=$21 }
	END {
		print approach;
		print line;

		# Confirm the starting point selected the intended built-in approach.
		approach_ok = approach ~ /KBFI 14R/
		mode_ok = mode == 5

		# After the scripted 30 degree intercept, lateral CDI should be alive
		# and nearly centered rather than still pegged at +/-2.0.
		cdi_centered = hd > -0.2 && hd < 0.2

		# KBFI 14R final approach course is about 150.5 degrees true.
		on_final_course = trk > 145 && trk < 156

		# Cross-track error is reported in meters from the simulated localizer.
		localizer_captured = xte > -10 && xte < 10

		exit !(approach_ok && mode_ok && cdi_centered && on_final_course && localizer_captured)
	}
' out.txt
 
