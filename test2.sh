#!/bin/bash 

# Test a completely synthetic ILS
cat > tracksim.txt <<EOF
SPEED 100
AP 1
INPUT.ALTBUG 100
INPUT.VLOCBUG 140 
47.70813066084988, -122.36369934517762 1000 
47.6998032598224, -122.36380732512066 1000
INPUT.MODE 5
47.54158263617579, -122.36577368233236 1000 

EOF

make BOARD=csim -j2 && \
./autotrim_csim --seconds 120 --tracksim ./tracksim.txt > out.txt && \
awk '
	/Started ILS/ { approach=$0 }
	/TSIM/ { line=$0; mode=$5; hd=$7; trk=$17; xte=$21 }
	END {
		print approach;
		print line;

		# Confirm mode 5 created a synthetic/fake ILS instead of selecting
		# a built-in airport approach.
		approach_ok = approach ~ /Started ILS/ && approach !~ /KBFI/
		mode_ok = mode == 5

		# After the scripted intercept, lateral CDI should be alive
		# and nearly centered rather than still pegged at +/-2.0.
		cdi_centered = hd > -0.2 && hd < 0.2

		# The synthetic final approach course is OBS 140 magnetic, about
		# 155.5 degrees true at this location.
		on_final_course = trk > 150 && trk < 161

		# Cross-track error is reported in meters from the simulated localizer.
		localizer_captured = xte > -10 && xte < 10

		exit !(approach_ok && mode_ok && cdi_centered && on_final_course && localizer_captured)
	}
' out.txt
 
