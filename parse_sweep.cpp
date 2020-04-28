#include <iostream>
#include <fstream>
#include <cmath>

#include "RunningLeastSquares.h"

///parse_sweep ./sweep6_unidirectional_battery_power.txt  | grep CR | gnuplot -e 'p [0:20][970:990] "-" u 2:3; pause 10'


RollingAverage<float, 400> avg;

using namespace std;

int main(int argc, char **argv) { 
    ifstream i(argv[1], ios_base::in);
	string line;
	int lastdir = 0;
	int lastms = 0;
	int sawpin = 0, sawtrim = 0;
	float lastAvg = 0;
	uint64_t pulseStartTime = 0;
	
	while(getline(i, line)) { 
		int dir, pulse, usec, lastusec;
		float val, startval, cutoffVal;
		if (sscanf(line.c_str(), "pin %d %d", &dir, &dir) == 2) { 
			sawpin = 1;
		} 
		if (sscanf(line.c_str(), "trim %d", &dir) == 1) { 
			sawtrim = 1;
		} 
		if (sscanf(line.c_str(), "p%d %d %f %f m %d", &dir, &usec, &val, &startval, &pulse) == 5) { 
			avg.add(val);
			if (sawtrim && pulseStartTime > 0) { 
				printf("CR %f %f\n", (usec - pulseStartTime)/1000000.0, val);
			}
			if (usec > pulse * 1000 && lastusec <= pulse * 1000) {
				cutoffVal = avg.average();
			}
			if (dir != lastdir || sawpin) {
				sawpin = 0; 
				sawtrim = 0;
				float delta = lastAvg - avg.average();
				if (abs(delta) < 15) 
					printf("PR %d %f %f\n", lastdir ==0 ? -lastms : +lastms, lastAvg - avg.average(), lastAvg - cutoffVal);
				lastAvg = avg.average();
				lastms = pulse;
				pulseStartTime = usec;
				cutoffVal = 0;
			}
			lastusec = usec;
			lastdir = dir; 
		}
	}
	
	return 0;
}
