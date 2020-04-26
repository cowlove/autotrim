#include <iostream>
#include <fstream>
#include <cmath>

#include "RunningLeastSquares.h"

RollingAverage<float, 400> avg;

using namespace std;

int main(int argc, char **argv) { 
    ifstream i(argv[1], ios_base::in);
	string line;
	int lastdir = 0;
	int lastms = 0;
	float lastAvg = 0;
	while(getline(i, line)) { 
		int dir, pulse, usec;
		float val, startval;
		sscanf(line.c_str(), "p%d %d %f %f m %d", &dir, &usec, &val, &startval, &pulse);
		avg.add(val);
		if (dir != lastdir) {
			float delta = lastAvg - avg.average();
			if (abs(delta) < 6) 
				printf("%d %f\n", lastdir ==0 ? -lastms : +lastms, lastAvg - avg.average());
			lastAvg = avg.average();
			lastms = pulse;
		}
		lastdir = dir; 
	}
	
	return 0;
}
