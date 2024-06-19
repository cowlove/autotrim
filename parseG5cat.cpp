#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

// trim from start
inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

vector<string> split(const char *line, const char *delim) {
  char *buf = strdup(line); 
  std::vector<string> rval;
  for(char *w = strtok(buf, delim); w != NULL; w = strtok(NULL, delim)) {
    string ws(w);
    trim(ws);
    rval.push_back(ws);
  }
  free(buf);
  return rval;
}

float getFloat(int *b) {
	int32_t x = b[0] + (b[1] << 8) + (b[2] << 16) +(b[3] << 24);
	float f = *((float *)(&x));
	return f;
}

static float mi = 500;
static float ma = 1000;
static float lastInteresting = 0;

bool doline(istream &i)  {
	string line;
	if (!getline(i, line)) 
		return false;

	float t; 
	int addr, dlen;
	if (sscanf(line.c_str(), " (%f) can0 %x [%d]", &t, &addr, &dlen) != 3)
		return false;

	vector<string> words = split(line.c_str(), " ");
	if (words.size() < 4)
		return false;

	float flts[128];
	uint8_t bytes[512];
	bool first = true;
	bool interesting = false;
	for (int i = 0; i < words.size() - 4; i++) { 
		int b[8];
		if (sscanf(words[i + 4].c_str(), "%02x%02x%02x%02x%02x%02x%02x%02x", b,b+1,b+2,b+3,b+4,b+5,b+6,b+7) == 8) { 
			if (first) { 
				printf("%8.3f %02d ", t, dlen);
			}
			float f1 = getFloat(b);
			float f2 = getFloat(b + 4);
			flts[i * 2] = f1;
			flts[i * 2 + 1] = f2;
			memcpy(bytes + i * 8, b, 8);
		}
		first = false;
	}
	for (int i = 0; i < (words.size() - 4) * 2; i++) {
			printf("%02d:%+09.3f%c ", i, flts[i], (flts[i] >= mi && flts[i] <= ma) ? 'X' : ' ');
			if (flts[i] >= mi && flts[i] <= ma)
				interesting = true;
	}

	if (bytes[0] == 0xdd && bytes[1] == 0x0a && dlen == 40) {
		mi = flts[7] - 2;
		ma = flts[7] + 2;
	}
	cout << endl;
	if (true && interesting) {
		printf("XXX %08.3f", t - lastInteresting);
		lastInteresting = t;
		cout << line << endl;
	}
	return true;
}


int main(int argc, char **argv) { 
	while(cin) { 
		doline(cin);  
	
	}
}
