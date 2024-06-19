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

bool doline(istream &i)  {
	string line;
	if (!getline(i, line)) 
		return false;

	float t; 
	int addr, dlen;
	if (sscanf(line.c_str(), " (%f) can0 %x [%d]", &t, &addr, &dlen) != 3)
		return false;

	vector<string> words = split(line.c_str(), " ");
	if (words.size() < 4 || dlen != 40)
		return false;

	for (auto i = words.begin() + 4; i != words.end(); i++) { 
		int b[8];
		float mi = 600;
		float ma = 900;
		if (sscanf(i->c_str(), "%02x%02x%02x%02x%02x%02x%02x%02x", b,b+1,b+2,b+3,b+4,b+5,b+6,b+7) == 8) { 
			float f1 = getFloat(b);
			float f2 = getFloat(b + 4);
			int idx = (int)(i - words.begin() - 4) * 8;
			if (0 && f1 >= mi && f1 <= ma) { 
				printf("got float %f at offset %d in line %s\n", f1, (int)(i - words.begin()) * 2, line.c_str());
			}

			printf("%02d: %+09.3f %02d: %+09.3f ", idx, f1, idx + 4, f2);
		}
	}
	cout << endl;
	return true;
}


int main(int argc, char **argv) { 
	while(cin) { 
		doline(cin);  
	
	}
}
