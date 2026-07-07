#pragma once

#ifdef CSIM

template <typename T>
class SDCardBufferedLog {
public:
  String currentFile = String("csim.log");
  int dropped = 0;
  int written = 0;
  int maxWaiting = 0;

  SDCardBufferedLog(const char *, int, int, int, bool, int = 0) {}
  void add(const T *, int = 0) { written++; }
  void flush() {}
};

#else

#include "/home/jim/Arduino/libraries/esp32jimlib/src/TTGO_TS.h"

#endif
