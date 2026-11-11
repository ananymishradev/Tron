#pragma once

#include "sim/sim_params.h"

#include <cstdio>

// CSV writer for offline plots; the runner owns it on the worker thread.
class StatsLog {
 public:
  ~StatsLog();
  bool open(const char* path);
  void write(int frame, double sim_ms, double render_ms, double fps,
             int particles, int bodies, unsigned pairs, unsigned contacts,
             const SimStats&);
  void close();

 private:
  std::FILE* file_ = nullptr;
  int rows_since_flush_ = 0;
};
