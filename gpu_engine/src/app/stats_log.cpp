#include "app/stats_log.h"

StatsLog::~StatsLog() { close(); }

bool StatsLog::open(const char* path) {
  close();
  file_ = std::fopen(path, "w");
  if (file_ == nullptr) {
    return false;
  }
  std::fprintf(file_,
               "frame,sim_ms,render_ms,fps,particles,bodies,pairs,contacts,"
               "kinetic,potential\n");
  return true;
}

void StatsLog::write(int frame, double sim_ms, double render_ms, double fps,
                     int particles, int bodies, unsigned pairs,
                     unsigned contacts, const SimStats& stats) {
  if (file_ == nullptr) {
    return;
  }
  std::fprintf(file_, "%d,%.9g,%.9g,%.9g,%d,%d,%u,%u,%.9g,%.9g\n", frame,
               sim_ms, render_ms, fps, particles, bodies, pairs, contacts,
               stats.kinetic, stats.potential);
  // Flush about once per second at 60 Hz so a crash does not lose the whole run.
  if (++rows_since_flush_ >= 60) {
    std::fflush(file_);
    rows_since_flush_ = 0;
  }
}

void StatsLog::close() {
  if (file_ != nullptr) {
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
  }
}
