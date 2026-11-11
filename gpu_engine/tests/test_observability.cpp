#include "app/probe.h"
#include "app/stats_log.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

int main() {
  const std::vector<float4> positions = {
      {0, 0, 5, 1}, {0.1f, 0, 4, 1}, {2, 0, 5, 1}};
  const std::vector<float4> velocities = {
      {1, 0, 0, 0}, {2, 0, 0, 0}, {3, 0, 0, 0}};
  ProbeResult result{};
  if (!probe_nearest(positions, velocities, {{0, 0, 0}, {0, 0, 1}}, 1.0f,
                     &result) ||
      result.index != 0) {
    return 1;
  }
  if (probe_nearest(positions, velocities, {{0, 0, 0}, {0, 1, 0}}, 0.1f,
                    &result)) {
    return 1;
  }

  const char* path = "observability_test.csv";
  StatsLog log;
  if (!log.open(path)) {
    return 1;
  }
  SimStats stats;
  for (int frame = 0; frame < 5; ++frame) {
    log.write(frame, 1, 2, 60, 10, 2, 3, 4, stats);
  }
  log.close();
  std::ifstream input(path);
  std::string line;
  int lines = 0;
  while (std::getline(input, line)) {
    std::stringstream stream(line);
    int fields = 0;
    while (std::getline(stream, line, ',')) {
      ++fields;
    }
    if (fields != 10) {
      return 1;
    }
    ++lines;
  }
  std::remove(path);
  return lines == 6 ? 0 : 1;
}
