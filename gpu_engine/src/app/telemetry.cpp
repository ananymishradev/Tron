#include "app/telemetry.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

void History::push(float value) {
  values_[head_] = value;
  head_ = (head_ + 1) % kCapacity;
  count_ = std::min(kCapacity, count_ + 1);
}

float History::at(int index) const {
  if (index < 0 || index >= count_) return 0.0f;
  const int oldest = (head_ - count_ + kCapacity) % kCapacity;
  return values_[(oldest + index) % kCapacity];
}

float History::percentile(float fraction) const {
  if (count_ == 0) return 0.0f;
  std::vector<float> copy;
  for (int i = 0; i < count_; ++i) copy.push_back(at(i));
  std::sort(copy.begin(), copy.end());
  const int index = std::clamp(static_cast<int>(fraction * (count_ - 1)), 0,
                               count_ - 1);
  return copy[index];
}

namespace {
std::string escape(const std::string& value) {
  std::string result;
  for (char c : value) {
    if (c == '"' || c == '\\') result += '\\';
    result += c;
  }
  return result;
}
}

// One JSON object per line; keep the key order stable for diffing recordings.
std::string telemetry_json(const Telemetry& t) {
  std::ostringstream out;
  out << "{\"type\":\"telemetry\",\"sequence\":" << t.sequence
      << ",\"frame\":" << t.frame << ",\"elapsed_s\":" << t.elapsed_s
      << ",\"sim_ms\":" << t.sim_ms << ",\"frame_ms\":" << t.frame_ms
      << ",\"fps\":" << t.fps << ",\"sim_hz\":" << t.sim_hz
      << ",\"substeps\":" << t.substeps << ",\"paused\":"
      << (t.paused ? "true" : "false") << ",\"particles\":" << t.particles
      << ",\"bodies\":" << t.bodies << ",\"pairs\":" << t.contacts.pairTests
      << ",\"contacts\":" << t.contacts.contacts
      << ",\"kinetic\":" << t.stats.kinetic
      << ",\"potential\":" << t.stats.potential
      << ",\"momentum\":[" << t.stats.momentum[0] << ","
      << t.stats.momentum[1] << "," << t.stats.momentum[2] << "]"
      << ",\"nan\":" << t.stats.firstNonFinite
      << ",\"grid_cells\":" << t.grid_cells
      << ",\"grid_clamped\":" << (t.grid_clamped ? "true" : "false")
      << ",\"device_bytes\":" << t.device_used_bytes
      << ",\"probe_valid\":" << (t.probe_valid ? "true" : "false")
      << ",\"log_path\":\"" << escape(t.log_path) << "\"}\n";
  return out.str();
}
