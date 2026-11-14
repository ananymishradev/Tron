#pragma once

#include "app/probe.h"
#include "sim/simulation.h"

#include <array>
#include <string>
#include <vector>

struct LogEvent {
  double seconds = 0.0;
  enum class Severity { Info, Warn, Error } severity = Severity::Info;
  std::string message;
};

// Fixed-size ring buffer; percentiles copy and sort, which is fine at UI rates.
class History {
 public:
  static constexpr int kCapacity = 240;
  void push(float value);
  int count() const { return count_; }
  float at(int index) const;
  float percentile(float fraction) const;
  const std::array<float, kCapacity>& values() const { return values_; }
  int head() const { return head_; }

 private:
  std::array<float, kCapacity> values_{};
  int head_ = 0;
  int count_ = 0;
};

// Immutable snapshot published by the worker; the UI only reads it.
struct Telemetry {
  uint64_t sequence = 0;
  uint64_t frame = 0;
  double elapsed_s = 0.0;
  double sim_ms = 0.0;
  double frame_ms = 0.0;
  double fps = 0.0;
  double sim_hz = 0.0;
  int substeps = 0;
  bool paused = false;
  bool stepping = false;
  bool grid_clamped = false;
  int grid_cells = 0;
  int grid_dim_x = 0;
  int grid_dim_y = 0;
  int grid_dim_z = 0;
  int particles = 0;
  int bodies = 0;
  SimStats stats{};
  ContactStats contacts{};
  StepTimings timings{};
  std::size_t device_used_bytes = 0;
  std::size_t device_total_bytes = 0;
  ProbeResult probe{};
  bool probe_valid = false;
  bool probe_pending = false;
  bool logging = false;
  int log_rows = 0;
  std::string log_path;
  std::string error;
  History sim_history;
  History frame_history;
  History fps_history;
  DensityGrid density;
  ViewPlane plane = ViewPlane::TopXZ;
  std::vector<LogEvent> events;
};

std::string telemetry_json(const Telemetry& telemetry);
