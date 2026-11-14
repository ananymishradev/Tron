#pragma once

#include "sim/collisions.h"
#include "sim/rigid.h"
#include "sim/sim_params.h"

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

enum class ViewPlane { TopXZ, SideXY };

struct DensityGrid {
  int width = 0;
  int height = 0;
  ViewPlane plane = ViewPlane::TopXZ;
  float min_a = 0.0f;
  float max_a = 1.0f;
  float min_b = 0.0f;
  float max_b = 1.0f;
  std::vector<uint32_t> particles;
  std::vector<uint8_t> bodies;
  uint32_t max_count = 0;
  uint32_t* particle_device = nullptr;
  uint8_t* body_device = nullptr;
};

// Everything needed to allocate and initialize a scene. Changing this requires a reset.
struct SceneConfig {
  int particles = 1'000'000;
  int bodies = 10'000;
  uint32_t seed = 42;
  InitParams init{};
  SimParams params{};
  float cell_size = 0.25f;
  int max_cells = 1 << 20;
  int max_contacts = 200'000;
};

struct StepOptions {
  bool collect_stats = true;
  bool collect_timings = false;
};

struct StepTimings {
  float grid_ms = 0.0f;
  float forces_ms = 0.0f;
  float integrate_ms = 0.0f;
  float collisions_ms = 0.0f;
  float rigid_ms = 0.0f;
  float stats_ms = 0.0f;
  float total_ms = 0.0f;
};

constexpr std::size_t kDeviceBudgetBytes = 4ull * 1024 * 1024 * 1024;
std::size_t estimate_device_bytes(const SceneConfig& config);

// Owns all CUDA state for one scene; the runner keeps this on its worker thread.
class Simulation {
 public:
  Simulation() = default;
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  bool init(const SceneConfig& config, std::string* error);
  void reset(uint32_t seed);
  void set_params(const SimParams& params);
  void step(const StepOptions& options = {}, StepTimings* timings = nullptr);

  const ParticleSystem& particles() const { return particles_; }
  const RigidBodySoA& bodies() const { return bodies_; }
  const UniformGrid& grid() const { return grid_; }
  const SimStats& stats() const { return stats_; }
  const ContactStats& contacts() const { return contact_stats_; }
  uint64_t frame() const { return frame_; }
  std::size_t device_bytes() const { return device_bytes_; }

  void copy_positions(std::vector<float4>* out) const;
  void copy_velocities(std::vector<float4>* out) const;
  void copy_body_positions(std::vector<float4>* out) const;
  void sample_density(ViewPlane plane, int width, int height,
                      int sample_stride, DensityGrid* out);

 private:
  void release();
  SceneConfig config_{};
  ParticleSystem particles_{};
  RigidBodySoA bodies_{};
  UniformGrid grid_{};
  Contact* contacts_ = nullptr;
  int* device_contact_count_ = nullptr;
  SimStats* device_stats_ = nullptr;
  cudaStream_t stream_ = nullptr;
  SimStats stats_{};
  ContactStats contact_stats_{};
  std::size_t device_bytes_ = 0;
  uint64_t frame_ = 0;
  bool initialized_ = false;
};

