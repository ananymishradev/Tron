#include "sim/simulation.h"

#include "sim/density.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <sstream>

namespace {

bool valid_config(const SceneConfig& config, std::string* error) {
  if (config.particles < 0 || config.bodies < 0 ||
      config.particles + config.bodies <= 0) {
    if (error) *error = "scene must contain at least one non-negative object";
    return false;
  }
  if (!(config.cell_size > 0.0f) || config.max_cells < 64 ||
      config.max_contacts < 1) {
    if (error) *error = "invalid grid or contact configuration";
    return false;
  }
  return true;
}

void record_begin(cudaEvent_t event, bool enabled, cudaStream_t stream) {
  if (enabled) CUDA_CHECK(cudaEventRecord(event, stream));
}

float record_end(cudaEvent_t begin, cudaEvent_t end, bool enabled,
                 cudaStream_t stream) {
  if (!enabled) return 0.0f;
  CUDA_CHECK(cudaEventRecord(end, stream));
  CUDA_CHECK(cudaEventSynchronize(end));
  float result = 0.0f;
  CUDA_CHECK(cudaEventElapsedTime(&result, begin, end));
  return result;
}

}  // namespace

// Conservative estimate used before any cudaMalloc; keep it in sync with the allocations below.
std::size_t estimate_device_bytes(const SceneConfig& config) {
  const std::uint64_t particles = static_cast<std::uint64_t>(
      std::max(0, config.particles));
  const std::uint64_t bodies =
      static_cast<std::uint64_t>(std::max(0, config.bodies));
  const std::uint64_t cells =
      static_cast<std::uint64_t>(std::max(0, config.max_cells));
  const std::uint64_t contacts =
      static_cast<std::uint64_t>(std::max(0, config.max_contacts));
  return static_cast<std::size_t>(
      particles * 84ull + bodies * 100ull + cells * 16ull + particles * 4ull +
      cells * 8ull + contacts * sizeof(Contact) + 4096ull);
}

Simulation::~Simulation() { release(); }

void Simulation::release() {
  if (stream_ != nullptr) {
    cudaStreamSynchronize(stream_);
  }
  cudaFree(device_contact_count_);
  cudaFree(device_stats_);
  cudaFree(contacts_);
  device_contact_count_ = nullptr;
  device_stats_ = nullptr;
  contacts_ = nullptr;
  grid_.free();
  bodies_.free();
  particles_.free();
  if (stream_ != nullptr) cudaStreamDestroy(stream_);
  stream_ = nullptr;
  initialized_ = false;
  frame_ = 0;
  device_bytes_ = 0;
}

bool Simulation::init(const SceneConfig& config, std::string* error) {
  release();
  if (!valid_config(config, error)) return false;
  const std::size_t estimate = estimate_device_bytes(config);
  // Fail before touching the device so a bad CLI value cannot half-initialize the scene.
  if (estimate > kDeviceBudgetBytes) {
    if (error) {
      *error = "estimated allocation exceeds 4 GB device budget";
    }
    return false;
  }
  std::size_t free_bytes = 0;
  std::size_t total_bytes = 0;
  cudaError_t memory_error = cudaMemGetInfo(&free_bytes, &total_bytes);
  if (memory_error != cudaSuccess) {
    if (error) *error = cudaGetErrorString(memory_error);
    return false;
  }
  if (free_bytes < estimate) {
    if (error) {
      std::ostringstream stream;
      stream << "insufficient device memory: " << free_bytes / (1024 * 1024)
             << " MiB available";
      *error = stream.str();
    }
    return false;
  }
  config_ = config;
  CUDA_CHECK(cudaStreamCreate(&stream_));
  particles_.alloc(config.particles);
  if (config.bodies > 0) bodies_.alloc(config.bodies);
  CUDA_CHECK(cudaMalloc(&device_stats_, sizeof(SimStats)));
  CUDA_CHECK(cudaMalloc(&contacts_, sizeof(Contact) * config.max_contacts));
  CUDA_CHECK(cudaMalloc(&device_contact_count_, sizeof(int)));
  launch_particles_init(particles_, config.particles, config.seed, config.init,
                        stream_);
  if (config.bodies > 0) {
    launch_rigid_init(bodies_, config.bodies, config.seed, config.init, stream_);
  }
  CUDA_CHECK(cudaStreamSynchronize(stream_));
  initialized_ = true;
  device_bytes_ = estimate;
  stats_ = {};
  stats_.firstNonFinite = -1;
  frame_ = 0;
  return true;
}

void Simulation::reset(uint32_t seed) {
  config_.seed = seed;
  launch_particles_init(particles_, config_.particles, seed, config_.init, stream_);
  if (bodies_.n > 0) {
    launch_rigid_init(bodies_, config_.bodies, seed, config_.init, stream_);
  }
  CUDA_CHECK(cudaStreamSynchronize(stream_));
  stats_ = {};
  stats_.firstNonFinite = -1;
  contact_stats_ = {};
  frame_ = 0;
}

void Simulation::set_params(const SimParams& params) { config_.params = params; }

void Simulation::step(const StepOptions& options, StepTimings* timings) {
  if (!initialized_) return;
  const bool timed = options.collect_timings && timings != nullptr;
  if (timed) *timings = {};
  cudaEvent_t total_begin{}, total_end{}, begin{}, end{};
  if (timed) {
    CUDA_CHECK(cudaEventCreate(&total_begin));
    CUDA_CHECK(cudaEventCreate(&total_end));
    CUDA_CHECK(cudaEventCreate(&begin));
    CUDA_CHECK(cudaEventCreate(&end));
    CUDA_CHECK(cudaEventRecord(total_begin, stream_));
  }
  // Snapshot the state before integration; collisions need the pre-step positions.
  CUDA_CHECK(cudaMemcpyAsync(particles_.posPrev, particles_.posMass,
                             sizeof(float4) * particles_.n,
                             cudaMemcpyDeviceToDevice, stream_));
  CUDA_CHECK(cudaMemcpyAsync(particles_.velPrev, particles_.vel,
                             sizeof(float4) * particles_.n,
                             cudaMemcpyDeviceToDevice, stream_));

  record_begin(begin, timed, stream_);
  launch_grid_setup(grid_, particles_.posPrev, particles_.n, config_.cell_size,
                    config_.max_cells, stream_);
  launch_grid_build(particles_.posPrev, particles_.n, grid_, stream_);
  if (timed) timings->grid_ms = record_end(begin, end, true, stream_);

  record_begin(begin, timed, stream_);
  launch_forces(particles_, config_.params, stream_);
  if (timed) timings->forces_ms = record_end(begin, end, true, stream_);

  record_begin(begin, timed, stream_);
  launch_integrate(particles_, config_.params, stream_);
  if (timed) timings->integrate_ms = record_end(begin, end, true, stream_);

  record_begin(begin, timed, stream_);
  launch_collisions(particles_, grid_, config_.params, &contact_stats_, stream_);
  if (timed) timings->collisions_ms = record_end(begin, end, true, stream_);

  record_begin(begin, timed, stream_);
  if (bodies_.n > 0) {
    launch_rigid_integrate(bodies_, config_.params, stream_);
    launch_contacts_generate(bodies_, grid_, contacts_, device_contact_count_,
                             config_.max_contacts, stream_);
    int contact_count = 0;
    CUDA_CHECK(cudaMemcpyAsync(&contact_count, device_contact_count_, sizeof(int),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    contact_count = std::min(contact_count, config_.max_contacts);
    if (contact_count > 0) {
      launch_contacts_solve(bodies_, contacts_, contact_count, 4,
                            config_.params, stream_);
    }
  }
  if (timed) timings->rigid_ms = record_end(begin, end, true, stream_);

  record_begin(begin, timed, stream_);
  // Stats are optional because the reduction is O(N); callers that only need motion can skip it.
  if (options.collect_stats) {
    launch_stats(particles_, config_.params, device_stats_, stream_);
    CUDA_CHECK(cudaMemcpyAsync(&stats_, device_stats_, sizeof(SimStats),
                               cudaMemcpyDeviceToHost, stream_));
  }
  CUDA_CHECK(cudaStreamSynchronize(stream_));
  if (timed) timings->stats_ms = record_end(begin, end, true, stream_);
  if (timed) {
    CUDA_CHECK(cudaEventRecord(total_end, stream_));
    CUDA_CHECK(cudaEventSynchronize(total_end));
    CUDA_CHECK(cudaEventElapsedTime(&timings->total_ms, total_begin, total_end));
    cudaEventDestroy(total_begin);
    cudaEventDestroy(total_end);
    cudaEventDestroy(begin);
    cudaEventDestroy(end);
  }
  ++frame_;
}

void Simulation::copy_positions(std::vector<float4>* out) const {
  if (!out) return;
  out->resize(particles_.n);
  CUDA_CHECK(cudaMemcpy(out->data(), particles_.posMass,
                        sizeof(float4) * particles_.n,
                        cudaMemcpyDeviceToHost));
}

void Simulation::copy_velocities(std::vector<float4>* out) const {
  if (!out) return;
  out->resize(particles_.n);
  CUDA_CHECK(cudaMemcpy(out->data(), particles_.vel, sizeof(float4) * particles_.n,
                        cudaMemcpyDeviceToHost));
}

void Simulation::copy_body_positions(std::vector<float4>* out) const {
  if (!out) return;
  out->resize(bodies_.n);
  if (bodies_.n > 0) {
    CUDA_CHECK(cudaMemcpy(out->data(), bodies_.pos, sizeof(float4) * bodies_.n,
                          cudaMemcpyDeviceToHost));
  }
}

void Simulation::sample_density(ViewPlane plane, int width, int height,
                                int sample_stride, DensityGrid* out) {
  if (!out) return;
  out->width = std::clamp(width, 1, 256);
  out->height = std::clamp(height, 1, 64);
  out->plane = plane;
  out->min_a = grid_.numCells > 0 ? grid_.min.x : config_.init.boxMin.x;
  out->max_a = grid_.numCells > 0 ? grid_.max.x : config_.init.boxMax.x;
  if (plane == ViewPlane::TopXZ) {
    out->min_b = grid_.numCells > 0 ? grid_.min.z : config_.init.boxMin.z;
    out->max_b = grid_.numCells > 0 ? grid_.max.z : config_.init.boxMax.z;
  } else {
    out->min_b = grid_.numCells > 0 ? grid_.min.y : config_.init.boxMin.y;
    out->max_b = grid_.numCells > 0 ? grid_.max.y : config_.init.boxMax.y;
  }
  launch_density(particles_, bodies_, plane, *out, sample_stride, stream_);
}
