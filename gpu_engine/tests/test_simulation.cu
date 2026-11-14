#include "sim/simulation.h"
#include "test_util.h"
#include "util/cuda_check.h"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

int main() {
  SceneConfig config;
  config.particles = 1000;
  config.bodies = 0;
  config.seed = 42;
  config.init.boxMin = {-2.0f, 0.0f, -2.0f};
  config.init.boxMax = {2.0f, 5.0f, 2.0f};
  Simulation sim;
  std::string error;
  CHECK(sim.init(config, &error));
  CHECK(sim.particles().n == 1000);
  CHECK(sim.device_bytes() > 0);
  std::vector<float4> initial, reset_state;
  sim.copy_positions(&initial);
  sim.reset(42);
  sim.copy_positions(&reset_state);
  CHECK(initial.size() == reset_state.size());
  CHECK(std::memcmp(initial.data(), reset_state.data(),
                    initial.size() * sizeof(float4)) == 0);
  sim.step();
  CHECK(sim.frame() == 1);
  CHECK(sim.stats().firstNonFinite == -1);

  SceneConfig huge;
  huge.particles = 1'000'000'000;
  Simulation huge_sim;
  CHECK(!huge_sim.init(huge, &error));
  CHECK(error.find("4 GB") != std::string::npos);

  SceneConfig tiny = config;
  tiny.cell_size = 1e-4f;
  tiny.max_cells = 64;
  Simulation clamped;
  CHECK(clamped.init(tiny, &error));
  clamped.step();
  CHECK(clamped.grid().clamped);

  Simulation nan_sim;
  CHECK(nan_sim.init(config, &error));
  const float4 nan_position{NAN, 0.0f, 0.0f, 1.0f};
  CUDA_CHECK(cudaMemcpy(nan_sim.particles().posMass, &nan_position,
                        sizeof(nan_position), cudaMemcpyHostToDevice));
  nan_sim.step();
  CHECK(nan_sim.stats().firstNonFinite == 0);
  return 0;
}
