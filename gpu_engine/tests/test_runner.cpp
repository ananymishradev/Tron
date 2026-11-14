#include "app/runner.h"

#include <cassert>
#include <chrono>
#include <thread>

int main() {
  RunnerConfig config;
  config.scene.particles = 256;
  config.scene.bodies = 0;
  config.scene.max_cells = 1 << 16;
  config.stats_hz = 30;

  SimulationRunner runner;
  std::string error;
  assert(runner.start(config, &error));
  assert(runner.running());

  Telemetry telemetry;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline &&
         !runner.poll(&telemetry)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(telemetry.sequence > 0);
  runner.set_paused(true);
  runner.step_once();
  runner.set_time_scale(0.5f);
  runner.request_view(ViewPlane::SideXY);
  runner.stop();
  runner.stop();
  assert(!runner.running());
  return 0;
}
