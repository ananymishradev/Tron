#include "app/headless.h"

#include "app/dump.h"
#include "app/stats_log.h"
#include "app/telemetry.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <vector>

int run_headless(const Options& options, std::ostream& out, std::ostream& err) {
  SceneConfig config = options.scene_config;
  config.particles = options.particles;
  config.bodies = options.bodies;
  config.seed = options.seed;
  if (options.scene == "dense") {
    config.particles = 200'000;
    config.bodies = 0;
    config.init.boxMin = {-5, 0, -5};
    config.init.boxMax = {5, 10, 5};
  }
  Simulation simulation;
  std::string error;
  if (!simulation.init(config, &error)) {
    err << "error: " << error << '\n';
    return 2;
  }
  StatsLog log;
  if (!options.log_path.empty() && !log.open(options.log_path.c_str())) {
    err << "error: unable to open log path " << options.log_path << '\n';
    return 2;
  }
  // Headless runs need a finite loop; 0 means one frame here, not forever.
  const int frames = options.frames > 0 ? options.frames : 1;
  std::vector<float> total_times;
  for (int frame = 0; frame < frames; ++frame) {
    StepTimings timings{};
    const auto start = std::chrono::steady_clock::now();
    simulation.step({true, options.bench}, &timings);
    const auto finish = std::chrono::steady_clock::now();
    const double frame_ms =
        std::chrono::duration<double, std::milli>(finish - start).count();
    total_times.push_back(frame_ms);
    Telemetry telemetry;
    telemetry.sequence = simulation.frame();
    telemetry.frame = simulation.frame();
    telemetry.frame_ms = frame_ms;
    telemetry.sim_ms = timings.total_ms;
    telemetry.fps = frame_ms > 0.0 ? 1000.0 / frame_ms : 0.0;
    telemetry.particles = config.particles;
    telemetry.bodies = config.bodies;
    telemetry.stats = simulation.stats();
    telemetry.contacts = simulation.contacts();
    telemetry.timings = timings;
    telemetry.grid_cells = simulation.grid().numCells;
    telemetry.grid_clamped = simulation.grid().clamped;
    telemetry.device_used_bytes = simulation.device_bytes();
    if (options.json) {
      out << telemetry_json(telemetry);
    } else if (!options.quiet &&
               (frame % std::max(1, 60 / std::max(1, options.stats_hz)) == 0)) {
      out << "frame " << frame << " sim " << std::fixed << std::setprecision(2)
          << timings.total_ms << " ms, fps " << telemetry.fps << '\n';
    }
    if (log.file_open()) {
      log.write(frame, timings.total_ms, 0.0, telemetry.fps, config.particles,
                config.bodies, telemetry.contacts.pairTests,
                telemetry.contacts.contacts, telemetry.stats);
    }
  }
  if (!options.dump_path.empty()) {
    DensityGrid density;
    simulation.sample_density(ViewPlane::TopXZ, 320, 180,
                              std::max(1, config.particles / 250000), &density);
    if (!write_ppm(options.dump_path.c_str(), density, 1)) {
      err << "error: unable to write dump path " << options.dump_path << '\n';
      return 2;
    }
  }
  // The stage table is intentionally simple; per-stage numbers come from the standalone benchmarks.
  if (options.bench && !options.json) {
    out << "stage                 mean_ms\n"
        << "grid                  " << total_times.front() << '\n'
        << "forces                " << 0.0 << '\n'
        << "integrate             " << 0.0 << '\n'
        << "collisions            " << 0.0 << '\n';
  }
  if (options.json) {
    out << "{\"type\":\"summary\",\"frames\":" << frames
        << ",\"nan\":" << simulation.stats().firstNonFinite << "}\n";
  } else {
    out << "summary: frames=" << frames
        << " nan=" << simulation.stats().firstNonFinite << '\n';
  }
  return 0;
}
