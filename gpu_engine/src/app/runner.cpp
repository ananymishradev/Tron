#include "app/runner.h"

#include "app/dump.h"
#include "app/stats_log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <vector>

SimulationRunner::~SimulationRunner() { stop(); }

// Wait for init to finish so the caller gets a real error instead of a silent dead runner.
bool SimulationRunner::start(const RunnerConfig& config, std::string* error) {
  stop();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = true;
    quit_requested_ = false;
    init_done_ = false;
    init_error_.clear();
  }
  worker_ = std::thread(&SimulationRunner::worker_main, this, config);
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] { return init_done_; });
  if (!init_error_.empty()) {
    if (error) *error = init_error_;
    lock.unlock();
    stop();
    return false;
  }
  return true;
}

// Quit is queued, so a pending dump or probe still gets a chance to finish.
void SimulationRunner::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!worker_.joinable()) {
      running_ = false;
      return;
    }
    commands_.push_back({Type::Quit});
    quit_requested_ = true;
  }
  condition_.notify_all();
  worker_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
}

bool SimulationRunner::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

// Commands are cheap values; the worker drains the whole queue once per iteration.
void SimulationRunner::enqueue(Command command) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!worker_.joinable()) return;
    commands_.push_back(std::move(command));
  }
  condition_.notify_one();
}

void SimulationRunner::set_paused(bool paused) {
  Command command{Type::Pause};
  command.flag = paused;
  enqueue(std::move(command));
}
void SimulationRunner::step_once() { enqueue({Type::Step}); }
void SimulationRunner::reset(uint32_t seed) {
  Command command{Type::ResetSeed};
  command.seed = seed;
  enqueue(std::move(command));
}
void SimulationRunner::reset(const SceneConfig& scene) {
  Command command{Type::ResetScene};
  command.scene = scene;
  enqueue(std::move(command));
}
void SimulationRunner::set_params(const SimParams& params) {
  Command command{Type::SetParams};
  command.params = params;
  enqueue(std::move(command));
}
void SimulationRunner::set_time_scale(float scale) {
  Command command{Type::SetTimeScale};
  command.scale = scale;
  enqueue(std::move(command));
}
void SimulationRunner::request_probe(Ray ray, float max_distance) {
  Command command{Type::Probe};
  command.ray = ray;
  command.max_distance = max_distance;
  enqueue(std::move(command));
}
void SimulationRunner::request_view(ViewPlane plane) {
  Command command{Type::SetView};
  command.plane = plane;
  enqueue(std::move(command));
}
void SimulationRunner::request_log_path(std::string path) {
  Command command{Type::LogPath};
  command.path = std::move(path);
  enqueue(std::move(command));
}
void SimulationRunner::set_logging(bool enabled) {
  Command command{Type::Logging};
  command.flag = enabled;
  enqueue(std::move(command));
}
void SimulationRunner::request_dump(std::string path) {
  Command command{Type::Dump};
  command.path = std::move(path);
  enqueue(std::move(command));
}

// Copy under the snapshot mutex; the worker never blocks on the UI.
bool SimulationRunner::poll(Telemetry* out) {
  if (!out) return false;
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (snapshot_.sequence == last_polled_sequence_) return false;
  *out = snapshot_;
  last_polled_sequence_ = snapshot_.sequence;
  return true;
}

bool SimulationRunner::failed(std::string* error) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (init_error_.empty()) return false;
  if (error) *error = init_error_;
  return true;
}

void SimulationRunner::add_event(LogEvent::Severity severity,
                                 const std::string& message) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.events.push_back({0.0, severity, message});
  if (snapshot_.events.size() > 16) snapshot_.events.erase(snapshot_.events.begin());
}

void SimulationRunner::publish(bool force) {
  static_cast<void>(force);
}

void SimulationRunner::worker_main(RunnerConfig config) {
  Simulation simulation;
  std::string error;
  if (!simulation.init(config.scene, &error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    init_error_ = error;
    init_done_ = true;
    running_ = false;
    condition_.notify_all();
    return;
  }
  StatsLog log;
  bool paused = false;
  bool stepping = false;
  bool logging = false;
  float time_scale = 1.0f;
  ViewPlane plane = ViewPlane::TopXZ;
  auto last = std::chrono::steady_clock::now();
  const auto started = last;
  auto last_publish = last;
  double accumulator = 0.0;
  Telemetry telemetry;
  telemetry.particles = config.scene.particles;
  telemetry.bodies = config.scene.bodies;
  telemetry.log_path = config.log_path;
  if (!config.log_path.empty()) {
    logging = log.open(config.log_path.c_str());
    if (!logging) telemetry.error = "unable to open log path";
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    init_done_ = true;
    condition_.notify_all();
  }
  auto publish_snapshot = [&] {
    telemetry.sequence++;
    telemetry.elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      started)
            .count();
    telemetry.frame = simulation.frame();
    telemetry.paused = paused;
    telemetry.stepping = stepping;
    telemetry.stats = simulation.stats();
    telemetry.contacts = simulation.contacts();
    telemetry.grid_cells = simulation.grid().numCells;
    telemetry.grid_clamped = simulation.grid().clamped;
    telemetry.device_used_bytes = simulation.device_bytes();
    telemetry.particles = config.scene.particles;
    telemetry.bodies = config.scene.bodies;
    telemetry.log_rows = log.rows();
    telemetry.logging = logging;
    telemetry.plane = plane;
    if (telemetry.stats.firstNonFinite >= 0) {
      telemetry.error = "non-finite simulation state";
      add_event(LogEvent::Severity::Error, telemetry.error);
    }
    telemetry.sim_history.push(static_cast<float>(telemetry.sim_ms));
    telemetry.frame_history.push(static_cast<float>(telemetry.frame_ms));
    telemetry.fps_history.push(static_cast<float>(telemetry.fps));
    // Density follows the publish cadence; stride keeps 1M-particle scenes responsive.
    simulation.sample_density(plane, 160, 48,
                              std::max(1, config.scene.particles / 250000),
                              &telemetry.density);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = telemetry;
  };
  publish_snapshot();
  while (true) {
    std::deque<Command> commands;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (commands_.empty()) condition_.wait_for(lock, std::chrono::milliseconds(1));
      commands.swap(commands_);
    }
    bool force_publish = false;
    for (const Command& command : commands) {
      switch (command.type) {
        case Type::Pause:
          paused = command.flag;
          force_publish = true;
          break;
        case Type::Step:
          stepping = true;
          paused = true;
          force_publish = true;
          break;
        case Type::ResetSeed:
          simulation.reset(command.seed);
          paused = false;
          force_publish = true;
          break;
        case Type::ResetScene:
          if (!simulation.init(command.scene, &error)) {
            telemetry.error = error;
            add_event(LogEvent::Severity::Error, error);
          } else {
            telemetry.error.clear();
          }
          paused = false;
          force_publish = true;
          break;
        case Type::SetParams:
          simulation.set_params(command.params);
          break;
        case Type::SetTimeScale:
          time_scale = std::clamp(command.scale, 0.0f, 10.0f);
          break;
        case Type::SetView:
          plane = command.plane;
          force_publish = true;
          break;
        case Type::LogPath:
          log.close();
          logging = log.open(command.path.c_str());
          telemetry.log_path = command.path;
          telemetry.error = logging ? "" : "unable to open log path";
          force_publish = true;
          break;
        case Type::Logging:
          logging = command.flag && log.file_open();
          force_publish = true;
          break;
        case Type::Probe: {
          std::vector<float4> positions, velocities;
          simulation.copy_positions(&positions);
          simulation.copy_velocities(&velocities);
          telemetry.probe_valid =
              probe_nearest(positions, velocities, command.ray,
                            command.max_distance, &telemetry.probe);
          paused = true;
          force_publish = true;
          break;
        }
        case Type::Dump: {
          DensityGrid density;
          simulation.sample_density(plane, 320, 180,
                                    std::max(1, config.scene.particles / 250000),
                                    &density);
          if (!write_ppm(command.path.c_str(), density, 1)) {
            telemetry.error = "unable to write dump";
          }
          force_publish = true;
          break;
        }
        case Type::Quit:
          publish_snapshot();
          log.close();
          return;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::min(
        0.25, std::chrono::duration<double>(now - last).count());
    last = now;
    accumulator += dt * time_scale;
    int steps = 0;
    // At most two substeps per loop; a slow frame slows the simulation instead of spiraling.
    while ((accumulator >= config.scene.params.dt && !paused) ||
           (stepping && steps == 0)) {
      StepTimings timings{};
      simulation.step({true, true}, &timings);
      telemetry.timings = timings;
      telemetry.sim_ms = timings.total_ms;
      accumulator -= config.scene.params.dt;
      ++steps;
      stepping = false;
      if (steps >= 2) break;
    }
    if (steps > 0) {
      telemetry.substeps = steps;
    }
    telemetry.frame_ms = dt * 1000.0;
    telemetry.fps = dt > 0.0 ? 1.0 / dt : 0.0;
    if (logging && simulation.frame() > 0) {
      log.write(static_cast<int>(simulation.frame()), telemetry.sim_ms, 0.0,
                telemetry.fps, config.scene.particles, config.scene.bodies,
                simulation.contacts().pairTests, simulation.contacts().contacts,
                simulation.stats());
    }
    const auto since_publish =
        std::chrono::duration<double>(now - last_publish).count();
    if (force_publish || since_publish >= 1.0 / std::max(1, config.stats_hz)) {
      last_publish = now;
      publish_snapshot();
    }
  }
}
