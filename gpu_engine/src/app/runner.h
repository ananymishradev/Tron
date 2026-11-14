#pragma once

#include "app/telemetry.h"
#include "sim/simulation.h"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

struct RunnerConfig {
  SceneConfig scene;
  int stats_hz = 10;
  std::string log_path;
};

// The worker owns CUDA and the log; public methods only enqueue commands.
class SimulationRunner {
 public:
  ~SimulationRunner();
  bool start(const RunnerConfig& config, std::string* error);
  void stop();
  bool running() const;
  void set_paused(bool paused);
  void step_once();
  void reset(uint32_t seed);
  void reset(const SceneConfig& config);
  void set_params(const SimParams& params);
  void set_time_scale(float scale);
  void request_probe(Ray ray, float max_distance);
  void request_view(ViewPlane plane);
  void request_log_path(std::string path);
  void set_logging(bool enabled);
  void request_dump(std::string path);
  bool poll(Telemetry* out);
  bool failed(std::string* error) const;

 private:
  enum class Type {
    Pause, Step, ResetSeed, ResetScene, SetParams, SetTimeScale, Probe,
    SetView, LogPath, Logging, Dump, Quit
  };
  struct Command {
    Type type;
    bool flag = false;
    uint32_t seed = 0;
    float scale = 1.0f;
    SimParams params{};
    SceneConfig scene{};
    Ray ray{};
    float max_distance = 0.0f;
    ViewPlane plane = ViewPlane::TopXZ;
    std::string path;
  };

  void enqueue(Command command);
  void worker_main(RunnerConfig config);
  void publish(bool force = false);
  void add_event(LogEvent::Severity severity, const std::string& message);

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Command> commands_;
  std::thread worker_;
  bool running_ = false;
  bool quit_requested_ = false;
  bool init_done_ = false;
  std::string init_error_;
  mutable std::mutex snapshot_mutex_;
  Telemetry snapshot_{};
  uint64_t last_polled_sequence_ = 0;
};
