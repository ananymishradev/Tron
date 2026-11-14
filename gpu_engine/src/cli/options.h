#pragma once

#include "sim/simulation.h"

#include <stdexcept>
#include <string>

enum class Mode { Auto, Sim, Bench, Info, Help, Version };

struct Options {
  Mode mode = Mode::Auto;
  int particles = 1'000'000;
  int bodies = 10'000;
  uint32_t seed = 42;
  int frames = 0;
  double duration = 0.0;
  int stats_hz = 10;
  float time_scale = 1.0f;
  bool bench = false;
  bool json = false;
  bool quiet = false;
  bool no_color = false;
  bool ascii = false;
  bool run_flags = false;
  bool headless = false;
  std::string scene = "sparse";
  std::string log_path;
  std::string dump_path;
  SceneConfig scene_config;
  SimParams params;
};

struct CliError : std::runtime_error {
  int exit_code;
  CliError(int code, const std::string& message)
      : std::runtime_error(message), exit_code(code) {}
};

Options parse_cli(int argc, char** argv);
Mode resolve_mode(const Options& options, bool stdin_tty, bool stdout_tty);
std::string help_text();
