#include "cli/options.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace {

// Keep parsing strict; silently accepting trailing junk makes shell mistakes hard to spot.
int integer_arg(const char* value, const char* name) {
  char* end = nullptr;
  const long parsed = std::strtol(value, &end, 10);
  if (*value == '\0' || end == value || *end != '\0') {
    throw CliError(2, std::string("invalid value for ") + name);
  }
  if (parsed < 0 || parsed > 2'000'000'000L) {
    throw CliError(2, std::string("value out of range for ") + name);
  }
  return static_cast<int>(parsed);
}

bool needs_value(int index, int argc, const char* option) {
  if (index + 1 >= argc) {
    throw CliError(2, std::string("missing value for ") + option);
  }
  return true;
}

}  // namespace

Options parse_cli(int argc, char** argv) {
  Options options;
  bool frames_set = false;
  bool duration_set = false;
  bool particles_set = false;
  bool bodies_set = false;
  std::string command;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "sim" || arg == "bench" || arg == "info" || arg == "help" ||
        arg == "version" || arg == "tui") {
      if (!command.empty()) throw CliError(2, "multiple commands specified");
      command = arg;
      continue;
    }
    if (arg == "--help" || arg == "-h") {
      options.mode = Mode::Help;
      continue;
    }
    if (arg == "--version") {
      options.mode = Mode::Version;
      continue;
    }
    if (arg == "--info") {
      options.mode = Mode::Info;
      continue;
    }
    // Legacy flag: selects sim mode and stays for existing scripts.
    if (arg == "--headless") {
      options.headless = true;
      options.mode = Mode::Sim;
      continue;
    }
    if (arg == "--tui") {
      options.mode = Mode::Tui;
      continue;
    }
    if (arg == "--bench") {
      options.bench = true;
      options.mode = Mode::Bench;
      options.run_flags = true;
      continue;
    }
    if (arg == "--json") {
      options.json = true;
      options.run_flags = true;
      continue;
    }
    if (arg == "--quiet") {
      options.quiet = true;
      continue;
    }
    if (arg == "--no-color") {
      options.no_color = true;
      continue;
    }
    if (arg == "--ascii") {
      options.ascii = true;
      continue;
    }
    if (arg == "--particles") {
      needs_value(i, argc, "--particles");
      options.particles = integer_arg(argv[++i], "--particles");
      particles_set = true;
      options.run_flags = true;
      continue;
    }
    if (arg == "--bodies") {
      needs_value(i, argc, "--bodies");
      options.bodies = integer_arg(argv[++i], "--bodies");
      bodies_set = true;
      options.run_flags = true;
      continue;
    }
    if (arg == "--seed") {
      needs_value(i, argc, "--seed");
      options.seed = static_cast<uint32_t>(integer_arg(argv[++i], "--seed"));
      options.run_flags = true;
      continue;
    }
    if (arg == "--frames") {
      needs_value(i, argc, "--frames");
      options.frames = integer_arg(argv[++i], "--frames");
      frames_set = true;
      options.run_flags = true;
      continue;
    }
    if (arg == "--duration") {
      needs_value(i, argc, "--duration");
      options.duration = std::strtod(argv[++i], nullptr);
      if (options.duration < 0.0) throw CliError(2, "duration must be non-negative");
      duration_set = true;
      options.run_flags = true;
      continue;
    }
    if (arg == "--stats-hz") {
      needs_value(i, argc, "--stats-hz");
      options.stats_hz = integer_arg(argv[++i], "--stats-hz");
      if (options.stats_hz < 1) throw CliError(2, "stats-hz must be positive");
      continue;
    }
    if (arg == "--log") {
      needs_value(i, argc, "--log");
      options.log_path = argv[++i];
      options.run_flags = true;
      continue;
    }
    if (arg == "--dump") {
      needs_value(i, argc, "--dump");
      options.dump_path = argv[++i];
      options.run_flags = true;
      continue;
    }
    if (arg == "--scene") {
      needs_value(i, argc, "--scene");
      options.scene = argv[++i];
      if (options.scene != "sparse" && options.scene != "dense") {
        throw CliError(2, "scene must be sparse or dense");
      }
      continue;
    }
    throw CliError(2, "unknown option: " + arg);
  }
  if (frames_set && duration_set) {
    throw CliError(2, "--frames and --duration cannot be combined");
  }
  if (command == "sim") options.mode = Mode::Sim;
  if (command == "bench") {
    options.mode = Mode::Bench;
    options.bench = true;
  }
  if (command == "info") options.mode = Mode::Info;
  if (command == "help") options.mode = Mode::Help;
  if (command == "version") options.mode = Mode::Version;
  if (command == "tui") options.mode = Mode::Tui;
  // Dense is a fixed preset; it overrides the particle and body counts.
  if (options.scene == "dense") {
    options.particles = 200'000;
    options.bodies = 0;
    if (!particles_set) options.scene_config.init.boxMin = {-5, 0, -5};
    if (!particles_set) options.scene_config.init.boxMax = {5, 10, 5};
  }
  if (duration_set) {
    options.frames = static_cast<int>(std::ceil(options.duration * 60.0));
  }
  options.scene_config.particles = options.particles;
  options.scene_config.bodies = options.bodies;
  options.scene_config.seed = options.seed;
  options.scene_config.params = options.params;
  return options;
}

Mode resolve_mode(const Options& options, bool stdin_tty, bool stdout_tty) {
  if (options.mode != Mode::Auto) return options.mode;
  if (stdin_tty && stdout_tty) return Mode::Tui;
  if (options.run_flags && (!stdin_tty || !stdout_tty)) return Mode::Sim;
  return Mode::Help;
}

std::string help_text() {
  return "Tron GPU physics engine\n\n"
         "Usage: gpu_engine [sim|bench|info|help|tui] [options]\n\n"
         "Options:\n"
         "  --particles N       particle count\n"
         "  --bodies N          rigid-body count\n"
         "  --seed N            deterministic seed\n"
         "  --frames N          number of frames\n"
         "  --duration SEC      run duration at 60 Hz\n"
         "  --scene sparse|dense\n"
         "  --bench             print stage timings\n"
         "  --json              emit JSONL telemetry\n"
         "  --log PATH          write CSV statistics\n"
         "  --dump PATH         write a density PPM\n"
         "  --info              print CUDA device information\n";
}
