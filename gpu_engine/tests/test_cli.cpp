#include "cli/options.h"
#include "test_util.h"

#include <initializer_list>
#include <vector>

static Options parse(std::initializer_list<const char*> args) {
  std::vector<const char*> argv{"gpu_engine"};
  for (const char* arg : args) argv.push_back(arg);
  return parse_cli(static_cast<int>(argv.size()),
                   const_cast<char**>(argv.data()));
}

int main() {
  Options defaults = parse({});
  CHECK(defaults.mode == Mode::Auto);
  CHECK(defaults.particles == 1'000'000);
  CHECK(defaults.bodies == 10'000);
  CHECK(defaults.seed == 42);
  Options sim = parse({"--particles", "1000", "--bodies", "10", "--seed", "7",
                       "--frames", "30", "--headless"});
  CHECK(sim.mode == Mode::Sim);
  CHECK(sim.particles == 1000);
  CHECK(sim.bodies == 10);
  CHECK(sim.seed == 7);
  CHECK(sim.frames == 30);
  Options dense = parse({"--scene", "dense"});
  CHECK(dense.particles == 200000);
  CHECK(dense.bodies == 0);
  Options duration = parse({"--duration", "2"});
  CHECK(duration.frames == 120);
  bool threw = false;
  try { parse({"--particles", "-1"}); } catch (const CliError&) { threw = true; }
  CHECK(threw);
  threw = false;
  try { parse({"--frames", "10", "--duration", "1"}); }
  catch (const CliError&) { threw = true; }
  CHECK(threw);
  CHECK(resolve_mode(defaults, true, true) == Mode::Tui);
  CHECK(parse({"tui"}).mode == Mode::Tui);
  CHECK(resolve_mode(sim, false, false) == Mode::Sim);
  CHECK(help_text().find("--particles") != std::string::npos);
  return 0;
}
