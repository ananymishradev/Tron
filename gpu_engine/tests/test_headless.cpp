#include "app/headless.h"
#include "test_util.h"

#include <sstream>

int main() {
  Options options;
  options.mode = Mode::Sim;
  options.particles = 1000;
  options.bodies = 0;
  options.frames = 3;
  options.json = true;
  options.scene_config.particles = 1000;
  options.scene_config.bodies = 0;
  std::ostringstream out;
  std::ostringstream err;
  CHECK(run_headless(options, out, err) == 0);
  CHECK(out.str().find("\"type\":\"summary\"") != std::string::npos);
  options.log_path = "/no/such/dir/x.csv";
  CHECK(run_headless(options, out, err) == 2);
  CHECK(err.str().find("log") != std::string::npos);
  return 0;
}
