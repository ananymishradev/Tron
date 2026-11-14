#include "app/headless.h"
#include "cli/options.h"
#include "tui/app.h"
#include "util/cuda_check.h"

#include <cuda_runtime.h>

#include <iostream>
#include <unistd.h>

void print_device_info();

int main(int argc, char** argv) {
  try {
    const Options options = parse_cli(argc, argv);
    if (options.mode == Mode::Info) {
      print_device_info();
      return 0;
    }
    if (options.mode == Mode::Help) {
      std::cout << help_text();
      return 0;
    }
    if (options.mode == Mode::Version) {
      std::cout << "0.2.0\n";
      return 0;
    }
    Mode mode = resolve_mode(options, isatty(STDIN_FILENO), isatty(STDOUT_FILENO));
    if (mode == Mode::Sim || mode == Mode::Bench) {
      return run_headless(options, std::cout, std::cerr);
    }
    if (mode == Mode::Tui) return TuiApp{}.run(options);
    std::cout << help_text();
    return 0;
  } catch (const CliError& error) {
    std::cerr << "error: " << error.what() << "\n\n" << help_text();
    return error.exit_code;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
