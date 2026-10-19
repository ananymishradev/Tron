#include <cstring>
#include <exception>
#include <iostream>

void print_device_info();

int main(int argc, char** argv) {
  if (argc > 1 && std::strcmp(argv[1], "--info") == 0) {
    try {
      print_device_info();
      return 0;
    } catch (const std::exception& error) {
      std::cerr << "device probe failed: " << error.what() << '\n';
      return 1;
    }
  }

  std::cout << "gpu_engine: use --info to print CUDA device information\n";
  return 0;
}
