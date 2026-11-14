#include "app/dump.h"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

int main() {
  DensityGrid density;
  density.width = 4;
  density.height = 2;
  density.particles = {0, 1, 0, 0, 0, 0, 0, 0};
  density.bodies = {0, 0, 1, 0, 0, 0, 0, 0};
  density.max_count = 1;
  const char* path = "dump_test.ppm";
  if (!write_ppm(path, density, 1)) return 1;
  std::ifstream file(path, std::ios::binary);
  const std::string bytes((std::istreambuf_iterator<char>(file)), {});
  const std::string header = "P6\n4 2\n255\n";
  if (bytes.substr(0, header.size()) != header ||
      bytes.size() != header.size() + 4 * 2 * 3) {
    std::remove(path);
    return 1;
  }
  std::remove(path);
  return 0;
}
