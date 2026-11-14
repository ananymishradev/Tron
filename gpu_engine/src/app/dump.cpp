#include "app/dump.h"

#include <algorithm>
#include <cmath>
#include <fstream>

bool write_ppm(const char* path, const DensityGrid& density, int scale) {
  if (!path || density.width <= 0 || density.height <= 0 || scale <= 0) {
    return false;
  }
  const int width = density.width * scale;
  const int height = density.height * scale;
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file << "P6\n" << width << " " << height << "\n255\n";
  // Log scaling keeps a few dense cells from washing out the rest of the image.
  const float denominator =
      std::log1p(static_cast<float>(std::max(1u, density.max_count)));
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const int cell_x = x / scale;
      const int cell_y = y / scale;
      const int index = cell_y * density.width + cell_x;
      unsigned char value = 0;
      if (index < static_cast<int>(density.bodies.size()) &&
          density.bodies[index]) {
        value = 255;
      } else if (index < static_cast<int>(density.particles.size())) {
        const float normalized =
            std::log1p(static_cast<float>(density.particles[index])) / denominator;
        value = static_cast<unsigned char>(std::clamp(normalized, 0.0f, 1.0f) * 255);
      }
      const unsigned char pixel[3] = {value, value, value};
      file.write(reinterpret_cast<const char*>(pixel), 3);
    }
  }
  return file.good();
}
