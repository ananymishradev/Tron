#pragma once

#include "render/camera.h"
#include "util/math.h"

#include <cuda_runtime_api.h>

#include <vector>

struct ProbeResult {
  int index = -1;
  Vec3 pos{};
  Vec3 vel{};
  int neighbors = 0;
};

bool probe_nearest(const std::vector<float4>& pos_mass,
                   const std::vector<float4>& vel, Ray ray, float max_distance,
                   ProbeResult* out);
