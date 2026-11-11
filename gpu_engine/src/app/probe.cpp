#include "app/probe.h"

#include <cmath>
#include <limits>

bool probe_nearest(const std::vector<float4>& pos_mass,
                   const std::vector<float4>& vel, Ray ray, float max_distance,
                   ProbeResult* out) {
  if (out == nullptr) {
    return false;
  }
  *out = {};
  out->index = -1;
  float best_distance = max_distance;
  const std::size_t count = std::min(pos_mass.size(), vel.size());
  for (std::size_t i = 0; i < count; ++i) {
    const Vec3 point{pos_mass[i].x, pos_mass[i].y, pos_mass[i].z};
    const Vec3 offset = point - ray.origin;
    const float along_ray = dot(offset, ray.dir);
    if (along_ray < 0.0f) {
      continue;
    }
    // Project onto the ray and keep the closest point; a click does not need a spatial index.
    const Vec3 closest = ray.origin + ray.dir * along_ray;
    const float distance = std::sqrt(length_squared(point - closest));
    if (distance <= best_distance) {
      best_distance = distance;
      out->index = static_cast<int>(i);
      out->pos = point;
      out->vel = {vel[i].x, vel[i].y, vel[i].z};
    }
  }
  return out->index >= 0;
}
