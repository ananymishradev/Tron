#include "render/camera.h"

#include <algorithm>
#include <cmath>

namespace {

Vec3 camera_position(float yaw, float pitch, float distance, Vec3 target) {
  const float horizontal = std::cos(pitch) * distance;
  return {target.x + std::sin(yaw) * horizontal,
          target.y + std::sin(pitch) * distance,
          target.z + std::cos(yaw) * horizontal};
}

}  // namespace

void Camera::orbit(float dyaw, float dpitch) {
  yaw_ += dyaw;
  pitch_ = std::clamp(pitch_ + dpitch, -1.45f, 1.45f);
}

void Camera::zoom(float delta) {
  distance_ = std::clamp(distance_ * std::exp(-delta * 0.1f), 1.0f, 200.0f);
}

Mat4 Camera::view_proj(float aspect) const {
  const Vec3 position = camera_position(yaw_, pitch_, distance_, target_);
  return mat4_mul(mat4_perspective(1.0f, aspect, 0.1f, 500.0f),
                  mat4_look_at(position, target_, {0.0f, 1.0f, 0.0f}));
}

Ray Camera::ray(float ndc_x, float ndc_y, float aspect) const {
  const Vec3 origin = camera_position(yaw_, pitch_, distance_, target_);
  const Vec3 forward = normalize(target_ - origin);
  const Vec3 right = normalize(cross(forward, {0.0f, 1.0f, 0.0f}));
  const Vec3 up = cross(right, forward);
  const float half_fovy = 0.5f;
  const Vec3 direction =
      normalize(forward + right * (ndc_x * aspect * std::tan(half_fovy)) +
                up * (ndc_y * std::tan(half_fovy)));
  return {origin, direction};
}
