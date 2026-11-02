#pragma once

#include "util/math.h"

struct Ray {
  Vec3 origin;
  Vec3 dir;
};

class Camera {
 public:
  void orbit(float dyaw, float dpitch);
  void zoom(float delta);
  Mat4 view_proj(float aspect) const;
  Ray ray(float ndc_x, float ndc_y, float aspect) const;

 private:
  Vec3 target_{0.0f, 5.0f, 0.0f};
  float yaw_ = 0.7f;
  float pitch_ = 0.35f;
  float distance_ = 30.0f;
};
