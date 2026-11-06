#pragma once

#include "util/math.h"

struct RefRigidBody {
  Vec3 position{};
  Vec3 velocity{};
  Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  Vec3 omega{};
};
