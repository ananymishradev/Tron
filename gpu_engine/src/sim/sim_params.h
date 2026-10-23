#pragma once

#include "util/math.h"

struct SimParams {
  Vec3 gravity{0.0f, -9.81f, 0.0f};
  float drag = 0.1f;
  float dt = 1.0f / 60.0f;
  float planeY = 0.0f;
  float restitution = 0.4f;
  float friction = 0.3f;
};

struct InitParams {
  Vec3 boxMin{-20.0f, 0.0f, -20.0f};
  Vec3 boxMax{20.0f, 25.0f, 20.0f};
  float radius = 0.1f;
  float massMin = 0.5f;
  float massMax = 2.0f;
  int material = 0;
};

// Host-side reduction. firstNonFinite is -1 when the state is clean.
struct SimStats {
  float kinetic = 0.0f;
  float potential = 0.0f;
  float momentum[3]{0.0f, 0.0f, 0.0f};
  int firstNonFinite = -1;
};
