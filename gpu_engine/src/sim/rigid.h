#pragma once

#include "sim/sim_params.h"
#include "sim/grid.h"

#include <cuda_runtime.h>

#include <cstdint>

// Same SoA idea as particles; quaternions are stored xyzw in a float4.
struct RigidBodySoA {
  int n = 0;
  float4* pos = nullptr;
  float4* quat = nullptr;
  float4* vel = nullptr;
  float4* omega = nullptr;
  float4* invInertiaBody = nullptr;
  float4* halfExtents = nullptr;
  int* shape = nullptr;

  void alloc(int count);
  void free();
};

// A contact with b = -1 is a body-plane contact.
struct Contact {
  int a;
  int b;
  Vec3 point;
  Vec3 normal;
  float depth;
};

void launch_rigid_init(RigidBodySoA&, int n, uint32_t seed,
                       const InitParams&, cudaStream_t);
void launch_rigid_integrate(RigidBodySoA&, const SimParams&, cudaStream_t);
void launch_contacts_generate(const RigidBodySoA&, const UniformGrid&,
                              Contact*, int* count, int max_contacts,
                              cudaStream_t);
void launch_contacts_solve(RigidBodySoA&, Contact*, int count, int iterations,
                           const SimParams&, cudaStream_t);
