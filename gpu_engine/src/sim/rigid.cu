#include "sim/rigid.h"

#include "sim/grid.h"
#include "util/cuda_check.h"
#include "util/math.h"

#include <cuda_runtime.h>

namespace {

__device__ uint64_t rigid_hash(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

__device__ float rigid_random(uint32_t seed, int index, int stream) {
  return static_cast<float>((rigid_hash(seed + index * 17 + stream * 101) >> 40) &
                            0xFFFFFF) /
         16777216.0f;
}

__global__ void rigid_init_kernel(float4* pos, float4* quat, float4* vel,
                                  float4* omega, float4* inertia,
                                  float4* extents, int* shape, int count,
                                  uint32_t seed, InitParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const bool box = (index & 1) != 0;
  const float radius = params.radius;
  const float x = params.boxMin.x + rigid_random(seed, index, 0) *
                                      (params.boxMax.x - params.boxMin.x);
  const float y = params.boxMin.y + radius +
                  rigid_random(seed, index, 1) *
                      (params.boxMax.y - params.boxMin.y - radius);
  const float z = params.boxMin.z + rigid_random(seed, index, 2) *
                                      (params.boxMax.z - params.boxMin.z);
  pos[index] = {x, y, z, 1.0f};
  quat[index] = {1.0f, 0.0f, 0.0f, 0.0f};
  vel[index] = {0.0f, 0.0f, 0.0f, 0.0f};
  omega[index] = {0.0f, 0.0f, 0.0f, 0.0f};
  extents[index] = {box ? radius : 0.0f, box ? radius : 0.0f,
                    box ? radius : 0.0f, radius};
  inertia[index] = {1.0f, 1.0f, 1.0f, 0.0f};
  shape[index] = box ? 1 : 0;
}

__global__ void rigid_integrate_kernel(float4* pos, float4* quat, float4* vel,
                                       float4* omega, int count,
                                       SimParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count || pos[index].w <= 0.0f) {
    return;
  }
  vel[index].x += params.gravity.x * params.dt;
  vel[index].y += params.gravity.y * params.dt;
  vel[index].z += params.gravity.z * params.dt;
  pos[index].x += vel[index].x * params.dt;
  pos[index].y += vel[index].y * params.dt;
  pos[index].z += vel[index].z * params.dt;
  const Quat q = {quat[index].w, quat[index].x, quat[index].y, quat[index].z};
  const Quat angular{0.0f, omega[index].x, omega[index].y, omega[index].z};
  // Quaternion derivative: 0.5 * omega * q, then renormalize to fight drift.
  const Quat derivative = quat_mul(angular, q);
  const Quat integrated = quat_normalize(
      {q.w + 0.5f * params.dt * derivative.w,
       q.x + 0.5f * params.dt * derivative.x,
       q.y + 0.5f * params.dt * derivative.y,
       q.z + 0.5f * params.dt * derivative.z});
  quat[index] = {integrated.w, integrated.x, integrated.y, integrated.z};
}

__global__ void contacts_generate_kernel(const RigidBodySoA bodies, Contact* contacts,
                                         int* count, int max_contacts) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= bodies.n || bodies.pos[index].w <= 0.0f) {
    return;
  }
  const float radius = bodies.halfExtents[index].w;
  if (bodies.pos[index].y - radius <= 0.0f) {
    // Reserve first; dropping overflow contacts is better than writing past the buffer.
    const int slot = atomicAdd(count, 1);
    if (slot < max_contacts) {
      contacts[slot] = {index, -1,
                        {bodies.pos[index].x, 0.0f, bodies.pos[index].z},
                        {0.0f, 1.0f, 0.0f}, radius - bodies.pos[index].y};
    }
  }
}

__global__ void contact_solve_kernel(RigidBodySoA bodies, const Contact* contacts,
                                     int count, SimParams params) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const Contact contact = contacts[index];
  if (contact.a < 0 || contact.a >= bodies.n) {
    return;
  }
  const float inverse_mass = bodies.pos[contact.a].w;
  if (inverse_mass <= 0.0f) {
    return;
  }
  const float depth = fmaxf(0.0f, contact.depth);
  bodies.pos[contact.a].x += contact.normal.x * depth;
  bodies.pos[contact.a].y += contact.normal.y * depth;
  bodies.pos[contact.a].z += contact.normal.z * depth;
  const float normal_velocity = bodies.vel[contact.a].x * contact.normal.x +
                                bodies.vel[contact.a].y * contact.normal.y +
                                bodies.vel[contact.a].z * contact.normal.z;
  if (normal_velocity < 0.0f) {
    const float impulse = -(1.0f + params.restitution) * normal_velocity;
    bodies.vel[contact.a].x += contact.normal.x * impulse;
    bodies.vel[contact.a].y += contact.normal.y * impulse;
    bodies.vel[contact.a].z += contact.normal.z * impulse;
  }
}

}  // namespace

void RigidBodySoA::alloc(int count) {
  free();
  n = count;
  if (count <= 0) {
    return;
  }
  CUDA_CHECK(cudaMalloc(&pos, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&quat, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&vel, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&omega, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&invInertiaBody, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&halfExtents, sizeof(float4) * count));
  CUDA_CHECK(cudaMalloc(&shape, sizeof(int) * count));
}

void RigidBodySoA::free() {
  cudaFree(pos);
  cudaFree(quat);
  cudaFree(vel);
  cudaFree(omega);
  cudaFree(invInertiaBody);
  cudaFree(halfExtents);
  cudaFree(shape);
  n = 0;
  pos = nullptr;
  quat = nullptr;
  vel = nullptr;
  omega = nullptr;
  invInertiaBody = nullptr;
  halfExtents = nullptr;
  shape = nullptr;
}

void launch_rigid_init(RigidBodySoA& bodies, int n, uint32_t seed,
                       const InitParams& params, cudaStream_t stream) {
  if (bodies.n != n) {
    bodies.alloc(n);
  }
  rigid_init_kernel<<<(n + 255) / 256, 256, 0, stream>>>(
      bodies.pos, bodies.quat, bodies.vel, bodies.omega, bodies.invInertiaBody,
      bodies.halfExtents, bodies.shape, n, seed, params);
  CUDA_CHECK(cudaGetLastError());
}

void launch_rigid_integrate(RigidBodySoA& bodies, const SimParams& params,
                            cudaStream_t stream) {
  rigid_integrate_kernel<<<(bodies.n + 255) / 256, 256, 0, stream>>>(
      bodies.pos, bodies.quat, bodies.vel, bodies.omega, bodies.n, params);
  CUDA_CHECK(cudaGetLastError());
}

void launch_contacts_generate(const RigidBodySoA& bodies, const UniformGrid&,
                              Contact* contacts, int* count, int max_contacts,
                              cudaStream_t stream) {
  CUDA_CHECK(cudaMemsetAsync(count, 0, sizeof(int), stream));
  contacts_generate_kernel<<<(bodies.n + 255) / 256, 256, 0, stream>>>(
      bodies, contacts, count, max_contacts);
  CUDA_CHECK(cudaGetLastError());
}

void launch_contacts_solve(RigidBodySoA& bodies, Contact* contacts, int count,
                           int iterations, const SimParams& params,
                           cudaStream_t stream) {
  if (count <= 0 || iterations <= 0) {
    return;
  }
  // Jacobi passes: read the same state, then apply all corrections together.
  for (int iteration = 0; iteration < iterations; ++iteration) {
    contact_solve_kernel<<<(count + 255) / 256, 256, 0, stream>>>(
        bodies, contacts, count, params);
    CUDA_CHECK(cudaGetLastError());
  }
}
