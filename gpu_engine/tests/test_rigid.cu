#include "test_util.h"
#include "util/cuda_check.h"
#include "sim/rigid.h"

#include <cuda_runtime.h>

int main() {
  RigidBodySoA bodies;
  bodies.alloc(1);
  const float4 position{0.0f, 0.1f, 0.0f, 1.0f};
  const float4 extent{0.0f, 0.0f, 0.0f, 0.1f};
  const float4 velocity{0.0f, 0.0f, 0.0f, 0.0f};
  CUDA_CHECK(cudaMemcpy(bodies.pos, &position, sizeof(position),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(bodies.halfExtents, &extent, sizeof(extent),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(bodies.vel, &velocity, sizeof(velocity),
                        cudaMemcpyHostToDevice));
  SimParams params;
  params.restitution = 0.0f;
  params.gravity = {0.0f, -9.81f, 0.0f};
  int* device_count = nullptr;
  Contact* device_contacts = nullptr;
  CUDA_CHECK(cudaMalloc(&device_count, sizeof(int)));
  CUDA_CHECK(cudaMalloc(&device_contacts, sizeof(Contact) * 8));
  for (int i = 0; i < 120; ++i) {
    launch_rigid_integrate(bodies, params, nullptr);
    launch_contacts_generate(bodies, UniformGrid{}, device_contacts,
                             device_count, 8, nullptr);
    int count = 0;
    CUDA_CHECK(cudaMemcpy(&count, device_count, sizeof(int),
                          cudaMemcpyDeviceToHost));
    launch_contacts_solve(bodies, device_contacts, count, 4, params, nullptr);
  }
  CUDA_CHECK(cudaDeviceSynchronize());
  float4 result{};
  CUDA_CHECK(cudaMemcpy(&result, bodies.pos, sizeof(result),
                        cudaMemcpyDeviceToHost));
  CHECK(result.y >= 0.09f);

  bodies.free();
  CUDA_CHECK(cudaFree(device_count));
  CUDA_CHECK(cudaFree(device_contacts));
  return 0;
}
