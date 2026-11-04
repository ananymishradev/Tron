#include "render/interop.h"

#include "util/cuda_check.h"

#include <cuda_gl_interop.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace {

// Pack position and speed into the VBO layout the point shader expects.
__global__ void pack_render_kernel(const float4* pos_mass, const float4* velocity,
                                   float4* output, int count) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float speed = sqrtf(velocity[index].x * velocity[index].x +
                            velocity[index].y * velocity[index].y +
                            velocity[index].z * velocity[index].z);
  output[index] = {pos_mass[index].x, pos_mass[index].y, pos_mass[index].z,
                   fminf(1.0f, speed)};
}

}  // namespace

ParticleVbo::~ParticleVbo() {
  if (resource_ != nullptr) {
    cudaGraphicsUnregisterResource(resource_);
  }
}

bool ParticleVbo::init(GLuint vbo) {
  vbo_ = vbo;
  // Escape hatch for drivers where registration succeeds but mapping is slow or broken.
  const char* disabled = std::getenv("GPU_ENGINE_DISABLE_INTEROP");
  if (disabled != nullptr && disabled[0] == '1') {
    return false;
  }
  const cudaError_t result = cudaGraphicsGLRegisterBuffer(
      &resource_, vbo_, cudaGraphicsMapFlagsWriteDiscard);
  if (result != cudaSuccess) {
    std::fprintf(stderr, "warning: CUDA-GL interop unavailable: %s\n",
                 cudaGetErrorString(result));
    resource_ = nullptr;
    return false;
  }
  return true;
}

float4* ParticleVbo::map() {
  if (resource_ == nullptr) {
    return nullptr;
  }
  CUDA_CHECK(cudaGraphicsMapResources(1, &resource_, nullptr));
  std::size_t bytes = 0;
  CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(
      reinterpret_cast<void**>(&mapped_), &bytes, resource_));
  return mapped_;
}

void ParticleVbo::unmap() {
  if (resource_ != nullptr) {
    CUDA_CHECK(cudaGraphicsUnmapResources(1, &resource_, nullptr));
    mapped_ = nullptr;
  }
}

void ParticleVbo::fallback_upload(const float4* host, int n) {
  if (resource_ != nullptr || vbo_ == 0 || n <= 0) {
    return;
  }
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float4) * n, host);
}

void launch_pack_render(const ParticleSystem& particles, float4* out,
                        cudaStream_t stream) {
  if (particles.n <= 0 || out == nullptr) {
    return;
  }
  pack_render_kernel<<<(particles.n + 255) / 256, 256, 0, stream>>>(
      particles.posMass, particles.vel, out, particles.n);
  CUDA_CHECK(cudaGetLastError());
}
