#pragma once

#include "sim/particles.h"

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <SDL2/SDL_opengl.h>
#include <cuda_runtime.h>

class ParticleVbo {
 public:
  ~ParticleVbo();
  bool init(GLuint vbo);
  float4* map();
  void unmap();
  bool using_interop() const { return resource_ != nullptr; }
  void fallback_upload(const float4* host, int n);

 private:
  GLuint vbo_ = 0;
  cudaGraphicsResource* resource_ = nullptr;
  float4* mapped_ = nullptr;
};

void launch_pack_render(const ParticleSystem& particles, float4* out,
                        cudaStream_t stream);
