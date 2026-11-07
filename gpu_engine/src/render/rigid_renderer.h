#pragma once

#include "render/camera.h"
#include "sim/rigid.h"

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <SDL2/SDL_opengl.h>
#include <cuda_runtime.h>

Mat4 rigid_model_matrix(Vec3 position, Quat rotation, Vec3 scale);

class RigidRenderer {
 public:
  ~RigidRenderer();
  bool init();
  void upload(const RigidBodySoA& bodies, cudaStream_t stream);
  void draw(const Camera& camera, float aspect);

 private:
  GLuint vao_ = 0;
  GLuint mesh_vbo_ = 0;
  GLuint instance_vbo_ = 0;
  GLuint program_ = 0;
  int instance_count_ = 0;
};
