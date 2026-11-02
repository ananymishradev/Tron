#pragma once

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <cuda_runtime_api.h>

class Renderer {
 public:
  ~Renderer();
  bool init(SDL_Window* window);
  GLuint particle_vbo() const { return particle_vbo_; }
  void upload_particles(const float4* host, int n);
  void draw_particles(int n);
  void draw_grid();
  void present();

 private:
  GLuint particle_vao_ = 0;
  GLuint particle_vbo_ = 0;
  GLuint grid_vao_ = 0;
  GLuint grid_vbo_ = 0;
  GLuint particle_program_ = 0;
  GLuint grid_program_ = 0;
  int particle_capacity_ = 0;
  SDL_Window* window_ = nullptr;
};
