#include "render/interop.h"
#include "render/renderer.h"
#include "util/cuda_check.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

static int run_case(SDL_Window* window, bool force_fallback) {
  Renderer renderer;
  if (!renderer.init(window)) {
    return 1;
  }
  ParticleVbo vbo;
  if (force_fallback) {
    setenv("GPU_ENGINE_DISABLE_INTEROP", "1", 1);
  } else {
    unsetenv("GPU_ENGINE_DISABLE_INTEROP");
  }
  const bool interop = vbo.init(renderer.particle_vbo());

  ParticleSystem particles;
  particles.alloc(3);
  const float4 positions[3] = {{1.0f, 2.0f, 3.0f, 1.0f},
                               {-1.0f, 0.0f, 2.0f, 1.0f},
                               {0.0f, 0.0f, 0.0f, 1.0f}};
  const float4 velocities[3] = {{3.0f, 0.0f, 0.0f, 0.1f},
                               {0.0f, 4.0f, 0.0f, 0.1f},
                               {0.0f, 0.0f, 5.0f, 0.1f}};
  CUDA_CHECK(cudaMemcpy(particles.posMass, positions, sizeof(positions),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(particles.vel, velocities, sizeof(velocities),
                        cudaMemcpyHostToDevice));

  std::vector<float4> expected(3);
  for (int i = 0; i < 3; ++i) {
    expected[i] = {positions[i].x, positions[i].y, positions[i].z,
                   static_cast<float>(i + 3) / 10.0f};
  }
  if (interop) {
    float4* mapped = vbo.map();
    launch_pack_render(particles, mapped, nullptr);
    CUDA_CHECK(cudaDeviceSynchronize());
    vbo.unmap();
  } else {
    std::vector<float4> host(3);
    CUDA_CHECK(cudaMemcpy(host.data(), particles.posMass, sizeof(float4) * 3,
                          cudaMemcpyDeviceToHost));
    for (int i = 0; i < 3; ++i) {
      host[i].w = expected[i].w;
    }
    vbo.fallback_upload(host.data(), 3);
  }
  std::vector<float4> actual(3);
  glBindBuffer(GL_ARRAY_BUFFER, renderer.particle_vbo());
  glGetBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float4) * 3, actual.data());
  for (int i = 0; i < 3; ++i) {
    if (actual[i].x != expected[i].x || actual[i].y != expected[i].y ||
        actual[i].z != expected[i].z || actual[i].w != expected[i].w) {
      particles.free();
      return 1;
    }
  }
  particles.free();
  return force_fallback && interop ? 1 : 0;
}

int main() {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_Window* window = SDL_CreateWindow("interop-test", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, 320, 240,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  if (window == nullptr) {
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (context == nullptr) {
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  const int result = run_case(window, false) || run_case(window, true);
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
