#include "render/camera.h"
#include "render/renderer.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <cuda_runtime_api.h>

#include <cstdio>
#include <vector>

int main() {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "SDL init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_Window* window = SDL_CreateWindow("render-test", SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, 320, 240,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  if (window == nullptr) {
    std::fprintf(stderr, "SDL window failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (context == nullptr) {
    std::fprintf(stderr, "GL context failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  Renderer renderer;
  if (!renderer.init(window)) {
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  std::vector<float4> points(1000);
  for (int i = 0; i < 1000; ++i) {
    points[i] = {static_cast<float>(i % 20) - 10.0f,
                 static_cast<float>(i / 20) - 25.0f, 0.0f, 0.5f};
  }
  renderer.upload_particles(points.data(), static_cast<int>(points.size()));
  glViewport(0, 0, 320, 240);
  glClearColor(0.02f, 0.02f, 0.02f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  renderer.draw_grid();
  renderer.draw_particles(static_cast<int>(points.size()));
  glFinish();

  std::vector<unsigned char> pixels(320 * 240 * 3);
  glReadPixels(0, 0, 320, 240, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  int differing = 0;
  for (std::size_t i = 0; i < pixels.size(); i += 3) {
    if (pixels[i] > 4 || pixels[i + 1] > 4 || pixels[i + 2] > 4) {
      ++differing;
    }
  }
  if (differing <= 500) {
    return 1;
  }

  Camera camera;
  const Mat4 wide = camera.view_proj(16.0f / 9.0f);
  const Mat4 standard = camera.view_proj(4.0f / 3.0f);
  if (wide.m[0] == standard.m[0]) {
    return 1;
  }
  const Ray center = camera.ray(0.0f, 0.0f, 16.0f / 9.0f);
  if (dot(center.dir, normalize(center.dir)) < 0.999f) {
    return 1;
  }

  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
