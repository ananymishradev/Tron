#include "render/rigid_renderer.h"

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include <cmath>

int main() {
  const Mat4 translation =
      rigid_model_matrix({1, 2, 3}, {1, 0, 0, 0}, {1, 1, 1});
  if (translation.m[12] != 1 || translation.m[13] != 2 ||
      translation.m[14] != 3) {
    return 1;
  }
  const Mat4 rotation = rigid_model_matrix(
      {0, 0, 0}, quat_from_axis_angle({0, 0, 1}, 1.57079632679f), {1, 1, 1});
  const Vec3 rotated{rotation.m[0], rotation.m[1], rotation.m[2]};
  if (std::fabs(rotated.x) > 1e-5f || std::fabs(rotated.y - 1.0f) > 1e-5f ||
      std::fabs(rotated.z) > 1e-5f) {
    return 1;
  }

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_Window* window = SDL_CreateWindow("rigid-render-test",
                                        SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, 320, 240,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  if (!window) {
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (!context) {
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  RigidRenderer renderer;
  const bool initialized = renderer.init();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return initialized ? 0 : 1;
}
