#include "render/renderer.h"

#include "render/shader.h"

#include <algorithm>
#include <vector>

namespace {

// Shaders are embedded so the binary has no runtime asset path.
constexpr const char* kPointVertex = R"GLSL(
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in float speed;
out float point_speed;
void main() {
  gl_Position = vec4(position.x / 30.0, position.y / 30.0 - 0.25,
                     position.z / 30.0, 1.0);
  gl_PointSize = 3.0;
  point_speed = speed;
}
)GLSL";

constexpr const char* kPointFragment = R"GLSL(
#version 330 core
in float point_speed;
out vec4 color;
void main() {
  float s = clamp(point_speed, 0.0, 1.0);
  color = s < 0.5 ? mix(vec4(0.1, 0.3, 1.0, 1.0), vec4(1.0), s * 2.0)
                  : mix(vec4(1.0), vec4(1.0, 0.1, 0.05, 1.0), (s - 0.5) * 2.0);
}
)GLSL";

constexpr const char* kGridVertex = R"GLSL(
#version 330 core
layout(location = 0) in vec3 position;
void main() {
  gl_Position = vec4(position.x / 30.0, -0.25, position.z / 30.0, 1.0);
}
)GLSL";

constexpr const char* kGridFragment = R"GLSL(
#version 330 core
out vec4 color;
void main() { color = vec4(0.25, 0.25, 0.25, 1.0); }
)GLSL";

}  // namespace

Renderer::~Renderer() {
  glDeleteProgram(particle_program_);
  glDeleteProgram(grid_program_);
  glDeleteBuffers(1, &particle_vbo_);
  glDeleteBuffers(1, &grid_vbo_);
  glDeleteVertexArrays(1, &particle_vao_);
  glDeleteVertexArrays(1, &grid_vao_);
}

bool Renderer::init(SDL_Window* window) {
  window_ = window;
  particle_program_ = make_program(kPointVertex, kPointFragment);
  grid_program_ = make_program(kGridVertex, kGridFragment);
  if (particle_program_ == 0 || grid_program_ == 0) {
    return false;
  }
  glGenVertexArrays(1, &particle_vao_);
  glGenBuffers(1, &particle_vbo_);
  glBindVertexArray(particle_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4, nullptr, GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 4, nullptr);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float) * 4,
                        reinterpret_cast<void*>(sizeof(float) * 3));
  glEnableVertexAttribArray(1);

  std::vector<float> grid;
  for (int i = -15; i <= 15; ++i) {
    grid.insert(grid.end(), {static_cast<float>(i), 0.0f, -15.0f,
                             static_cast<float>(i), 0.0f, 15.0f});
    grid.insert(grid.end(), {-15.0f, 0.0f, static_cast<float>(i),
                             15.0f, 0.0f, static_cast<float>(i)});
  }
  glGenVertexArrays(1, &grid_vao_);
  glGenBuffers(1, &grid_vbo_);
  glBindVertexArray(grid_vao_);
  glBindBuffer(GL_ARRAY_BUFFER, grid_vbo_);
  glBufferData(GL_ARRAY_BUFFER, grid.size() * sizeof(float), grid.data(),
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);
  return true;
}

// Fallback path used when CUDA-GL interop is unavailable.
void Renderer::upload_particles(const float4* host, int n) {
  if (n > particle_capacity_) {
    particle_capacity_ = std::max(n, particle_capacity_ * 2);
    glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float4) * particle_capacity_, nullptr,
                 GL_DYNAMIC_DRAW);
  }
  glBindBuffer(GL_ARRAY_BUFFER, particle_vbo_);
  glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float4) * n, host);
}

void Renderer::draw_particles(int n) {
  glUseProgram(particle_program_);
  glBindVertexArray(particle_vao_);
  glDrawArrays(GL_POINTS, 0, n);
}

void Renderer::draw_grid() {
  glUseProgram(grid_program_);
  glBindVertexArray(grid_vao_);
  glDrawArrays(GL_LINES, 0, 124);
}

void Renderer::present() {
  SDL_GL_SwapWindow(window_);
}
