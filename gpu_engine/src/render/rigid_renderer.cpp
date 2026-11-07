#include "render/rigid_renderer.h"

#include "render/shader.h"

#include <vector>

namespace {

struct Instance {
  Mat4 model;
  float color[4];
};

constexpr const char* kVertex = R"GLSL(
#version 330 core
layout(location = 0) in vec3 vertex;
layout(location = 2) in vec4 model0;
layout(location = 3) in vec4 model1;
layout(location = 4) in vec4 model2;
layout(location = 5) in vec4 model3;
layout(location = 6) in vec4 instance_color;
out vec4 color;
uniform mat4 view_proj;
void main() {
  mat4 model = mat4(model0, model1, model2, model3);
  gl_Position = view_proj * model * vec4(vertex, 1.0);
  color = instance_color;
}
)GLSL";

constexpr const char* kFragment = R"GLSL(
#version 330 core
in vec4 color;
out vec4 output_color;
void main() { output_color = color; }
)GLSL";

void set_instance_attributes() {
  for (int i = 0; i < 4; ++i) {
    glEnableVertexAttribArray(2 + i);
    glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, sizeof(Instance),
                          reinterpret_cast<void*>(sizeof(float) * 4 * i));
    glVertexAttribDivisor(2 + i, 1);
  }
  glEnableVertexAttribArray(6);
  glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Instance),
                        reinterpret_cast<void*>(sizeof(Mat4)));
  glVertexAttribDivisor(6, 1);
}

}  // namespace

Mat4 rigid_model_matrix(Vec3 position, Quat rotation, Vec3 scale) {
  const Mat3 basis = mat3_from_quat(rotation);
  Mat4 result = mat4_identity();
  result.m[0] = basis.m[0] * scale.x;
  result.m[1] = basis.m[3] * scale.x;
  result.m[2] = basis.m[6] * scale.x;
  result.m[4] = basis.m[1] * scale.y;
  result.m[5] = basis.m[4] * scale.y;
  result.m[6] = basis.m[7] * scale.y;
  result.m[8] = basis.m[2] * scale.z;
  result.m[9] = basis.m[5] * scale.z;
  result.m[10] = basis.m[8] * scale.z;
  result.m[12] = position.x;
  result.m[13] = position.y;
  result.m[14] = position.z;
  return result;
}

RigidRenderer::~RigidRenderer() {
  glDeleteProgram(program_);
  glDeleteBuffers(1, &mesh_vbo_);
  glDeleteBuffers(1, &instance_vbo_);
  glDeleteVertexArrays(1, &vao_);
}

bool RigidRenderer::init() {
  program_ = make_program(kVertex, kFragment);
  if (program_ == 0) {
    return false;
  }
  const float cube[] = {
      -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1,
      -1, -1,  1, 1, -1, 1, 1, 1, 1, -1, 1, 1,
  };
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &mesh_vbo_);
  glGenBuffers(1, &instance_vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cube), cube, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Instance), nullptr, GL_DYNAMIC_DRAW);
  set_instance_attributes();
  glBindVertexArray(0);
  return true;
}

void RigidRenderer::upload(const RigidBodySoA& bodies, cudaStream_t stream) {
  if (bodies.n <= 0) {
    instance_count_ = 0;
    return;
  }
  // The rigid path still round-trips through the host; body interop is future work.
  std::vector<float4> positions(bodies.n);
  std::vector<float4> rotations(bodies.n);
  std::vector<float4> extents(bodies.n);
  std::vector<int> shapes(bodies.n);
  cudaMemcpyAsync(positions.data(), bodies.pos, sizeof(float4) * bodies.n,
                  cudaMemcpyDeviceToHost, stream);
  cudaMemcpyAsync(rotations.data(), bodies.quat, sizeof(float4) * bodies.n,
                  cudaMemcpyDeviceToHost, stream);
  cudaMemcpyAsync(extents.data(), bodies.halfExtents, sizeof(float4) * bodies.n,
                  cudaMemcpyDeviceToHost, stream);
  cudaMemcpyAsync(shapes.data(), bodies.shape, sizeof(int) * bodies.n,
                  cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);
  std::vector<Instance> instances(bodies.n);
  for (int i = 0; i < bodies.n; ++i) {
    const Vec3 scale = shapes[i] == 0
                           ? Vec3{extents[i].w, extents[i].w, extents[i].w}
                           : Vec3{extents[i].x, extents[i].y, extents[i].z};
    instances[i].model = rigid_model_matrix(
        {positions[i].x, positions[i].y, positions[i].z},
        {rotations[i].x, rotations[i].y, rotations[i].z, rotations[i].w}, scale);
    instances[i].color[0] = shapes[i] == 0 ? 0.2f : 0.9f;
    instances[i].color[1] = shapes[i] == 0 ? 0.7f : 0.45f;
    instances[i].color[2] = 0.95f;
    instances[i].color[3] = 1.0f;
  }
  glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Instance) * instances.size(),
               instances.data(), GL_DYNAMIC_DRAW);
  instance_count_ = bodies.n;
}

void RigidRenderer::draw(const Camera& camera, float aspect) {
  if (instance_count_ <= 0) {
    return;
  }
  glUseProgram(program_);
  glUniformMatrix4fv(glGetUniformLocation(program_, "view_proj"), 1, GL_FALSE,
                     camera.view_proj(aspect).m);
  glBindVertexArray(vao_);
  glDrawArraysInstanced(GL_LINE_LOOP, 0, 8, instance_count_);
}
