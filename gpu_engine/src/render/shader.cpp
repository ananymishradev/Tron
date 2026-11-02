#include "render/shader.h"

#include <cstdio>
#include <vector>

GLuint compile_shader(GLenum type, const char* source) {
  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint compiled = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
  if (compiled == GL_FALSE) {
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> log(static_cast<std::size_t>(length) + 1);
    glGetShaderInfoLog(shader, length, nullptr, log.data());
    std::fprintf(stderr, "OpenGL shader compile failed: %s\n", log.data());
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

GLuint link_program(GLuint vertex_shader, GLuint fragment_shader) {
  const GLuint program = glCreateProgram();
  glAttachShader(program, vertex_shader);
  glAttachShader(program, fragment_shader);
  glLinkProgram(program);
  GLint linked = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &linked);
  if (linked == GL_FALSE) {
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> log(static_cast<std::size_t>(length) + 1);
    glGetProgramInfoLog(program, length, nullptr, log.data());
    std::fprintf(stderr, "OpenGL program link failed: %s\n", log.data());
    glDeleteProgram(program);
    return 0;
  }
  return program;
}

GLuint make_program(const char* vertex_source, const char* fragment_source) {
  const GLuint vertex = compile_shader(GL_VERTEX_SHADER, vertex_source);
  const GLuint fragment = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
  if (vertex == 0 || fragment == 0) {
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    return 0;
  }
  const GLuint program = link_program(vertex, fragment);
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  return program;
}
