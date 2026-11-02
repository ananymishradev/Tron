#pragma once

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <SDL2/SDL_opengl.h>

#include <string>

GLuint compile_shader(GLenum type, const char* source);
GLuint link_program(GLuint vertex_shader, GLuint fragment_shader);
GLuint make_program(const char* vertex_source, const char* fragment_source);
