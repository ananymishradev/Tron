#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in float speed;
out float point_speed;
void main() {
  gl_Position = vec4(position, 1.0);
  gl_PointSize = 2.0;
  point_speed = speed;
}
