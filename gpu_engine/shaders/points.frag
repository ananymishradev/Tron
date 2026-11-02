#version 330 core
in float point_speed;
out vec4 color;
void main() { color = vec4(1.0 - point_speed, 0.5, point_speed, 1.0); }
