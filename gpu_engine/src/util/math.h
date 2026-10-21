#pragma once

#include <cmath>

#ifdef __CUDACC__
#define HD __host__ __device__
#else
#define HD
#endif

struct Vec3 {
  float x, y, z;

  HD Vec3 operator+(Vec3 other) const { return {x + other.x, y + other.y, z + other.z}; }
  HD Vec3 operator-(Vec3 other) const { return {x - other.x, y - other.y, z - other.z}; }
  HD Vec3 operator-() const { return {-x, -y, -z}; }
  HD Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
  HD Vec3 operator/(float scalar) const {
    return {x / scalar, y / scalar, z / scalar};
  }
  HD Vec3& operator+=(Vec3 other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
};

HD inline Vec3 operator*(float scalar, Vec3 value) { return value * scalar; }

struct Quat {
  float w, x, y, z;
};

struct Mat3 {
  float m[9]{};
};

struct Mat4 {
  float m[16]{};
};

HD inline float dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

HD inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

HD inline float length_squared(Vec3 value) { return dot(value, value); }

HD inline Vec3 normalize(Vec3 value) {
  const float squared = length_squared(value);
  if (squared <= 0.0f) {
    return {0.0f, 0.0f, 0.0f};
  }
  const float inverse_length = 1.0f / sqrtf(squared);
  return value * inverse_length;
}

HD inline Quat quat_mul(Quat a, Quat b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

HD inline Quat quat_normalize(Quat value);

HD inline Quat quat_from_axis_angle(Vec3 axis, float radians) {
  const Vec3 unit_axis = normalize(axis);
  const float half_angle = radians * 0.5f;
  const float sine = sinf(half_angle);
  return quat_normalize(
      {cosf(half_angle), unit_axis.x * sine, unit_axis.y * sine, unit_axis.z * sine});
}

HD inline Quat quat_normalize(Quat value) {
  const float squared =
      value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z;
  if (squared <= 0.0f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  const float inverse_length = 1.0f / sqrtf(squared);
  return {value.w * inverse_length, value.x * inverse_length, value.y * inverse_length,
          value.z * inverse_length};
}

// q * v * q^-1 in vector form. Normalize first so repeated rotations do not drift.
HD inline Vec3 quat_rotate(Quat rotation, Vec3 value) {
  const Quat unit = quat_normalize(rotation);
  const Vec3 quaternion_vector{unit.x, unit.y, unit.z};
  const Vec3 twice_cross = 2.0f * cross(quaternion_vector, value);
  return value + unit.w * twice_cross + cross(quaternion_vector, twice_cross);
}

HD inline Mat3 mat3_from_quat(Quat rotation) {
  const Quat q = quat_normalize(rotation);
  return {{1.0f - 2.0f * (q.y * q.y + q.z * q.z),
           2.0f * (q.x * q.y - q.z * q.w), 2.0f * (q.x * q.z + q.y * q.w),
           2.0f * (q.x * q.y + q.z * q.w), 1.0f - 2.0f * (q.x * q.x + q.z * q.z),
           2.0f * (q.y * q.z - q.x * q.w), 2.0f * (q.x * q.z - q.y * q.w),
           2.0f * (q.y * q.z + q.x * q.w), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)}};
}

HD inline Vec3 mat3_mul_vec(Mat3 matrix, Vec3 value) {
  return {matrix.m[0] * value.x + matrix.m[1] * value.y + matrix.m[2] * value.z,
          matrix.m[3] * value.x + matrix.m[4] * value.y + matrix.m[5] * value.z,
          matrix.m[6] * value.x + matrix.m[7] * value.y + matrix.m[8] * value.z};
}

HD inline Mat3 mat3_transpose(Mat3 matrix) {
  return {{matrix.m[0], matrix.m[3], matrix.m[6], matrix.m[1], matrix.m[4], matrix.m[7],
           matrix.m[2], matrix.m[5], matrix.m[8]}};
}

HD inline Mat4 mat4_identity() {
  return {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
           0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
}

HD inline Mat4 mat4_mul(Mat4 a, Mat4 b) {
  Mat4 result{};
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      for (int k = 0; k < 4; ++k) {
        result.m[row * 4 + column] += a.m[row * 4 + k] * b.m[k * 4 + column];
      }
    }
  }
  return result;
}

HD inline Mat4 mat4_perspective(float fovy, float aspect, float near_plane,
                                float far_plane) {
  const float inverse_tangent = 1.0f / tanf(fovy * 0.5f);
  Mat4 result{};
  result.m[0] = inverse_tangent / aspect;
  result.m[5] = inverse_tangent;
  result.m[10] = (far_plane + near_plane) / (near_plane - far_plane);
  result.m[11] = -1.0f;
  result.m[14] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
  return result;
}

HD inline Mat4 mat4_look_at(Vec3 eye, Vec3 target, Vec3 up) {
  const Vec3 forward = normalize(target - eye);
  const Vec3 side = normalize(cross(forward, up));
  const Vec3 corrected_up = cross(side, forward);
  Mat4 result = mat4_identity();
  result.m[0] = side.x;
  result.m[1] = side.y;
  result.m[2] = side.z;
  result.m[4] = corrected_up.x;
  result.m[5] = corrected_up.y;
  result.m[6] = corrected_up.z;
  result.m[8] = -forward.x;
  result.m[9] = -forward.y;
  result.m[10] = -forward.z;
  result.m[12] = -dot(side, eye);
  result.m[13] = -dot(corrected_up, eye);
  result.m[14] = dot(forward, eye);
  return result;
}

#undef HD
