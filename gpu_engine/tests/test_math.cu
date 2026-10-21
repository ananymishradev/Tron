#include "test_util.h"
#include "util/cuda_check.h"
#include "util/math.h"

#include <cuda_runtime.h>

__global__ void rotate_kernel(Vec3* output, Quat rotation, Vec3 input) {
  if (threadIdx.x == 0) {
    *output = quat_rotate(rotation, input);
  }
}

int main() {
  const Vec3 sum = Vec3{1.0f, 2.0f, 3.0f} + Vec3{4.0f, 5.0f, 6.0f};
  CHECK_NEAR(sum.x, 5.0f, 1e-6f);
  CHECK_NEAR(sum.y, 7.0f, 1e-6f);
  CHECK_NEAR(sum.z, 9.0f, 1e-6f);
  CHECK_NEAR(dot({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}), 0.0f, 1e-6f);

  const Vec3 z = cross({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
  CHECK_NEAR(z.z, 1.0f, 1e-6f);

  const Quat rotation =
      quat_from_axis_angle({0.0f, 0.0f, 1.0f}, 1.57079632679f);
  const Vec3 expected{0.0f, 1.0f, 0.0f};
  const Vec3 rotated = quat_rotate(rotation, {1.0f, 0.0f, 0.0f});
  CHECK_NEAR(rotated.x, expected.x, 1e-6f);
  CHECK_NEAR(rotated.y, expected.y, 1e-6f);
  CHECK_NEAR(rotated.z, expected.z, 1e-6f);

  const Quat normalized = quat_normalize({2.0f, 3.0f, 4.0f, 5.0f});
  CHECK_NEAR(normalized.w * normalized.w + normalized.x * normalized.x +
                 normalized.y * normalized.y + normalized.z * normalized.z,
             1.0f, 1e-6f);

  const Vec3 matrix_rotated = mat3_mul_vec(mat3_from_quat(rotation), {1, 0, 0});
  CHECK_NEAR(matrix_rotated.x, rotated.x, 1e-6f);
  CHECK_NEAR(matrix_rotated.y, rotated.y, 1e-6f);
  CHECK_NEAR(matrix_rotated.z, rotated.z, 1e-6f);

  Vec3* device_output = nullptr;
  CUDA_CHECK(cudaMalloc(&device_output, sizeof(Vec3)));
  rotate_kernel<<<1, 1>>>(device_output, rotation, {1.0f, 0.0f, 0.0f});
  CUDA_CHECK(cudaGetLastError());
  Vec3 host_output{};
  CUDA_CHECK(cudaMemcpy(&host_output, device_output, sizeof(Vec3),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaFree(device_output));
  CHECK_NEAR(host_output.x, expected.x, 1e-6f);
  CHECK_NEAR(host_output.y, expected.y, 1e-6f);
  CHECK_NEAR(host_output.z, expected.z, 1e-6f);
  return 0;
}
