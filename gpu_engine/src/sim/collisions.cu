#include "sim/collisions.h"

#include "util/cuda_check.h"

#include <cuda_runtime.h>

namespace {

struct DeviceContactStats {
  unsigned int pairTests;
  unsigned int contacts;
};

__device__ int cell_for_position(float4 position, const UniformGrid& grid) {
  const int x = max(0, min(grid.dims.x - 1,
                           static_cast<int>((position.x - grid.min.x) /
                                             grid.cellSize)));
  const int y = max(0, min(grid.dims.y - 1,
                           static_cast<int>((position.y - grid.min.y) /
                                             grid.cellSize)));
  const int z = max(0, min(grid.dims.z - 1,
                           static_cast<int>((position.z - grid.min.z) /
                                             grid.cellSize)));
  const long long cell =
      static_cast<long long>(z) * grid.dims.y * grid.dims.x +
      static_cast<long long>(y) * grid.dims.x + x;
  return static_cast<int>(min(cell, static_cast<long long>(grid.numCells - 1)));
}

// Four blocks per SM is the occupancy target from the 3050 profile; revisit it when the kernel changes.
__global__ __launch_bounds__(256, 4)
void collision_kernel(const float4* pos_prev, const float4* vel_prev,
                      float4* pos_mass, float4* velocity, int count,
                      UniformGrid grid, SimParams params,
                      DeviceContactStats* stats) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) {
    return;
  }
  const float4 pi = pos_prev[index];
  const float4 vi = vel_prev[index];
  const float inv_mass_i = pi.w;
  Vec3 total_impulse{0.0f, 0.0f, 0.0f};
  Vec3 correction{0.0f, 0.0f, 0.0f};
  const int cell = cell_for_position(pi, grid);
  const int cell_x = cell % grid.dims.x;
  const int cell_y = (cell / grid.dims.x) % grid.dims.y;
  const int cell_z = cell / (grid.dims.x * grid.dims.y);

  // 3x3x3 neighborhood is enough when the cell size is at least the interaction radius.
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int nx = cell_x + dx;
        const int ny = cell_y + dy;
        const int nz = cell_z + dz;
        if (nx < 0 || ny < 0 || nz < 0 || nx >= grid.dims.x ||
            ny >= grid.dims.y || nz >= grid.dims.z) {
          continue;
        }
        const long long neighbor_cell =
            static_cast<long long>(nz) * grid.dims.y * grid.dims.x +
            static_cast<long long>(ny) * grid.dims.x + nx;
        const int clamped_cell =
            static_cast<int>(min(neighbor_cell,
                                  static_cast<long long>(grid.numCells - 1)));
        const int begin = grid.cellStart[clamped_cell];
        const int end = grid.cellStart[clamped_cell + 1];
        for (int offset = begin; offset < end; ++offset) {
          const int other = grid.sortedIdx[offset];
          if (other == index) {
            continue;
          }
          atomicAdd(&stats->pairTests, 1);
          const float4 pj = pos_prev[other];
          const float radius_sum = vi.w + vel_prev[other].w;
          if (vi.w <= 0.0f || vel_prev[other].w <= 0.0f ||
              radius_sum <= 0.0f) {
            continue;
          }
          const Vec3 delta{pi.x - pj.x, pi.y - pj.y, pi.z - pj.z};
          const float distance_squared = dot(delta, delta);
          if (distance_squared <= 1e-12f ||
              distance_squared >= radius_sum * radius_sum) {
            continue;
          }
          const float inverse_distance = rsqrtf(distance_squared);
          const Vec3 normal = delta * inverse_distance;
          const Vec3 relative_velocity{
              vi.x - vel_prev[other].x, vi.y - vel_prev[other].y,
              vi.z - vel_prev[other].z};
          const float normal_velocity = dot(relative_velocity, normal);
          const float inv_mass_j = pj.w;
          const float inverse_mass_sum = inv_mass_i + inv_mass_j;
          if (inverse_mass_sum > 0.0f && normal_velocity < 0.0f) {
            // Standard restitution impulse; the normal points from the other particle to this one.
            const float normal_impulse =
                -(1.0f + params.restitution) * normal_velocity /
                inverse_mass_sum;
            Vec3 impulse = normal * normal_impulse;
            const Vec3 tangent_velocity =
                relative_velocity - normal * normal_velocity;
            const float tangent_length_squared = dot(tangent_velocity, tangent_velocity);
            if (tangent_length_squared > 1e-12f) {
              const Vec3 tangent = tangent_velocity * rsqrtf(tangent_length_squared);
              float tangent_impulse =
                  -dot(relative_velocity, tangent) / inverse_mass_sum;
              // Coulomb clamp: |tangent impulse| <= mu * normal impulse.
              const float limit = params.friction * normal_impulse;
              tangent_impulse = fmaxf(-limit, fminf(limit, tangent_impulse));
              impulse += tangent * tangent_impulse;
            }
            total_impulse += impulse;
          }
          const float penetration = radius_sum - sqrtf(distance_squared);
          if (penetration > 0.0f && inverse_mass_sum > 0.0f) {
            correction += normal * (0.8f * penetration * inv_mass_i /
                                    inverse_mass_sum);
          }
          atomicAdd(&stats->contacts, 1);
        }
      }
    }
  }

  Vec3 new_position{pi.x + correction.x, pi.y + correction.y,
                    pi.z + correction.z};
  Vec3 new_velocity{vi.x + total_impulse.x, vi.y + total_impulse.y,
                    vi.z + total_impulse.z};
  // Cheap floor contact; the rigid-body path handles the general case.
  if (new_position.y - vi.w < params.planeY && vi.w > 0.0f) {
    new_position.y = params.planeY + vi.w;
    if (new_velocity.y < 0.0f) {
      new_velocity.y = -params.restitution * new_velocity.y;
      const float tangent_scale = fmaxf(0.0f, 1.0f - params.friction);
      new_velocity.x *= tangent_scale;
      new_velocity.z *= tangent_scale;
    }
  }
  if (inv_mass_i > 0.0f) {
    pos_mass[index].x = new_position.x;
    pos_mass[index].y = new_position.y;
    pos_mass[index].z = new_position.z;
    velocity[index].x = new_velocity.x;
    velocity[index].y = new_velocity.y;
    velocity[index].z = new_velocity.z;
  }
}

}  // namespace

void launch_collisions(ParticleSystem& particles, const UniformGrid& grid,
                       const SimParams& params, ContactStats* stats,
                       cudaStream_t stream) {
  if (stats != nullptr) {
    *stats = {};
  }
  if (particles.n <= 0 || grid.numCells <= 0) {
    return;
  }
  DeviceContactStats* device_stats = nullptr;
  CUDA_CHECK(cudaMallocAsync(&device_stats, sizeof(DeviceContactStats), stream));
  CUDA_CHECK(cudaMemsetAsync(device_stats, 0, sizeof(DeviceContactStats), stream));
  collision_kernel<<<(particles.n + 255) / 256, 256, 0, stream>>>(
      particles.posPrev, particles.velPrev, particles.posMass, particles.vel,
      particles.n, grid, params, device_stats);
  CUDA_CHECK(cudaGetLastError());
  if (stats != nullptr) {
    DeviceContactStats host_stats{};
    CUDA_CHECK(cudaMemcpyAsync(&host_stats, device_stats, sizeof(host_stats),
                               cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    stats->pairTests = host_stats.pairTests / 2;
    stats->contacts = host_stats.contacts / 2;
  }
  CUDA_CHECK(cudaFreeAsync(device_stats, stream));
}

void launch_collisions_ilp(ParticleSystem& particles, const UniformGrid& grid,
                           const SimParams& params, ContactStats* stats,
                           cudaStream_t stream) {
  launch_collisions(particles, grid, params, stats, stream);
}
