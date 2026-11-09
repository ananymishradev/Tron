#pragma once

// Tuned on the RTX 3050; keep these together so benchmarks and kernels agree.
constexpr int kBlockSize = 256;
constexpr int kParticlesPerThread = 1;
constexpr float kCellSizeFactor = 2.0f;
constexpr bool kUseSmem = false;
