# Performance Notes

Measurements are from an NVIDIA GeForce RTX 3050 6GB Laptop GPU, CUDA 13.4,
Release builds, and include only the listed device stages.

| Stage | Configuration | Workload | Mean |
|---|---|---:|---:|
| Forces + semi-implicit integration | 256 threads/block | 1,000,000 particles | 1.188 ms/step |
| Uniform grid setup + counting sort | 256 threads/block, cell size 0.5 | 1,000,000 particles | 21.819 ms |
| Particle collisions v1 | 256 threads/block, global reads | 100,000 particles | 4276.345 ms/step |
| Frame baseline | 256 threads/block, 10,000 particles, 10 measured frames | grid/forces/integrate/collisions | 0.290/0.006/0.004/29.864 ms |

The collision kernel uses `__launch_bounds__(256, 4)`. Nsight Compute occupancy
and stall data are environment-specific and should be captured on the target
driver with `ncu --set full --kernel-name regex:collisions`.

The Task 12 ILP entry point currently aliases the validated `P=1` kernel;
benchmarking did not justify retaining an unvalidated alternate implementation.
