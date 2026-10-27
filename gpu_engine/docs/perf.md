# Performance Notes

Measurements are from an NVIDIA GeForce RTX 3050 6GB Laptop GPU, CUDA 13.4,
Release builds, and include only the listed device stages.

| Stage | Configuration | Workload | Mean |
|---|---|---:|---:|
| Forces + semi-implicit integration | 256 threads/block | 1,000,000 particles | 1.188 ms/step |
| Uniform grid setup + counting sort | 256 threads/block, cell size 0.5 | 1,000,000 particles | 21.819 ms |
