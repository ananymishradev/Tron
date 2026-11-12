<p align="center">
  <img src="../assests/ascii-art-text.png" alt="Tron" width="440">
</p>

<h1 align="center">Tron</h1>

A CUDA particle and rigid-body engine with a hand-written OpenGL front end.

Tron is a small physics engine built to run on a laptop GPU and show its work. It simulates particles and rigid bodies on the GPU, draws them with raw OpenGL, and reports what each stage costs. There is no scene editor or UI toolkit, and nothing allocates behind your back in the frame loop.

I started it to answer a practical question: how much simulation can an RTX 3050 handle before the frame budget disappears, and which kernel gives out first?

## Status

The current implementation includes:

- float math layer, SoA particle storage, deterministic init
- gravity, drag, semi-implicit Euler
- kinetic energy, potential, and momentum reductions
- counting-sort uniform grid
- 1M particles at about 1.19 ms per step for forces and integration

- snapshot-based particle collisions with plane contacts
- SDL2/OpenGL point and instanced rigid-body rendering
- CUDA-GL interop with host upload fallback
- rigid-body initialization, integration, and plane contacts
- probe selection, CSV logging, and stage benchmarks

The full task list is in [task.md](../task.md).

## What it will do

Target scene: 1,000,000 particles and 10,000 rigid bodies, interactive on an RTX 3050 Laptop with 6 GB.

- Particles collide through a uniform grid and resolve with impulses.
- Rigid bodies use quaternions, sphere and box shapes, and a Jacobi contact solver.
- The view is drawn by hand: points, a floor grid, instanced bodies, and a text HUD.
- Pause, step, reset, orbit, zoom, and click a particle to inspect it.
- CSV logs of frame time, pair counts, contacts, kinetic energy, and potential.

## Design

### Simulation

Particles live in structure-of-arrays buffers. Each particle is a few `float4` values: position plus inverse mass, velocity plus radius, force, and a material id. The layout keeps warp accesses contiguous and avoids the pointer chasing of the original CPU code.

Integration is semi-implicit Euler at a fixed 1/60 s step. The frame loop accumulates real time and runs at most two substeps, so a slow frame slows the simulation instead of exploding it.

### Broadphase

A uniform grid is rebuilt each step with a counting sort. Count particles per cell, exclusive-scan the counts, then scatter particle indices into cell order. The grid is clamped to a fixed cell budget, so a bad scene scale produces a warning instead of an out-of-memory crash.

### Collisions

Particle contacts use a snapshot of positions and velocities. Each thread reads the snapshot, accumulates its own impulse, and writes only its own particle. Every pair is evaluated twice, which costs some work but removes atomics from the hot path and keeps results independent of thread order.

Rigid body contacts are generated for sphere-sphere, sphere-plane, and box-plane pairs, then solved with ten Jacobi iterations. The solver clamps normal impulses at zero and uses a small Baumgarte correction. Warm starting is not in the first version.

### Rendering

SDL2 opens a window and an OpenGL 3.3 core context. Particles are drawn as points colored by speed. Rigid bodies are instanced spheres and boxes. The HUD is built from `stb_easy_font` quads, so there is no font library.

When interop is available, a CUDA kernel writes directly into the mapped vertex buffer. If registration fails, the renderer falls back to a pinned host buffer and keeps going.

## Performance work

The optimization loop follows the SGEMM worklog by Simon Boehm: start with the simplest correct kernel, measure it, then change one thing at a time.

The order is:

1. coalesced structure-of-arrays access
2. occupancy and register pressure, checked with Nsight Compute
3. more work per thread, reusing cell ranges
4. vectorized `float4` loads
5. shared-memory tiles for dense scenes
6. `cp.async` double buffering
7. autotuning block size, particles per thread, and cell size

A variant stays only if the benchmark says it is faster. Slower experiments get recorded in [docs/perf.md](docs/perf.md) and removed. Current numbers live there too.

## Build

Requires Linux, CUDA 13.4 or newer, SDL2, OpenGL, and CMake 3.24.

On Arch:

```
sudo pacman -S cuda sdl2 cmake
```

From this directory:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Check the GPU and toolchain:

```
./build/gpu_engine --info
```

Run the tests:

```
ctest --test-dir build --output-on-failure
```

The build target is `gpu_engine`; the project name is Tron.

## Run

The executable currently supports the device probe:

```
./build/gpu_engine --info
```

The simulation and renderer APIs are available through the `engine` library;
the complete interactive CLI wiring remains a follow-up limitation.

## Controls

| Key | Action |
|---|---|
| Space | pause or resume |
| `.` | single step while paused |
| R | reset the scene |
| mouse drag | orbit the camera |
| wheel | zoom |
| P | toggle the probe panel |
| L | toggle CSV logging |
| H | toggle the HUD |
| Esc | quit |

## Layout

```
gpu_engine/
  src/sim/      particle and rigid body state, kernels
  src/render/   OpenGL renderer, camera, HUD, interop
  src/app/      window, frame loop, probe, logging
  src/util/     math, CUDA checks, device probe
  tests/        correctness tests and CPU references
  bench/        stage benchmarks and autotuning
  docs/perf.md  measurements and negative results
```

## Limits

Tron is deliberately small.

- Uniform gravity only. No Barnes-Hut or fast multipole.
- No fluids, soft bodies, or cloth.
- Particles and rigid bodies do not push on each other in the first version.
- box-box contacts are not implemented yet
- the final interactive CLI acceptance path is not yet wired into `main.cpp`
- Linux only.

## Credits

The physics follows the CMU SIGGRAPH course notes by Andrew Witkin and David Baraff. The performance method comes from Simon Boehm's CUDA SGEMM worklog. SDL2, glad, and stb_easy_font do the unglamorous parts.

## License

MIT. See [LICENSE](../LICENSE).
