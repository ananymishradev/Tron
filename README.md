<p align="center">
  <img src="assests/ascii-art-text.png" alt="Tron" width="440">
</p>

<h1 align="center">Tron</h1>

<p align="center">
  <strong>A GPU physics playground for seeing a million particles move.</strong>
</p>

Tron is a small CUDA-powered particle and rigid-body engine built around one
simple idea: physics is more exciting when you can watch the machine do the
work.

It combines GPU simulation, a hand-written SDL2/OpenGL renderer, collision
experiments, profiling tools, and live observability in one focused project.
The goal is not to hide the hard parts behind a large framework. The goal is
to make the algorithms visible, measurable, and fun to explore.

## Why Tron?

Most physics demos show the final picture. Tron also shows the engineering
behind that picture:

- How far can an RTX 3050 push a particle simulation?
- What does a uniform-grid broadphase buy us?
- Which collision strategy is actually faster?
- What happens when CUDA/OpenGL interop is unavailable?
- Can a simulation remain inspectable while it is running?

The project is designed as a learning system and a performance experiment,
not just a finished demo.

## What is inside

### Particle simulation

Particles use structure-of-arrays GPU buffers with deterministic,
seed-controlled initialization. The simulation includes gravity, drag,
semi-implicit Euler integration, particle collisions, plane contacts, and
kinetic/potential/momentum statistics.

### Uniform-grid broadphase

The broadphase uses counting sort and CUB exclusive scan to organize particles
into spatial cells. If a scene requests an unreasonable number of cells, Tron
clamps the grid and reports a warning instead of silently exhausting device
memory.

### Rigid bodies

Rigid bodies use quaternion orientation, sphere and box shapes, plane contacts,
and a Jacobi-style contact solve. Their state lives in GPU-friendly
structure-of-arrays buffers as well.

### Rendering and inspection

The renderer uses SDL2 and raw OpenGL 3.3. It supports point-cloud particles,
instanced rigid-body rendering, an orbit camera, and CUDA/OpenGL interop with a
host-upload fallback for drivers or contexts that do not support registration.

The engine also includes ray-based particle probing and CSV statistics logging,
so a run can be inspected visually and numerically.

## Current status

The core engine, render components, CUDA/OpenGL fallback, rigid-body APIs,
benchmarks, and correctness tests are implemented.

The remaining product-level work is wiring every subsystem into one polished
interactive CLI experience. In other words: the engine pieces are here, and
the next step is turning them into the final show.

## Performance snapshot

On an NVIDIA GeForce RTX 3050 6GB Laptop GPU:

| Workload | Result |
|---|---:|
| Forces + integration, 1M particles | ~1.19 ms/step |
| Uniform-grid setup + sort, 1M particles | ~21.82 ms |
| Collision baseline, 100K particles | ~4276 ms/step |

The collision baseline is intentionally kept as a reference point for future
optimization. Performance notes and negative results live in
[`gpu_engine/docs/perf.md`](gpu_engine/docs/perf.md).

## Build

### Requirements

- Linux
- CUDA 13.4+
- CMake 3.24+
- SDL2
- OpenGL
- An NVIDIA GPU with compute capability 8.6 for the target configuration

On Arch Linux:

```bash
sudo pacman -S cuda sdl2 cmake
```

### Configure and build

From the repository root:

```bash
cmake -S gpu_engine -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Check the device

```bash
./build/gpu_engine --info
```

### Run the tests

```bash
ctest --test-dir build --output-on-failure
```

The test suite covers CUDA smoke checks, math, deterministic particle
initialization, forces, grid construction, collisions, rendering,
CUDA/OpenGL fallback, rigid bodies, and observability.

## Documentation

- [Quick user guide](docs/user-guide.md) — build, CLI, TUI keys, and output files
- [Developer guide](docs/developer-guide.md) — architecture, simulation pipeline,
  tests, and benchmarks
- [Performance notes](gpu_engine/docs/perf.md) — measured timings and optimization log

## Project layout

```text
gpu_engine/
  src/sim/       particle, grid, collision, and rigid-body systems
  src/render/    camera, OpenGL renderer, rigid renderer, interop
  src/app/       probing, logging, and application scaffolding
  src/util/      CUDA checks, math, and device information
  tests/         GPU and CPU-reference correctness tests
  bench/         stage benchmarks and tuning matrix
  docs/perf.md   measurements and optimization notes
```

## Controls planned for the interactive viewer

| Input | Action |
|---|---|
| `Space` | Pause or resume |
| `.` | Step once while paused |
| `R` | Reset the scene |
| Mouse drag | Orbit the camera |
| Mouse wheel | Zoom |
| `P` | Toggle particle probing |
| `L` | Toggle CSV logging |
| `H` | Toggle the HUD |
| `Esc` | Quit |

## Honest limits

Tron is deliberately focused:

- uniform gravity only; no long-range N-body gravity
- no fluids, cloth, soft bodies, or box-box contacts yet
- particles and rigid bodies are not two-way coupled in v1
- Linux is the supported platform
- the final interactive CLI wiring is still in progress

## The story behind the project

Tron is an attempt to make computational physics feel tangible: not just a
formula in a notebook, and not just a finished animation, but a living system
whose data structures, kernels, tradeoffs, and failures can all be explored.

If you are interested in CUDA, simulation, graphics, or performance
engineering, this repository is meant to be readable as well as runnable.

## License

MIT. See [`LICENSE`](LICENSE).
