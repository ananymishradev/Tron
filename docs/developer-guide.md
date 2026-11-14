# Tron developer guide

This guide is for working on the engine itself: the simulation pipeline, the
terminal UI, the render library, the tests, and the benchmarks.

If you only want to build and run Tron, read the
[quick user guide](user-guide.md) first.

## 1. Architecture at a glance

```text
                    +----------------------+
                    |      src/main.cpp    |
                    |  parse_cli + mode    |
                    +----------+-----------+
                               |
             +-----------------+-----------------+
             |                                   |
     +-------v--------+                  +-------v--------+
     |  TuiApp        |                  |  run_headless  |
     |  src/tui/      |                  |  src/app/      |
     +-------+--------+                  +-------+--------+
             |                                   |
     +-------v--------+                  +-------v--------+
     | SimulationRunner|                 |  Simulation    |
     | worker thread   |                 |  CUDA state    |
     +-------+--------+                  +-------+--------+
             |                                   |
             +-----------------+-----------------+
                               |
                    +----------v-----------+
                    |   simulation step    |
                    | grid -> forces ->    |
                    | integrate -> collide |
                    | -> rigid -> stats    |
                    +----------------------+
```

The main executable links `engine_core`, `cli`, and `engine_tui`. The
SDL2/OpenGL renderer lives in `engine_gl`, which is built and tested separately
but is not yet linked into the main executable.

## 2. Source map

```text
gpu_engine/
  src/
    main.cpp              mode selection and top-level error handling
    cli/                  command-line parser and help text
    sim/
      particles.*         particle SoA, deterministic init, forces, integration
      grid.*              uniform grid setup, counting sort, CUB scan
      collisions.*        particle collision impulses and plane contacts
      rigid.*             rigid-body state, contacts, Jacobi solve
      simulation.*        scene lifetime, step orchestration, density sampling
      density.*           particle/body density projection
      sim_params.h        SimParams, InitParams, SimStats
      tuning.h            block size and kernel-selection constants
    app/
      runner.*            worker thread, command queue, telemetry publishing
      telemetry.*         snapshot types, history ring buffer, JSONL
      headless.*          finite headless run loop
      stats_log.*         CSV statistics writer
      dump.*              binary PPM density writer
      probe.*             ray/sphere probe and neighbor count
    render/
      camera.*            orbit camera and ray generation
      shader.*            shader compile/link helpers
      renderer.*          particle VBO and point/grid rendering
      interop.*           CUDA/OpenGL registration and fallback pack kernel
      rigid_renderer.*    instanced sphere/box rendering
    tui/
      app.*               FTXUI screen, input handling, runner wiring
      view.*              dashboard and help overlay
      field.*             ASCII density field and cell-to-ray mapping
      format.*            human-readable number formatting
      theme.*             semantic colors and terminal capability detection
      terminal_guard.*    alternate-screen RAII guard
    util/
      math.h              host/device Vec3, Quat, Mat3, Mat4
      cuda_check.h        CUDA error macro
      device_info.cu      --info implementation
  tests/                  CPU-reference and GPU correctness tests
  bench/                  standalone stage benchmarks and autotune matrix
  docs/perf.md            measured timings and optimization notes
```

## 3. Build graph

| Target | Kind | Contents | Links |
|---|---|---|---|
| `engine_core` | static library | `sim/`, `app/`, `util/` | `CUDA::cudart` |
| `engine_gl` | static library | `render/` | `engine_core`, `SDL2::SDL2`, `OpenGL::GL` |
| `cli` | static library | `cli/options.cpp` | `engine_core` |
| `engine_tui` | static library | `tui/` | `engine_core`, FTXUI |
| `gpu_engine` | executable | `main.cpp` | `engine_core`, `cli`, `engine_tui` |
| `test_*` | executables | one test per source | the library under test |
| `bench_*` | executables | stage benchmarks | `engine_core` |

Build options:

- `TRON_BUILD_GL=ON` (default) builds the SDL2/OpenGL library and its tests.
- `TRON_FETCH_DEPS=ON` (default) fetches FTXUI v7.0.3 with `FetchContent`.
- `TRON_FETCH_DEPS=OFF` expects an installed `ftxui` CMake package.

CUDA separable compilation is enabled, and the architecture is pinned in
`gpu_engine/CMakeLists.txt`:

```cmake
set(CMAKE_CUDA_ARCHITECTURES 86)
```

## 4. Runtime paths

### CLI parsing

`parse_cli` is a single pass over `argv`. It recognizes commands, flags, and
options, then fills `Options`. The parser is intentionally strict: unknown
options, trailing junk in integer values, and missing values throw `CliError`
with exit code 2.

`resolve_mode` handles the no-command case:

- interactive stdin and stdout -> TUI;
- run flags with a non-TTY output -> headless simulation;
- otherwise -> help.

### Headless path

`run_headless`:

1. copies the CLI scene values into a `SceneConfig`;
2. applies the `dense` preset;
3. calls `Simulation::init`;
4. steps a finite number of frames;
5. emits text or JSONL telemetry;
6. optionally writes CSV rows and a PPM density image;
7. prints a summary.

`--bench` enables CUDA event timing in `Simulation::step`; otherwise the timing
fields remain zero.

### TUI path

`TuiApp::run`:

1. creates a `TerminalGuard`;
2. detects the theme from `--no-color`, `--ascii`, `TERM`, `NO_COLOR`, and the
   locale;
3. starts a `SimulationRunner` with the CLI scene and log path;
4. creates an FTXUI fullscreen screen and a 30 Hz ticker;
5. polls telemetry on repaint and renders the dashboard or help overlay;
6. translates keys into runner commands;
7. requests a final dump if `--dump` was supplied, then stops the runner.

The UI thread never calls CUDA directly. The runner owns the `Simulation`, the
CSV writer, and the density sampling.

## 5. Simulation step

`Simulation::step` is the central pipeline. With timing enabled, each stage is
wrapped in CUDA events.

```text
1. D2D snapshot: posMass/vel -> posPrev/velPrev
2. Grid setup:    AABB reduction -> dimensions -> clamp -> allocate
3. Grid build:    clear counts -> count -> CUB exclusive sum -> place
4. Forces:        gravity + drag
5. Integrate:     semi-implicit Euler
6. Collisions:    grid-neighbor impulses + plane contact
7. Rigid bodies:  integrate -> generate contacts -> solve 4 Jacobi passes
8. Stats:         kinetic, potential, momentum, first non-finite index
9. frame++
```

Important details:

- `posPrev` and `velPrev` are the broadphase snapshot. Collisions read them
  while the live state is written, so the step is not reading partially
  integrated positions.
- The grid is rebuilt from `posPrev` every step. `cellStart` is the exclusive
  prefix sum of `cellCount`; `sortedIdx` holds original particle indices, not
  reordered particle data.
- The rigid-body path uses the same grid built from particles. Contacts are
  generated on the device, the count is copied to the host, and the solve runs
  only when at least one contact exists.
- `StepOptions::collect_stats` controls the O(N) stats reduction. The TUI and
  runner request it every step; callers that only need motion can disable it.
- `StepOptions::collect_timings` controls CUDA event recording. Event creation
  and synchronization add overhead, so leave it off outside benchmarks and
  `--bench`.

## 6. Core data structures

### `ParticleSystem`

Structure-of-arrays GPU buffers:

| Buffer | Layout |
|---|---|
| `posMass` | `xyz` position, `w` inverse mass |
| `vel` | `xyz` velocity, `w` radius |
| `posPrev`, `velPrev` | previous-step snapshot |
| `force` | accumulated force |
| `material` | integer material id |

Inverse mass is stored so force integration can multiply. A non-positive
inverse mass means static or infinite mass; those particles are skipped by
forces, integration, and stats.

### `UniformGrid`

| Field | Meaning |
|---|---|
| `min`, `max` | AABB of the current particle cloud |
| `cellSize` | Requested cell size, clamped to at least `1e-6` |
| `dims` | Cell counts on each axis |
| `numCells` | Usable cell count after clamping |
| `clamped` | Whether the requested grid exceeded `max_cells` |
| `cellCount` | Particles per cell |
| `cellStart` | Exclusive prefix sum over `cellCount` |
| `cellCursor` | Scatter cursor, copied from `cellStart` |
| `sortedIdx` | Particle indices grouped by cell |
| `scanStorage` | CUB scratch buffer |
| `aabbScratch` | Six floats for the device AABB reduction |

The grid warns once when it clamps, not every frame. A clamped grid is a
configuration problem, not a transient condition.

### `RigidBodySoA`

Rigid bodies use the same SoA idea:

- `pos`: `xyz` position, `w` inverse mass;
- `quat`: orientation `xyzw`;
- `vel`: linear velocity;
- `omega`: angular velocity;
- `invInertiaBody`: body-space inverse inertia;
- `halfExtents`: shape half-extents;
- `shape`: sphere or box.

`Contact` stores two body indices (`b == -1` means a body-plane contact), the
contact point, the normal, and the penetration depth.

### `Telemetry` and `History`

`Telemetry` is an immutable snapshot published by the worker and copied by the
UI under a mutex. It contains frame counters, timing, physics stats, contact
stats, density, probe state, logging state, and recent events.

`History` is a fixed 240-sample ring buffer. `percentile` copies and sorts the
samples; this is fine at UI rates and keeps the implementation simple.

### `DensityGrid`

`DensityGrid` is a host-side projection:

- `width`, `height`, `plane`, and visible bounds;
- `particles`: per-cell particle count;
- `bodies`: per-cell body occupancy;
- `max_count`: observed maximum, used for log scaling.

The device density kernels write into device buffers; `sample_density` exposes
the host vectors for the TUI and PPM writer.

## 7. GPU kernels and algorithms

### Initialization

`init_kernel` uses SplitMix64-derived random values from `(seed, index,
stream)`. It fills positions inside `InitParams::boxMin..boxMax`, assigns a
mass in `massMin..massMax`, stores inverse mass, and gives each particle a small
random velocity.

### Forces and integration

`forces_kernel` computes:

```text
force = mass * gravity - drag * velocity
```

`integrate_kernel` is semi-implicit Euler:

```text
velocity += dt * force * inverse_mass
position += dt * velocity
```

### Uniform-grid broadphase

`launch_grid_setup`:

1. initializes the AABB scratch to `(+FLT_MAX, -FLT_MAX)`;
2. runs `reduce_aabb_kernel` with one block;
3. copies the AABB to the host;
4. computes dimensions from the AABB and `cell_size`;
5. clamps to `max_cells`;
6. allocates or reuses grid buffers;
7. emits a one-time clamp warning if needed.

`launch_grid_build`:

1. clears `cellCount`;
2. counts particles per cell with atomics;
3. synchronizes before CUB queries scratch size;
4. runs `cub::DeviceScan::ExclusiveSum`;
5. copies the total particle count to `cellStart[numCells]`;
6. copies `cellStart` to `cellCursor`;
7. scatters particle indices into `sortedIdx`.

The `cudaStreamSynchronize` before the CUB query is required because CUB asks
for scratch size on the host while the count kernel may still be running.

### Particle collisions

`collision_kernel` runs one thread per particle:

1. read the previous position and velocity;
2. locate the particle's cell;
3. scan the 3x3x3 neighboring cells;
4. skip self and non-positive radii;
5. compute the normal and relative velocity;
6. apply a restitution impulse when approaching;
7. clamp the tangent impulse with Coulomb friction;
8. accumulate a positional correction;
9. apply the floor contact;
10. write the new position and velocity only for dynamic particles.

Contact counters are incremented on both sides of a pair, so the host halves
them.

`launch_collisions_ilp` and `launch_collisions_smem` are benchmark entry points
that currently alias the scalar kernel. The ILP and SMEM implementations were
not retained because benchmarking did not justify them.

### Rigid bodies

`launch_rigid_integrate` advances position and quaternion orientation. The
quaternion derivative is:

```text
q_dot = 0.5 * omega_quat * q
```

followed by renormalization.

`launch_contacts_generate` reserves contact slots with `atomicAdd`; overflow is
dropped rather than writing past the buffer. `launch_contacts_solve` runs four
Jacobi-style passes. Each pass reads the same state, then all corrections are
applied together.

### Statistics

`launch_stats` initializes the device stats, runs the reduction, and converts
the `INT_MAX` sentinel back to `-1`. The first non-finite index is tracked with
`atomicMin` so the UI can point at a concrete particle.

### Density

`launch_density` projects particles and bodies onto either `TOP XZ` or
`SIDE XY`. Out-of-view particles are rejected rather than clamped, because
clamping would put false density in the edge bins. The TUI and PPM writer use
log scaling to keep dense clusters readable.

## 8. Threading model

`SimulationRunner` is the boundary between the UI and CUDA:

- `start` launches `worker_main` and waits for initialization to finish.
- Public setters enqueue small `Command` values.
- The worker drains the whole command queue once per loop iteration.
- `poll` copies the latest `Telemetry` snapshot under `snapshot_mutex_`.
- `stop` enqueues `Quit` and joins the worker.

The worker loop:

1. wait up to 1 ms for commands;
2. apply commands;
3. accumulate wall-clock time and run at most two simulation substeps;
4. write CSV rows if logging is enabled;
5. publish a snapshot at `stats_hz`.

A slow frame slows the simulation instead of spiraling into an unbounded catch-up
loop.

`publish(bool force)` is currently a no-op stub. The real publishing happens in
the `publish_snapshot` lambda inside `worker_main`.

## 9. Tests

Run the full suite:

```bash
ctest --test-dir build --output-on-failure
```

Run one test:

```bash
ctest --test-dir build -R test_grid --output-on-failure
```

Or run the executable directly:

```bash
./build/test_grid
```

| Test | Covers |
|---|---|
| `test_smoke` | CUDA runtime and basic allocation |
| `test_math` | Vec3, Quat, Mat3, Mat4 operations |
| `test_particles` | Deterministic initialization and SoA layout |
| `test_forces` | Gravity, drag, static-particle handling |
| `test_grid` | AABB, dimensions, counting sort, clamping |
| `test_collisions` | Particle impulses and plane contact |
| `test_rigid` | Rigid integration, contacts, Jacobi solve |
| `test_observability` | Probe and CSV statistics |
| `test_simulation` | Scene init, step, reset, device budget, clamp, NaN detection |
| `test_density` | Top projection and max-count tracking |
| `test_telemetry` | History ring buffer and JSONL |
| `test_headless` | Headless run and log-path failure |
| `test_dump` | PPM output |
| `test_runner` | Worker start/stop, commands, polling |
| `test_cli` | Parsing, defaults, mode resolution |
| `test_tui_view` | Dashboard rendering |
| `test_probe` | Ray/sphere probe and neighbor count |
| `test_tui_smoke` | TUI starts and restores the cursor; skipped without `script` |
| `test_render` | SDL2/OpenGL renderer (GL build only) |
| `test_interop` | CUDA/OpenGL registration and fallback (GL build only) |
| `test_rigid_render` | Instanced rigid rendering (GL build only) |

The TUI smoke test runs:

```bash
TERM=xterm-256color script -qec \
  "./build/gpu_engine tui --particles 1000 --bodies 10 --frames 2 --no-color --ascii" \
  /dev/null
```

and checks that the terminal restore sequence is emitted. It returns 77 when
`script` is unavailable, which CTest treats as a skip.

## 10. Benchmarks

The benchmark targets are built but not registered with CTest. Run them from
the build directory.

```bash
# Forces + integration
./build/bench_linear 1000000 100

# Grid setup + build
./build/bench_grid 1000000

# Collisions
./build/bench_collisions 100000 10

# Full frame stages
./build/bench_frame 1000000
```

`bench_linear [particles] [iterations]` reports milliseconds per step.

`bench_grid [particles]` reports grid build time, cell count, and clamp status.

`bench_collisions [particles] [iterations]` reports milliseconds per step and
the final pair/contact counts.

`bench_frame [particles]` runs 100 warmup frames and 300 measured frames by
default. The measured-frame count is the third argument; the second argument is
currently unused.

`gpu_engine/bench/autotune.sh` writes a configuration matrix to
`gpu_engine/bench/results.csv`: 3 block sizes, 3 particles-per-thread values,
3 cell-size factors, and 2 SMEM settings. The timing column is currently
`not-measured`; the script is a harness for future target-machine measurements.

Measured numbers and negative results live in
[`gpu_engine/docs/perf.md`](../gpu_engine/docs/perf.md).

## 11. Extending the engine

### Add a force

1. Add the parameter to `SimParams` in `src/sim/sim_params.h`.
2. Update `forces_kernel` in `src/sim/forces.cu`.
3. Add a CPU-reference test in `tests/test_forces.cu`.
4. Expose the parameter through the CLI if it should be user-tunable.
5. Add it to the TUI only if it is stable enough for the dashboard.

### Add a GPU kernel

1. Put the launch declaration in the relevant header.
2. Implement the kernel in the matching `.cu` file.
3. Use `CUDA_CHECK` after every launch and runtime call.
4. Launch on the stream passed by the caller; do not use the default stream.
5. Add a test with a small deterministic input.
6. Add a benchmark only when the kernel is on a hot path.

### Add a CLI option

1. Add the field to `Options` in `src/cli/options.h`.
2. Parse it in `src/cli/options.cpp`; reject bad values with `CliError(2, ...)`.
3. Update `help_text()`.
4. Add a parser test in `tests/test_cli.cpp`.
5. Thread it into `SceneConfig` or the runner configuration.

### Add a TUI panel

1. Add the data to `Telemetry` if the worker must publish it.
2. Publish it in the `publish_snapshot` lambda inside
   `SimulationRunner::worker_main`.
3. Render it in `src/tui/view.cpp`.
4. Keep the view pure: no CUDA calls and no runner calls from the render path.
5. Add or update `tests/test_tui_view.cpp`.

## 12. Debugging

Useful commands:

```bash
# Device and driver information
./build/gpu_engine --info

# JSONL telemetry for a short run
./build/gpu_engine sim --particles 1000 --bodies 10 --frames 3 --json

# Compute Sanitizer
compute-sanitizer ./build/test_grid
compute-sanitizer ./build/test_collisions

# Nsight Compute on the collision kernel
ncu --set full --kernel-name regex:collisions ./build/bench_collisions 100000 1

# CUDA debugger
cuda-gdb ./build/test_grid
```

Useful environment variables:

- `GPU_ENGINE_DISABLE_INTEROP=1` forces the host-upload fallback in the
  CUDA/OpenGL interop path.
- `NO_COLOR=1` disables TUI color.
- `TERM=dumb` also disables TUI color.

When a kernel produces wrong results, first check:

1. stream ordering and missing synchronizations;
2. host/device pointer direction in `cudaMemcpy`;
3. grid cell clamping and boundary checks;
4. whether the kernel is reading `posPrev` versus `posMass`;
5. whether a launch error is being swallowed.

## 13. Known issues and next work

- The SDL2/OpenGL renderer is a library with tests, not part of the main
  executable yet. The TUI uses the ASCII density field.
- The TUI footer lists `P` for probe, but no key handler sends a probe request.
  Mouse picking is also not wired.
- The headless `--bench` stage table is a placeholder. Use `bench_*` for real
  per-stage numbers.
- `SimulationRunner::publish(bool force)` is a no-op stub; publishing happens
  through `publish_snapshot` in `worker_main`.
- `Telemetry::sim_hz` is declared but not populated.
- `reduce_aabb_kernel` calls `__syncthreads()` inside the `if (lane < stride)`
  branch. That is a divergent barrier and should be fixed before relying on the
  reduction for arbitrary block sizes. The current one-block launch and tests
  have not exposed it consistently.
- The `dense` preset overrides explicit particle and body counts. It is a
  preset, not a set of defaults.
- `--log` and `--dump` require existing parent directories.
- Several tests (`test_runner`, `test_probe`, `test_tui_view`) use `assert`.
  A Release build defines `NDEBUG`, which turns those checks into no-ops. Run
  the suite in a Debug or RelWithDebInfo build, or convert the tests to the
  `CHECK` helper in `tests/test_util.h`.
- Particles and rigid bodies are not two-way coupled.
- The collision baseline is intentionally unoptimized and is kept as a
  reference point.

## 14. Conventions

- C++17 and CUDA C++17.
- Host/device helpers use `HD` from `util/math.h`.
- GPU state is structure-of-arrays; avoid array-of-structures in hot paths.
- Pass `cudaStream_t` explicitly and launch on it.
- Use `CUDA_CHECK` for runtime calls and `cudaGetLastError()` after kernel
  launches.
- Keep host/device copies explicit; document whether a function synchronizes.
- Prefer small, named kernels over one large kernel with mode flags.
- Keep comments focused on non-obvious behavior, invariants, and tradeoffs.
- Do not silently clamp or drop data without reporting it once.

## 15. License

MIT. See [`../LICENSE`](../LICENSE).
