# Tron quick user guide

Tron is a CUDA particle and rigid-body playground. You can run it as a
terminal dashboard or as a headless simulation that writes telemetry, CSV
statistics, and density images.

This guide covers the current build and the commands that are wired up today.
For internals, see the [developer guide](developer-guide.md).

## 1. Requirements

- Linux (the supported platform)
- An NVIDIA GPU and a working driver
- CUDA toolkit (the project is tested with CUDA 13.4)
- CMake 3.24 or newer
- SDL2 and OpenGL development headers for the GL renderer and GL tests
- Network access on the first configure, unless FTXUI is already installed

On Arch Linux:

```bash
sudo pacman -S cuda sdl2 cmake
```

`script` from util-linux is useful for the TUI smoke test, but it is not needed
to build or run the engine.

## 2. Build

From the repository root:

```bash
cmake -S gpu_engine -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Check that CUDA sees the device:

```bash
./build/gpu_engine --info
```

### Headless-only build

If you do not need the SDL2/OpenGL renderer or its tests:

```bash
cmake -S gpu_engine -B build -DCMAKE_BUILD_TYPE=Release -DTRON_BUILD_GL=OFF
cmake --build build -j
```

### FTXUI dependency

By default CMake fetches FTXUI v7.0.3 at configure time. If FTXUI is already
installed and you want to avoid the download:

```bash
cmake -S gpu_engine -B build -DTRON_FETCH_DEPS=OFF
```

### GPU architecture

`gpu_engine/CMakeLists.txt` currently pins:

```cmake
set(CMAKE_CUDA_ARCHITECTURES 86)
```

If your GPU is not compute capability 8.6, edit that line before configuring.
A mismatch usually shows up as "no kernel image is available for execution on
the device" at runtime.

If you change build options, remove or reconfigure the build directory so CMake
does not keep stale cache values.

## 3. First run

Start the terminal dashboard:

```bash
./build/gpu_engine
```

With a terminal on stdin and stdout, no arguments selects the TUI. You can also
be explicit:

```bash
./build/gpu_engine tui
```

The default scene is large: 1,000,000 particles and 10,000 rigid bodies. For a
first run, use a smaller scene:

```bash
./build/gpu_engine tui --particles 200000 --bodies 2000
```

Run a short headless simulation:

```bash
./build/gpu_engine sim --particles 100000 --bodies 1000 --frames 120
```

The headless path prints a per-frame line at the configured statistics rate,
then a summary.

## 4. Commands and mode selection

| Command | Meaning |
|---|---|
| `tui` | Terminal dashboard |
| `sim` | Headless simulation |
| `bench` | Headless simulation with stage timing collection |
| `info` | Print CUDA device information |
| `help` | Print command help |
| `version` | Print the version |
| `--headless` | Alias for `sim` |
| `--tui` | Alias for `tui` |

If no command is given:

1. An interactive terminal (stdin and stdout are TTYs) starts the TUI.
2. Run flags such as `--particles` or `--frames` with a non-TTY output select
   headless simulation.
3. Otherwise Tron prints help.

## 5. Command-line options

| Option | Default | Meaning |
|---|---:|---|
| `--particles N` | 1,000,000 | Particle count |
| `--bodies N` | 10,000 | Rigid-body count |
| `--seed N` | 42 | Deterministic scene seed |
| `--frames N` | 0 | Frame limit. Headless: `0` means one frame. TUI: `0` means run until quit |
| `--duration SEC` | 0 | Run duration converted to frames at 60 Hz; cannot be combined with `--frames` |
| `--scene sparse\|dense` | `sparse` | Scene preset. `dense` uses 200,000 particles and no rigid bodies |
| `--bench` | off | Collect stage timings and print the stage table |
| `--json` | off | Emit one JSON object per frame, plus a summary object |
| `--log PATH` | empty | Write per-frame CSV statistics |
| `--dump PATH` | empty | Write a top-down density image in binary PPM format |
| `--stats-hz N` | 10 | Statistics/publish rate |
| `--quiet` | off | Suppress the per-frame text lines (the summary is still printed) |
| `--no-color` | off | Disable color in the TUI |
| `--ascii` | off | Use ASCII-only TUI output |
| `--info` | — | Print CUDA device information |
| `--help`, `-h` | — | Print help |
| `--version` | — | Print the version |

Notes:

- `--duration 2` is converted to 120 frames at 60 Hz.
- `--scene dense` is a fixed preset. It overrides the particle and body counts
  with 200,000 particles and zero bodies.
- `--frames` and `--duration` are mutually exclusive.
- `--json` is a headless output mode; the TUI does not emit JSONL.
- Invalid values, unknown options, and missing option values exit with status 2.

## 6. TUI guide

The TUI starts a worker thread that owns the CUDA simulation. The UI thread
only reads telemetry snapshots and sends commands to that worker.

The dashboard shows:

- **Header** — run state, particle count, current frame.
- **Performance** — frame rate, simulation time, frame time, and the 50th
  percentile of recent simulation times.
- **Physics** — kinetic energy, potential energy, broadphase pair tests,
  contacts, grid cell count, device memory estimate, and a state check.
- **Field** — an ASCII density view of the current top (`TOP XZ`) or side
  (`SIDE XY`) projection. The shading is log-scaled.

### Keys

| Key | Action |
|---|---|
| `Space` | Pause or resume |
| `.` | Advance one simulation step while paused |
| `R` / `r` | Reset the scene with the current seed |
| `L` / `l` | Toggle CSV logging |
| `V` / `v` | Toggle top and side field views |
| `?` | Toggle the help overlay |
| `Q` / `q` / `Esc` / `Ctrl-C` | Quit |

`L` only writes rows if a log file was opened at startup with `--log PATH`.
There is no file picker inside the TUI.

The footer also displays `P probe`, but the probe key is not connected in this
build. Probe support exists in the runner and in the field code, but the TUI
does not yet send a ray from a mouse or keyboard action.

### TUI examples

```bash
# Smaller scene with CSV logging and no color
./build/gpu_engine tui --particles 200000 --bodies 2000 \
  --log tui.csv --no-color

# Run for 300 simulation frames, then write a density image
./build/gpu_engine tui --particles 100000 --bodies 1000 \
  --frames 300 --dump tui-density.ppm

# ASCII-only output
./build/gpu_engine tui --particles 100000 --bodies 0 --ascii
```

The dashboard targets a terminal around 100x30. Smaller terminals may clip the
panels because the current layout uses a fixed 100x30 field area.

## 7. Headless examples

```bash
# Short run with text output
./build/gpu_engine sim --particles 100000 --bodies 1000 --frames 120

# One JSON object per frame, plus a summary object
./build/gpu_engine sim --particles 200000 --bodies 2000 --frames 300 \
  --json > telemetry.jsonl

# CSV statistics and a density image
./build/gpu_engine sim --particles 200000 --bodies 2000 --frames 300 \
  --log run.csv --dump density.ppm

# Timing run
./build/gpu_engine sim --particles 50000 --bodies 0 --frames 60 --bench

# Dense preset
./build/gpu_engine sim --scene dense --frames 60 --json > dense.jsonl
```

## 8. Output files

### JSONL telemetry (`--json`)

Each frame emits a JSON object with `"type":"telemetry"`. The run ends with a
summary object:

```json
{"type":"summary","frames":120,"nan":-1}
```

The telemetry object includes:

| Field | Meaning |
|---|---|
| `sequence` | Monotonic publish sequence |
| `frame` | Simulation frame |
| `elapsed_s` | Wall-clock seconds since the worker started |
| `sim_ms` | Simulation step time; populated when timing collection is on |
| `frame_ms` | Wall-clock frame delta |
| `fps` | Reciprocal of the frame delta |
| `sim_hz` | Reserved; currently not populated |
| `substeps` | Simulation substeps in the last loop |
| `paused` | Pause state |
| `particles`, `bodies` | Scene counts |
| `pairs`, `contacts` | Broadphase pair tests and contacts |
| `kinetic`, `potential` | Energy totals |
| `momentum` | Momentum vector |
| `nan` | First non-finite particle index, or `-1` |
| `grid_cells` | Grid cells in use |
| `grid_clamped` | Whether the grid was clamped |
| `device_bytes` | Estimated device allocation |
| `probe_valid` | Whether a probe result is present |
| `log_path` | Configured CSV path |

`sim_ms` is only meaningful when timing collection is enabled. In the headless
path, that means passing `--bench`.

### CSV statistics (`--log`)

The CSV header is:

```text
frame,sim_ms,render_ms,fps,particles,bodies,pairs,contacts,kinetic,potential
```

One row is written per frame. `render_ms` is currently `0` in the headless
path. `sim_ms` is populated when `--bench` enables stage timing collection.

### Density image (`--dump`)

`--dump` writes a binary PPM (`P6`) image. The headless path requests a
top-down `TOP XZ` projection, but the sampler clamps density grids to 256x64,
so the resulting image is 256x64. The image is grayscale:

- empty cells are black;
- particle density is log-scaled;
- cells containing rigid bodies are white.

The output directory must already exist. In the headless path, a failed open
exits with status 2.

### Stage table (`--bench`)

`--bench` enables CUDA event timing and prints a coarse stage table. The current
headless table is a placeholder: it reports the total step time in the `grid`
row and leaves the other stage rows at zero. Use the standalone benchmark
targets in the [developer guide](developer-guide.md#benchmarks) for real
per-stage numbers.

## 9. Exit codes

| Code | Meaning |
|---:|---|
| `0` | Success |
| `1` | Runtime failure, including TUI startup failure |
| `2` | CLI error, invalid scene, allocation failure, or an output file that could not be opened |

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| `no kernel image is available` | The GPU architecture does not match `CMAKE_CUDA_ARCHITECTURES`; edit it and rebuild |
| `estimated allocation exceeds 4 GB device budget` | Reduce `--particles`, `--bodies`, or the grid/contact limits in `SceneConfig` |
| `insufficient device memory` | Reduce the scene size or close other GPU processes |
| `warning: uniform grid clamped` | The AABB and cell size request more than `max_cells`; raise `max_cells`, increase `cell_size`, or shrink the scene bounds |
| TUI does not start | Run it from an interactive terminal, or use `./build/gpu_engine tui` |
| TUI colors look wrong | Try `--no-color`, `--ascii`, or set a UTF-8 locale |
| FTXUI configure failure | Install FTXUI v7 and configure with `-DTRON_FETCH_DEPS=OFF` |
| GL tests fail without a display | Configure with `-DTRON_BUILD_GL=OFF` or run with a working display |
| `--log` or `--dump` fails | Make sure the parent directory exists and is writable |
| TUI probe does nothing | The `P` key is not wired yet; probe support is library-level for now |

## 11. Current limits

Tron is deliberately focused:

- the main executable runs the terminal TUI or a headless simulation; the
  SDL2/OpenGL renderer is built and tested as a library but is not yet wired
  into the main interactive path;
- the TUI has no mouse camera or mouse picking;
- the `P` probe key shown in the TUI footer is not connected;
- particles and rigid bodies are not two-way coupled;
- there are no fluids, cloth, soft bodies, or box-box contacts;
- uniform gravity only; no long-range N-body gravity;
- Linux is the supported platform;
- the particle collision baseline is intentionally slow and is kept as a
  reference for future optimization.

For measured performance and the optimization history, see
[`gpu_engine/docs/perf.md`](../gpu_engine/docs/perf.md).
