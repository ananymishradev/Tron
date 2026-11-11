cmake_minimum_required(VERSION 3.24)
project(gpu_engine LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

find_package(CUDAToolkit REQUIRED)
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

set(ENGINE_SOURCES
  src/util/device_info.cu
  src/sim/particles.cu
  src/sim/forces.cu
  src/sim/integrate.cu
  src/sim/stats.cu
  src/sim/grid.cu
  src/sim/collisions.cu
  src/render/camera.cpp
  src/render/shader.cpp
  src/render/renderer.cpp
  src/render/interop.cu
  src/render/rigid_renderer.cpp
  src/sim/rigid.cu
  src/app/app.cpp
  src/app/input.cpp
  src/app/probe.cpp
  src/app/stats_log.cpp
)

add_library(engine STATIC ${ENGINE_SOURCES})
target_include_directories(engine PUBLIC src third_party)
target_link_libraries(engine PUBLIC CUDA::cudart SDL2::SDL2 OpenGL::GL)

add_executable(gpu_engine src/main.cpp)
target_link_libraries(gpu_engine PRIVATE engine)

enable_testing()
function(add_engine_test name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE engine)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_engine_test(test_smoke tests/test_smoke.cu)
add_engine_test(test_math tests/test_math.cu)
add_engine_test(test_particles tests/test_particles.cu)
add_engine_test(test_forces tests/test_forces.cu)
add_engine_test(test_grid tests/test_grid.cu)
add_engine_test(test_collisions tests/test_collisions.cu)
add_engine_test(test_render tests/test_render.cpp)
add_engine_test(test_interop tests/test_interop.cu)
add_engine_test(test_rigid tests/test_rigid.cu)
add_engine_test(test_rigid_render tests/test_rigid_render.cpp)
add_engine_test(test_observability tests/test_observability.cpp)
add_executable(bench_linear bench/bench_linear.cu)
target_link_libraries(bench_linear PRIVATE engine)
add_executable(bench_grid bench/bench_grid.cu)
target_link_libraries(bench_grid PRIVATE engine)
add_executable(bench_collisions bench/bench_collisions.cu)
target_link_libraries(bench_collisions PRIVATE engine)
add_executable(bench_frame bench/bench_frame.cu)
target_link_libraries(bench_frame PRIVATE engine)
