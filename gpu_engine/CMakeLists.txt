cmake_minimum_required(VERSION 3.24)
project(gpu_engine VERSION 0.2.0 LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 86)
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

option(TRON_BUILD_GL "Build the SDL2/OpenGL renderer and GL tests" ON)
option(TRON_FETCH_DEPS "Fetch FTXUI at configure time" ON)

find_package(CUDAToolkit REQUIRED)
include(FetchContent)
if(TRON_FETCH_DEPS)
  set(FTXUI_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(FTXUI_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(FTXUI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(FTXUI_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  set(FTXUI_QUIET ON CACHE BOOL "" FORCE)
  FetchContent_Declare(ftxui
    GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI.git
    GIT_TAG v7.0.3)
  FetchContent_MakeAvailable(ftxui)
else()
  find_package(ftxui CONFIG REQUIRED)
endif()

add_library(engine_core STATIC
  src/util/device_info.cu
  src/sim/particles.cu
  src/sim/forces.cu
  src/sim/integrate.cu
  src/sim/stats.cu
  src/sim/grid.cu
  src/sim/collisions.cu
  src/sim/rigid.cu
  src/sim/simulation.cpp
  src/sim/density.cu
  src/app/probe.cpp
  src/app/stats_log.cpp
  src/app/telemetry.cpp
  src/app/headless.cpp
  src/app/dump.cpp
  src/app/runner.cpp
)
target_include_directories(engine_core PUBLIC src third_party)
target_link_libraries(engine_core PUBLIC CUDA::cudart)
add_library(engine ALIAS engine_core)

if(TRON_BUILD_GL)
  find_package(SDL2 REQUIRED)
  find_package(OpenGL REQUIRED)
  add_library(engine_gl STATIC
    src/render/camera.cpp
    src/render/shader.cpp
    src/render/renderer.cpp
    src/render/interop.cu
    src/render/rigid_renderer.cpp)
  target_include_directories(engine_gl PUBLIC src third_party)
  target_link_libraries(engine_gl PUBLIC engine_core SDL2::SDL2 OpenGL::GL)
endif()

add_executable(gpu_engine src/main.cpp)
target_link_libraries(gpu_engine PRIVATE engine_core)

add_library(cli STATIC src/cli/options.cpp)
target_include_directories(cli PUBLIC src)
target_link_libraries(cli PUBLIC engine_core)
target_link_libraries(gpu_engine PRIVATE cli)
add_library(engine_tui STATIC
  src/tui/theme.cpp src/tui/format.cpp src/tui/field.cpp
  src/tui/view.cpp src/tui/app.cpp src/tui/terminal_guard.cpp)
target_include_directories(engine_tui PUBLIC src)
target_link_libraries(engine_tui PUBLIC engine_core
  ftxui::screen ftxui::dom ftxui::component)
target_link_libraries(gpu_engine PRIVATE engine_tui)

enable_testing()
function(add_engine_test name source)
  add_executable(${name} ${source})
  target_link_libraries(${name} PRIVATE engine_core)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_engine_test(test_smoke tests/test_smoke.cu)
add_engine_test(test_math tests/test_math.cu)
add_engine_test(test_particles tests/test_particles.cu)
add_engine_test(test_forces tests/test_forces.cu)
add_engine_test(test_grid tests/test_grid.cu)
add_engine_test(test_collisions tests/test_collisions.cu)
add_engine_test(test_rigid tests/test_rigid.cu)
add_engine_test(test_observability tests/test_observability.cpp)
add_engine_test(test_simulation tests/test_simulation.cu)
add_engine_test(test_density tests/test_density.cu)
add_engine_test(test_telemetry tests/test_telemetry.cpp)
add_engine_test(test_headless tests/test_headless.cpp)
add_engine_test(test_dump tests/test_dump.cpp)
add_engine_test(test_runner tests/test_runner.cpp)
add_executable(test_cli tests/test_cli.cpp)
target_link_libraries(test_cli PRIVATE cli)
add_test(NAME test_cli COMMAND test_cli)
add_executable(test_tui_view tests/test_tui_view.cpp)
target_link_libraries(test_tui_view PRIVATE engine_tui)
add_test(NAME test_tui_view COMMAND test_tui_view)
add_engine_test(test_probe tests/test_probe.cpp)
add_test(NAME test_tui_smoke
  COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_tui_smoke.sh
          $<TARGET_FILE:gpu_engine>)
set_tests_properties(test_tui_smoke PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)

if(TRON_BUILD_GL)
  add_executable(test_render tests/test_render.cpp)
  target_link_libraries(test_render PRIVATE engine_gl)
  add_test(NAME test_render COMMAND test_render)
  add_executable(test_interop tests/test_interop.cu)
  target_link_libraries(test_interop PRIVATE engine_gl)
  add_test(NAME test_interop COMMAND test_interop)
  add_executable(test_rigid_render tests/test_rigid_render.cpp)
  target_link_libraries(test_rigid_render PRIVATE engine_gl)
  add_test(NAME test_rigid_render COMMAND test_rigid_render)
endif()

foreach(bench bench_linear bench_grid bench_collisions bench_frame)
  add_executable(${bench} bench/${bench}.cu)
  target_link_libraries(${bench} PRIVATE engine_core)
endforeach()
