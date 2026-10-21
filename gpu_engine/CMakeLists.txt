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
