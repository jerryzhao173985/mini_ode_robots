# mini_ode_robots — CMake build (portable alternative to the Makefile)
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
#
# Finds ODE via pkg-config (ode-config also works). Only external dep: libode.
cmake_minimum_required(VERSION 3.10)
project(mini_ode_robots CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra -Wno-unused-parameter -Wno-deprecated-declarations)

# --- locate ODE (double precision) ---
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ODE ode)
endif()
if(NOT ODE_FOUND)
  # fall back to ode-config
  execute_process(COMMAND ode-config --cflags OUTPUT_VARIABLE ODE_CFLAGS_RAW OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND ode-config --libs   OUTPUT_VARIABLE ODE_LDFLAGS    OUTPUT_STRIP_TRAILING_WHITESPACE)
  separate_arguments(ODE_CFLAGS_RAW)
  separate_arguments(ODE_LDFLAGS)
endif()

# --- the engine library ---
file(GLOB MOR_SRC CONFIGURE_DEPENDS src/*.cpp robots/*.cpp)
add_library(mor STATIC ${MOR_SRC})
target_include_directories(mor PUBLIC include robots ${ODE_INCLUDE_DIRS})
if(ODE_CFLAGS_RAW)
  target_compile_options(mor PUBLIC ${ODE_CFLAGS_RAW})
endif()
target_link_libraries(mor PUBLIC ${ODE_LIBRARIES} ${ODE_LDFLAGS})
if(ODE_LIBRARY_DIRS)
  target_link_directories(mor PUBLIC ${ODE_LIBRARY_DIRS})
endif()

# --- examples + tests ---
enable_testing()
foreach(ex test_math selfcheck test_features demo_robot demo_arm demo_snake demo_sensors demo_hexapod)
  add_executable(${ex} examples/${ex}.cpp)
  target_link_libraries(${ex} PRIVATE mor)
  add_test(NAME ${ex} COMMAND ${ex})
endforeach()
