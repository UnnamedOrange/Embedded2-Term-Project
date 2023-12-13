# Copyright (c) UnnamedOrange. Licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

cmake_minimum_required(VERSION 3.26 FATAL_ERROR)

# Set this will supress some warnings.
set(BUILDING_SDK "yes" CACHE INTERNAL "")

# Basic config.
include("./cmake/common.cmake")
project(blur
  VERSION 0.1.0
  LANGUAGES C CXX ASM
)

# Config self use headers.
include("./cmake/macros.internal.cmake")
header_directories("${SDK_ROOT}/lib/")
header_directories("src/")
# Build library first.
add_subdirectory("lib/")

# Compile project.
file(
  GLOB_RECURSE
  SOURCES
  CONFIGURE_DEPENDS
  "src/*"
)
add_source_files("${SOURCES}")
include(./cmake/executable.cmake)
target_compile_features(${CMAKE_PROJECT_NAME} PRIVATE cxx_std_20)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic) # -Werror
