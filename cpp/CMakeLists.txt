cmake_minimum_required(VERSION 3.20)
project(mrmp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)
# Python and the TypeScript engines round every multiply/add separately; letting
# the compiler fuse a*b+c into one FMA gives differently-rounded results that
# drift over iterative planners, breaking the cross-language bit-identical
# contract. Keep C++ on the same separate-rounding semantics.
add_compile_options(-ffp-contract=off)

# --- core + maps library ---
file(GLOB_RECURSE MRMP_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/maps/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/search/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sampling/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/decentralized/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kinodynamic/*.cpp)

add_library(mrmp ${MRMP_SOURCES})
target_include_directories(mrmp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# --- demos: one executable per demo_*.cpp ---
file(GLOB DEMO_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/demos/demo_*.cpp)
foreach(demo_src ${DEMO_SOURCES})
  get_filename_component(demo_name ${demo_src} NAME_WE)
  add_executable(${demo_name} ${demo_src})
  target_link_libraries(${demo_name} PRIVATE mrmp)
  set_target_properties(${demo_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/demos)
endforeach()

# --- tests (GoogleTest via FetchContent) ---
option(MRMP_BUILD_TESTS "Build tests" ON)
if(MRMP_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  # GoogleTest is fetched after our global strict flags, so its own sources would
  # otherwise be compiled under -Wextra; silence upstream warnings on its targets.
  foreach(gt gtest gtest_main gmock gmock_main)
    if(TARGET ${gt})
      target_compile_options(${gt} PRIVATE -w)
    endif()
  endforeach()

  enable_testing()
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
  add_executable(mrmp_tests ${TEST_SOURCES})
  target_link_libraries(mrmp_tests PRIVATE mrmp GTest::gtest_main)
  # Lets tests read the real configs/maps under the repo root.
  target_compile_definitions(mrmp_tests PRIVATE
    MRMP_REPO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
  target_include_directories(mrmp_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  include(GoogleTest)
  gtest_discover_tests(mrmp_tests)
endif()
