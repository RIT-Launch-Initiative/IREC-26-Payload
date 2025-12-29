cmake_minimum_required(VERSION 3.16)

project(radio)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(radio main.cpp)

add_subdirectory(vendor/fuzztest)

enable_testing()

include(GoogleTest)

add_executable(
  fuzz_test
  fuzz_test.cpp
)

# If you have other dependencies than FuzzTest, link them:
# target_link_libraries(first_fuzz_test PRIVATE other_deps)
fuzztest_setup_fuzzing_flags()

link_fuzztest(fuzz_test)
gtest_discover_tests(fuzz_test)