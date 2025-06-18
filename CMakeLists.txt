cmake_minimum_required(VERSION 3.10)
project(yuxi_solution)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Main executable
add_executable(planner src/main.cpp)
target_link_libraries(planner ${OpenCV_LIBS})

# --- Google Test Setup (Fixed) ---
# Option 1: Preferred - Use FetchContent with explicit targets
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1  # Use a stable release tag
)
FetchContent_MakeAvailable(googletest)

# Add test executable
add_executable(utils_test tests/utils.cpp)

# Link with gtest and gtest_main
target_link_libraries(utils_test
  PRIVATE
  gtest
  gtest_main
)

enable_testing()
add_test(NAME utils_test_suite COMMAND utils_test)

# Option 2: Alternative if Option 1 fails - use find_package
# find_package(GTest REQUIRED)
# target_link_libraries(utils_test GTest::GTest GTest::Main)