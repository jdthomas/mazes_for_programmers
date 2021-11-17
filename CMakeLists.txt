cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE
  ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake
    CACHE STRING "Vcpkg toolchain file")

project(
  "mazes"
  VERSION 0.1.0
  LANGUAGES CXX)

# add_library(project_options INTERFACE)
# add_library(project_warnings INTERFACE)

# include(cmake/StaticAnalyzers.cmake)
# include(cmake/Sanitizers.cmake)
# enable_sanitizers(project_options)
# include(cmake/CompilerWarnings.cmake)
# TODO: Cleanup the warnings and enable!
# set(WARNINGS_AS_ERRORS OFF)
# set_project_warnings(project_warnings)

find_package(fmt REQUIRED)
find_package(mdspan REQUIRED)
find_package(range-v3 REQUIRED)
find_package(tbb REQUIRED)
find_package(parallelstl CONFIG REQUIRED)
find_package(SFML REQUIRED system window graphics)

add_executable(${PROJECT_NAME} src/maze.cpp)
target_link_libraries(${PROJECT_NAME} fmt::fmt-header-only std::mdspan sfml-graphics TBB::tbb)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
