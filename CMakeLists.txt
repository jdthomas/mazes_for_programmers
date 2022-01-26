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

find_package(cxxopts REQUIRED)
find_package(fmt REQUIRED)
find_package(libpng REQUIRED)
find_package(mdspan REQUIRED)
find_package(range-v3 REQUIRED)
find_package(TBB REQUIRED)
find_package(ParallelSTL CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_path(PNGPP_INCLUDE_DIRS "png++/color.hpp")




add_library(mazes src/maze.cpp src/maze_generators.cpp src/maze.h)
target_link_libraries(mazes PUBLIC fmt::fmt std::mdspan TBB::tbb ${CMAKE_THREAD_LIBS_INIT})
target_compile_features(mazes PUBLIC cxx_std_17)

add_executable(mazes_stats src/stats.cpp)
target_link_libraries(mazes_stats mazes fmt::fmt std::mdspan TBB::tbb ${CMAKE_THREAD_LIBS_INIT})
target_compile_features(mazes_stats PUBLIC cxx_std_17)

if (APPLE)
find_package(SFML COMPONENTS system window graphics)
if (SFML_FOUND)
  find_package(ImGui-SFML CONFIG REQUIRED)
  find_package(imgui CONFIG REQUIRED)
  add_executable(mazes_gui MACOSX_BUNDLE res/MazeIcon.icns src/gui.cpp)
  target_link_libraries(mazes_gui mazes fmt::fmt png_static std::mdspan sfml-graphics TBB::tbb cxxopts::cxxopts imgui::imgui ImGui-SFML::ImGui-SFML ${CMAKE_THREAD_LIBS_INIT} "-framework GameController")
  target_include_directories(mazes_gui PRIVATE ${PNGPP_INCLUDE_DIRS})
  target_compile_features(mazes_gui PUBLIC cxx_std_17)
  set_target_properties(mazes_gui PROPERTIES
      BUNDLE True
      MACOSX_BUNDLE_GUI_IDENTIFIER
      com.coffeetocode.mazes_gui
      MACOSX_BUNDLE_BUNDLE_NAME mazes_gui
      MACOSX_BUNDLE_BUNDLE_VERSION "0.1"
      MACOSX_BUNDLE_SHORT_VERSION_STRING "0.1"
      MACOSX_BUNDLE_ICON_FILE "MazeIcon.icns"
    )

  set_source_files_properties(res/MazeIcon.icns PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")
  install(TARGETS mazes_gui
      BUNDLE DESTINATION . COMPONENT Runtime
      RUNTIME DESTINATION bin COMPONENT Runtime
      )
  set(CPACK_GENERATOR "DragNDrop")
  include(CPack)
endif ()
endif ()
#
enable_testing()
add_subdirectory(tests)
add_subdirectory(benchmarks)
