cmake_minimum_required(VERSION 3.16)
project(navigation_octree_pcl_viewer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(visualize_octree_gazebo
  visualize_octree_gazebo.cpp
  "${PROJECT_ROOT}/src/octree_manager.cpp"
)

target_include_directories(visualize_octree_gazebo PRIVATE
  "${PROJECT_ROOT}/include"
  ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(visualize_octree_gazebo PRIVATE
  ${PCL_DEFINITIONS}
)

target_link_libraries(visualize_octree_gazebo PRIVATE
  ${PCL_LIBRARIES}
)

target_link_libraries(visualize_octree_gazebo PRIVATE
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
