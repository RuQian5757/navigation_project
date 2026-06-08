cmake_minimum_required(VERSION 3.16)
project(navigation_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(navigation_octree STATIC
    src/octree_manager.cpp
    src/leaf_feature_exporter.cpp
    src/random_forest_voxel_predictor.cpp
    src/astar_planner.cpp
)

target_link_libraries(navigation_octree PRIVATE Threads::Threads)

target_include_directories(navigation_octree PUBLIC ${PROJECT_SOURCE_DIR}/include)

set_target_properties(navigation_octree PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

add_executable(navigation_node
    src/main.cpp
)

target_link_libraries(navigation_node PRIVATE navigation_octree)

add_executable(test_octree
    tests/test_octree.cpp
)

target_link_libraries(test_octree PRIVATE navigation_octree)

add_custom_target(check COMMAND test_octree)

add_executable(leaf_feature_exporter_gazebo
    src/gazebo_leaf_feature_exporter.cpp
)

target_link_libraries(leaf_feature_exporter_gazebo PRIVATE
    navigation_octree
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    Threads::Threads
)

add_executable(rf_octree_path_planner_gazebo
    src/gazebo_rf_path_planner.cpp
)

target_link_libraries(rf_octree_path_planner_gazebo PRIVATE
    navigation_octree
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    Threads::Threads
)

add_executable(octree_performance_benchmark
    src/octree_performance_benchmark.cpp
)

target_link_libraries(octree_performance_benchmark PRIVATE
    navigation_octree
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    Threads::Threads
)

add_subdirectory(dynamic_world_cloud)
