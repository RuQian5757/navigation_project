cmake_minimum_required(VERSION 3.16)
project(navigation_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(navigation_octree STATIC
    src/octree_manager.cpp
)

target_link_libraries(navigation_octree PRIVATE Threads::Threads)

target_include_directories(navigation_octree PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_executable(navigation_node
    src/main.cpp
)

target_link_libraries(navigation_node PRIVATE navigation_octree)

add_executable(test_octree
    tests/test_octree.cpp
)

target_link_libraries(test_octree PRIVATE navigation_octree)

add_custom_target(check COMMAND test_octree)

add_subdirectory(dynamic_world_cloud)
