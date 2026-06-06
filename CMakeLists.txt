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

if(EXISTS "${PROJECT_SOURCE_DIR}/src/pointcloud_bridge.cpp")
    add_executable(pointcloud_bridge
        src/pointcloud_bridge.cpp
    )

    target_link_libraries(pointcloud_bridge PRIVATE
        navigation_octree
        gz-transport13::gz-transport13
        gz-msgs10::gz-msgs10
    )
endif()

if(EXISTS "${PROJECT_SOURCE_DIR}/src/test_dynamic_cloud.cpp")
    add_executable(test_dynamic_cloud
        src/test_dynamic_cloud.cpp
    )

    target_link_libraries(test_dynamic_cloud PRIVATE
        gz-transport13::gz-transport13
        gz-msgs10::gz-msgs10
    )
endif()

add_custom_target(check COMMAND test_octree)

add_subdirectory(dynamic_world_cloud)
