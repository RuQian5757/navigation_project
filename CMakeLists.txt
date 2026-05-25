cmake_minimum_required(VERSION 3.16)
project(navigation_project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(navigation_node
    src/main.cpp
    src/octree_manager.cpp
)

target_link_libraries(navigation_node PRIVATE Threads::Threads)

# Future dependencies:
# find_package(ONNXRuntime REQUIRED)
# find_package(PCL REQUIRED)
# target_link_libraries(navigation_node PRIVATE ${ONNXRuntime_LIBRARIES} ${PCL_LIBRARIES})
