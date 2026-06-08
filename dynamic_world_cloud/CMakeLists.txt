find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL_COMMON QUIET pcl_common)

if(NOT PCL_COMMON_FOUND)
  message(WARNING
    "pcl_common was not found. Install libpcl-dev to build DynamicWorldCloud.")
  add_custom_target(dynamic_world_cloud
    COMMAND ${CMAKE_COMMAND} -E echo
      "pcl_common not found. Install libpcl-dev, then rerun cmake."
    COMMAND ${CMAKE_COMMAND} -E false
  )
  return()
endif()

add_library(dynamic_world_cloud SHARED
  DynamicWorldCloud.cc
)

add_library(rf_octree_path_planner_plugin SHARED
  RFOctreePathPlanner.cc
)

target_include_directories(dynamic_world_cloud PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
)

target_include_directories(rf_octree_path_planner_plugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(dynamic_world_cloud PRIVATE
  ${PCL_COMMON_CFLAGS_OTHER}
)

target_link_directories(dynamic_world_cloud PRIVATE
  ${PCL_COMMON_LIBRARY_DIRS}
)

target_link_libraries(dynamic_world_cloud PRIVATE
  gz-sim8::gz-sim8
  gz-common5::graphics
  gz-msgs10::gz-msgs10
  gz-plugin2::register
  gz-transport13::gz-transport13
  ${PCL_COMMON_LIBRARIES}
)

target_link_libraries(rf_octree_path_planner_plugin PRIVATE
  navigation_octree
  gz-sim8::gz-sim8
  gz-msgs10::gz-msgs10
  gz-plugin2::register
  gz-transport13::gz-transport13
)

set_target_properties(dynamic_world_cloud PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME DynamicWorldCloud
)

set_target_properties(rf_octree_path_planner_plugin PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME RFOctreePathPlanner
)

add_custom_target(dynamic_world_cloud_plugins DEPENDS
  dynamic_world_cloud
  rf_octree_path_planner_plugin
)
