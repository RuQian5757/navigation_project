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

target_include_directories(dynamic_world_cloud PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PCL_COMMON_INCLUDE_DIRS}
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

set_target_properties(dynamic_world_cloud PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME DynamicWorldCloud
)
