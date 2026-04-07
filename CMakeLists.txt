cmake_minimum_required(VERSION 3.8)
project(pc_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  pcl_ros
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/pc_filter/PCFilterNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Add my library
add_library(pc_filter_lib SHARED
  src/pc_filter/pc_filter_lib.cpp 
)
ament_target_dependencies(pc_filter_lib ${dependencies})

add_executable(pc_filter_node src/pc_filter_main.cpp)
ament_target_dependencies(pc_filter_node ${dependencies})
target_link_libraries(pc_filter_node ${PROJECT_NAME} pc_filter_lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  pc_filter_node
  pc_filter_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
