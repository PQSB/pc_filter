cmake_minimum_required(VERSION 3.8)
project(pc_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
# --------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# ----------------------------------------------------------------

# For argument management
include(FetchContent)
fetchcontent_declare(cxxopts
  GIT_REPOSITORY  https://github.com/jarro2783/cxxopts
  GIT_TAG         v3.2.1
)
FetchContent_MakeAvailable(cxxopts)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  rosbag2_cpp
  rosbag2_storage
  pcl_conversions
  cv_bridge
)
# --------------------------------------------------------------------------

add_definitions(${PCL_DEFINITIONS})
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# -----------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/pc_filter/PCFilterNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} Eigen3::Eigen cxxopts)

# Add my library
add_library(pc_filter_lib SHARED
  src/pc_filter/pc_filter_lib.cpp 
)
ament_target_dependencies(pc_filter_lib ${dependencies})
target_link_libraries(pc_filter_lib ${PCL_LIBRARIES} Eigen3::Eigen)

# add_executable(rosbag_pc_filter src/filter_cloud_by_detections.cpp)
# ament_target_dependencies(rosbag_pc_filter ${dependencies})
# # --- NUEVO: Enlazar PCL y Eigen al nodo ejecutable ---
# target_link_libraries(rosbag_pc_filter ${PROJECT_NAME} pc_filter_lib ${PCL_LIBRARIES} Eigen3::Eigen)

add_executable(offline_pc_filter src/offline_pc_filter.cpp)
ament_target_dependencies(offline_pc_filter ${dependencies})
target_link_libraries(offline_pc_filter ${PROJECT_NAME} pc_filter_lib ${PCL_LIBRARIES} Eigen3::Eigen)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  #rosbag_pc_filter
  offline_pc_filter
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
