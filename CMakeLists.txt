cmake_minimum_required(VERSION 3.8)
project(lidar_object_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# Ground removal
add_executable(ground_removal_node
  src/ground_removal/ground_removal_node.cpp
  src/ground_removal/ground_removal.cpp
  src/ground_removal/cell.cpp
  src/ground_removal/gauss_blur.cpp
)

target_include_directories(ground_removal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ground_removal_node PUBLIC cxx_std_17)

ament_target_dependencies(ground_removal_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(ground_removal_node
  ${PCL_LIBRARIES}
)

# Clustering
add_executable(cluster_object_node
  src/clustering/component_clustering_node.cpp
  src/clustering/component_clustering.cpp
)

target_include_directories(cluster_object_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(cluster_object_node PUBLIC cxx_std_17)

ament_target_dependencies(cluster_object_node
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
)

target_link_libraries(cluster_object_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Visualization
add_executable(box_vis_node
  src/visualization/box_vis_node.cpp
  src/visualization/box_vis.cpp
)

target_include_directories(box_vis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(box_vis_node PUBLIC cxx_std_17)

ament_target_dependencies(box_vis_node
  rclcpp
  visualization_msgs
  Eigen3
)

target_link_libraries(box_vis_node
  ${PCL_LIBRARIES}
)

# installation
install(TARGETS
  ground_removal_node
  cluster_object_node
  box_vis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
