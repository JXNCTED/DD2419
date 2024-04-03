cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robp_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)



add_executable(vi_odom src/vi_odom.cpp)
add_executable(wheel_odom src/wheel_odom.cpp)
add_executable(filter_odom src/filter_odom.cpp)
add_executable(lidar_compensator src/lidar_compensator.cpp)
add_executable(lidar_landmarker src/lidar_landmarker.cpp)

add_library(ekf src/ekf.cpp)
target_include_directories(ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(vi_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(wheel_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_include_directories(lidar_compensator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(lidar_landmarker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(lidar_compensator ${PCL_LIBRARIES})
target_link_libraries(lidar_landmarker ${PCL_LIBRARIES})


target_compile_features(vi_odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(wheel_odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(filter_odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(lidar_compensator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(lidar_landmarker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  vi_odom
  "pcl_conversions"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "robp_interfaces"
  "tf2_ros"
  "tf2_geometry_msgs"
  "cv_bridge"
  "OpenCV"
)

ament_target_dependencies(
  wheel_odom
  "pcl_conversions"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "robp_interfaces"
  "tf2_ros"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  filter_odom
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

ament_target_dependencies(
  lidar_compensator
  "pcl_conversions"
  "PCL"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
)

ament_target_dependencies(
  lidar_landmarker
  "pcl_conversions"
  "PCL"
  "sensor_msgs"
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "robp_interfaces"
  "tf2_ros"
)


install(TARGETS 
  vi_odom 
  wheel_odom 
  filter_odom
  lidar_compensator
  lidar_landmarker
  DESTINATION lib/${PROJECT_NAME})

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
