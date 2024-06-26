cmake_minimum_required(VERSION 3.5)
project(robp_phidgets_motors)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Defaults to release mode
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(libphidget22 REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robp_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(robp_phidgets_motors SHARED src/motors.cpp src/motor.cpp)
ament_target_dependencies(robp_phidgets_motors
	libphidget22
  phidgets_api
  rclcpp
  rclcpp_components
  robp_interfaces
  std_srvs
)

rclcpp_components_register_nodes(robp_phidgets_motors
  "robp::phidgets::Motors")

install(TARGETS robp_phidgets_motors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()