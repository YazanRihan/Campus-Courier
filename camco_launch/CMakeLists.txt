cmake_minimum_required(VERSION 3.8)
project(camco_launch)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camco_description REQUIRED)
find_package(kobuki_node REQUIRED)
find_package(rplidar_ros REQUIRED)
find_package(cmd_vel_mux REQUIRED)
find_package(kobuki_safety_controller REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
