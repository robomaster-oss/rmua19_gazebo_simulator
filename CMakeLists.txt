cmake_minimum_required(VERSION 3.5)
project(rmua19_ignition_simulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ros package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(rmoss_ign_base REQUIRED)

# Install
install( DIRECTORY
    launch resource config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable scripts
install(PROGRAMS
        scripts/referee_system/simple_competition_1v1.py 
    DESTINATION lib/${PROJECT_NAME})

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
