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

# ignition package
find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)

# include
include_directories(include)

# generate rmua19_ignition_simulator lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    geometry_msgs
    rmoss_interfaces
)
target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

#robot_base_node
add_executable(robot_base nodes/robot_base_node.cpp)
target_link_libraries(robot_base ${PROJECT_NAME})


#install
install( DIRECTORY
    ign launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS robot_base
  DESTINATION lib/${PROJECT_NAME} 
)

# Install executable scripts
install(PROGRAMS 
        scripts/simple_competition_1v1_referee.py 
    DESTINATION lib/${PROJECT_NAME})

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
