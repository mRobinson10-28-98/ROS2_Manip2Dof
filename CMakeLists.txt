cmake_minimum_required(VERSION 3.8)
project(manip2dof_practice)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories 
include_directories(/usr/include/eigen3)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manip2dof_interfaces REQUIRED)

# libraries
add_library(Manip2Dof src/Manip2Dof.cpp)
target_include_directories(Manip2Dof PUBLIC ${PROJECT_SOURCE_DIR}/include/2dof_practice)

# executables/nodes
add_executable(manip2dof_tracecircle_talker src/Manip2Dof_SetGoalTraceCircle.cpp)
add_executable(manip2dof_solvegoal src/Manip2Dof_SolveGoal.cpp)
target_link_libraries(manip2dof_solvegoal Manip2Dof)
#target_include_directories(2dof_listener PUBLIC ${PROJECT_SOURCE_DIR}/include/2dof_practice)

ament_target_Dependencies(manip2dof_tracecircle_talker rclcpp std_msgs geometry_msgs) 
ament_target_Dependencies(manip2dof_solvegoal rclcpp std_msgs geometry_msgs manip2dof_interfaces) 

install(TARGETS
  manip2dof_tracecircle_talker
  manip2dof_solvegoal
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
