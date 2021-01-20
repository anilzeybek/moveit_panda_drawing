cmake_minimum_required(VERSION 3.0.2)
project(panda_hw)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
