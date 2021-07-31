cmake_minimum_required(VERSION 3.0.2)
project(moveit_panda_drawing)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
 CATKIN_DEPENDS 
 moveit_core
 moveit_ros_planning
 moveit_ros_planning_interface
)

add_executable(draw src/draw.cpp src/letter_poses.h src/letter_poses.cpp)
target_link_libraries(draw ${catkin_LIBRARIES})
