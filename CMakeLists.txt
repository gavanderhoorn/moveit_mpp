cmake_minimum_required(VERSION 2.8.3)

project(moveit_mpp)

add_compile_options(-std=c++11)

find_package(
  catkin
    REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    pluginlib
    roscpp
)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    pluginlib
    roscpp
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_planner_manager src/mpp_planner_manager.cpp)
target_link_libraries(${PROJECT_NAME}_planner_manager ${catkin_LIBRARIES})
