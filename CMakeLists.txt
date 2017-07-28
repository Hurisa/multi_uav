cmake_minimum_required(VERSION 2.8.3)
project(multi_uav)

find_package(catkin REQUIRED
  
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS catkin
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(initvel src/initvel.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(random_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(random_vel_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(initvel
${catkin_LIBRARIES}
)
