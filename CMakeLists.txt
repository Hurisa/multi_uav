cmake_minimum_required(VERSION 2.8.3)
project(multi_uav)

find_package(catkin REQUIRED
  
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2_msgs
)

catkin_package(
  CATKIN_DEPENDS catkin
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(initvel src/initvel.cpp)
add_executable(rotate src/rotate.cpp)
add_executable(override src/override.cpp)
add_executable(front_vel src/front_vel.cpp)
add_executable(control_yaw src/control_yaw.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(random_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(random_vel_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(initvel ${catkin_LIBRARIES})
target_link_libraries(rotate ${catkin_LIBRARIES})
target_link_libraries(override ${catkin_LIBRARIES})
target_link_libraries(front_vel ${catkin_LIBRARIES})
target_link_libraries(control_yaw ${catkin_LIBRARIES})
