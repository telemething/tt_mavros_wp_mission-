cmake_minimum_required(VERSION 2.8.3)
project(tt_mavros_wp_mission CXX)


add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs actionlib std_msgs message_generation roscpp sensor_msgs geometry_msgs mavros)

### Build the message files
#find_package(catkin REQUIRED genmsg actionlib_msgs)

## Generate actions in the 'action' folder
add_action_files( DIRECTORY action FILES FlyMission.action )

add_message_files(FILES
  MissionStatus.msg Waypoint.msg )

add_service_files(FILES
  StartMission.srv WaypointPush.srv )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs std_msgs )

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
  message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

set(target global_pos_mission)
add_executable(${target} src/global_pos_mission.cpp src/UtilitiesGeo.cpp)
target_link_libraries(${target} ${catkin_LIBRARIES})

install(
  TARGETS ${target}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


set(target local_pos_mission)
add_executable(${target} src/local_pos_mission.cpp)
target_link_libraries(${target} ${catkin_LIBRARIES})

install(
  TARGETS ${target}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




