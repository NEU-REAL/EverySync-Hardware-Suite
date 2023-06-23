cmake_minimum_required(VERSION 2.8.3)

project(every_sync)

# Set compile flags (-Ofast actually makes a big difference over -O3 here (maybe 20% faster)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast")

find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_generation
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_numbered_msgs
  imu_numbered_msgs
)

add_message_files(
  FILES
  ImuMicro.msg
  TimeNumbered.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS message_runtime
)

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
link_directories(
	../firmware/libraries/ADIS16445/
)

add_library(${PROJECT_NAME}_nodelet
  src/synchronizer_nodelet.cpp
  src/synchronizer.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(synchronizer_node src/synchronizer_node.cpp)
target_link_libraries(synchronizer_node ${catkin_LIBRARIES})

add_executable(imu_receiver
   src/imu_receiver.cpp
)
add_dependencies(imu_receiver ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(imu_receiver ${catkin_LIBRARIES})

############
## EXPORT ##
############
# Install nodelet library
install(TARGETS ${PROJECT_NAME}_nodelet
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install xml files
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
