cmake_minimum_required(VERSION 2.8.3)
project(arm_vicon_calibr)

find_package(catkin REQUIRED COMPONENTS
  arm_msgs
  roscpp
  geometry_msgs
  std_msgs
  actionlib
  actionlib_msgs
  usc_utilities
  sl_controller_interface
  tf
  tf_conversions
  visualization_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
	arm_msgs
	roscpp
	geometry_msgs
	std_msgs
	actionlib
	actionlib_msgs
	usc_utilities
	sl_controller_interface
	tf
	tf_conversions
	visualization_msgs
  DEPENDS 
	Eigen
  LIBRARIES 
 	${PROJECT_NAME}_lib
)


set(VRPN_ROOT vrpn)
set(ARTLIB_ROOT art_lib)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${VRPN_ROOT}
  ${ARTLIB_ROOT}
)

add_library(vicon_obj_lib
  src/vicon_obj.cpp
  ${ARTLIB_ROOT}/art_timer.cpp
  ${ARTLIB_ROOT}/art_signals.cpp
  ${ARTLIB_ROOT}/art_time.cpp
  ${VRPN_ROOT}/vrpn_Connection.C
  ${VRPN_ROOT}/vrpn_Tracker.C
  ${VRPN_ROOT}/vrpn_Shared.C
  ${VRPN_ROOT}/vrpn_FileConnection.C
  ${VRPN_ROOT}/vrpn_BaseClass.C
  ${VRPN_ROOT}/vrpn_Serial.C
  ${VRPN_ROOT}/vrpn_EndpointContainer.C
  ${VRPN_ROOT}/vrpn_Thread.C
)

target_link_libraries(vicon_obj_lib
  rt
  pthread
)

add_library(${PROJECT_NAME}_lib STATIC
	src/arm_vicon_calibr.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
	${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}
	src/arm_vicon_calibr.cpp
)

target_link_libraries(${EXEC_NAME}
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}\
  vicon_obj_lib
)

add_dependencies(${EXEC_NAME}
        arm_msgs_gencpp
)
