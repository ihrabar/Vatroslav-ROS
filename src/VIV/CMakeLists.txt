cmake_minimum_required(VERSION 2.8.3)
project(vatroslav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  std_msgs
  tf
  message_generation
)

FIND_PACKAGE( Boost 1.58 COMPONENTS REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#MESSAGE ("This is ", ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
find_package(
  Loki REQUIRED
  OpenCV REQUIRED COMPONENTS flann # for matrix.h
)


INCLUDE(FindLoki)
FIND_PACKAGE(Loki REQUIRED)
INCLUDE_DIRECTORIES(${LOKI_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
#add_message_files(FILES arrayData.msg gridData.msg Path.msg planData.msg stateInfo.msg vehInfo.msg)

# Generate services in the 'srv' folder
#add_service_files(FILES CommStatus.srv GetMyPlan.srv Removal.srv)

# Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf message_runtime
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
#add_executable(canCommunicationTest Communication/CanAdv.cpp Communication/DummyImpl.cpp Communication/CommPrint.cpp Communication/CommPar.cpp Communication/CommImpl.cpp Communication/Communication.cpp Communication/SerialBoost.cpp Communication/TimeoutSerial.cpp Communication/comm_test.cpp)
add_executable(flipperTest Devices/MotorEPOS.cpp Devices/LinAct.cpp Devices/Sensor.cpp Devices/Flipper.cpp Devices/Kinematics.cpp Devices/WirelessVIV.cpp Devices/flipper_test.cpp Communication/CanAdv.cpp Communication/DummyImpl.cpp Communication/CommPrint.cpp Communication/CommPar.cpp Communication/CommImpl.cpp Communication/Communication.cpp Communication/SerialBoost.cpp Communication/TimeoutSerial.cpp)
#add_executable(agvController src/agvController.cpp src/collisionDetection.cpp src/pathFollowing.cpp)

## Specify libraries to link a library or executable target against
#TARGET_LINK_LIBRARIES( canCommunicationTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libloki_LIBRARIES})
TARGET_LINK_LIBRARIES( flipperTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libloki_LIBRARIES})
#target_link_libraries(canCommunication ${catkin_LIBRARIES})

# This prevents files being compiled before the necessary messages
# get generated!
#add_dependencies(canCommunication canCommunication) ?????????????? kakoooooo
#add_dependencies(agvController agv_control_gencpp)
#add_dependencies(motionSimSR agv_control_gencpp)








