cmake_minimum_required(VERSION 2.8.3)
project(vatroslav)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
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
add_message_files(FILES CanMsg.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )


add_executable(mainControl  Devices/MotorEPOS.cpp Devices/Sensor.cpp Devices/Flipper.cpp Devices/Kinematics.cpp Devices/WirelessVIV.cpp Communication/CanAdv.cpp Communication/DummyImpl.cpp Communication/CommPrint.cpp Communication/CommPar.cpp Communication/CommImpl.cpp Communication/Communication.cpp Communication/SerialBoost.cpp Communication/TimeoutSerial.cpp  Devices/LinAct.cpp Devices/flipper_test.cpp)

add_executable(canCommunication Communication/canCommunicationNode.cpp  Devices/MotorEPOS.cpp Devices/Sensor.cpp Devices/Flipper.cpp Devices/Kinematics.cpp Devices/WirelessVIV.cpp Communication/CanAdv.cpp Communication/DummyImpl.cpp Communication/CommPrint.cpp Communication/CommPar.cpp Communication/CommImpl.cpp Communication/Communication.cpp Communication/SerialBoost.cpp Communication/TimeoutSerial.cpp  Devices/LinAct.cpp)

TARGET_LINK_LIBRARIES( canCommunication ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libloki_LIBRARIES})
TARGET_LINK_LIBRARIES( mainControl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libloki_LIBRARIES})

# This prevents files being compiled before the necessary messages
# get generated!
add_dependencies(canCommunication vatroslav_gencpp )
add_dependencies(mainControl vatroslav_gencpp)









