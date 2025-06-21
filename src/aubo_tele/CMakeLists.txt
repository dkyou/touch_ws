cmake_minimum_required(VERSION 3.0.2)
project(aubo_tele)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED
  COMPONENTS
    trac_ik_lib

    roscpp
    rospy
    std_msgs
    message_generation

    omni_msgs

    geometry_msgs
     
    sensor_msgs
    omni_common
    omni_description
    
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  output.msg
  OmniButtonEvent.msg
  OmniFeedback.msg
  OmniState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs

  geometry_msgs 
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
    
  DEPENDS
    Boost
    orocos_kdl
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(aubo_tele src/aubo_tele.cpp)
target_link_libraries(aubo_tele
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS aubo_tele
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_executable(aubo_tele_pose src/aubo_tele_pose.cpp)
target_link_libraries(aubo_tele_pose
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS aubo_tele_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

