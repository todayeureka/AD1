###  cmake 필요 최소 버전
cmake_minimum_required(VERSION 2.8.3)
###  ${PROJECT_NAME}이라는 변수에 저장
project(ad1_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    geometry_msgs
)

add_message_files(
  FILES
  PlantRobotInfo.msg
  CmdMsg.msg
  MissionInfo.msg
  RobotMission.msg
)


add_service_files(
  FILES
#  SetBtn.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs
)


#add_executable(joy_control src/joy_control.cpp)
#add_dependencies(joy_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(joy_control ${catkin_LIBRARIES})
