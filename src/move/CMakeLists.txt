cmake_minimum_required(VERSION 3.0.2)
project(move)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs message_generation)

add_service_files(
    FILES
    PositionCommand.srv
    Battery.srv
    Position.srv
    CameraInfo.srv
)

generate_messages()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(commander src/commander.cpp)
add_executable(handler src/handler.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})
target_link_libraries(handler ${catkin_LIBRARIES})
