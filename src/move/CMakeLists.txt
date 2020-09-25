cmake_minimum_required(VERSION 3.0.2)
project(move)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs mavros_msgs message_generation)

add_service_files(
    FILES
    Pos.srv
    Rot.srv
    vel.srv
)

generate_messages()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(move_node src/move.cpp)
add_executable(handle_node src/handle.cpp)
target_link_libraries(move_node ${catkin_LIBRARIES})
target_link_libraries(handle_node ${catkin_LIBRARIES})
