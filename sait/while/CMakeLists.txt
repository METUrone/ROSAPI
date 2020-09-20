cmake_minimum_required(VERSION 3.0.2)
project(while)

find_package(catkin REQUIRED COMPONENTS roscpp drone_library geometry_msgs mavros_msgs msgs )

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS) ## if you dont add this, executables are not found
add_executable(move_node src/node.cpp)
target_link_libraries(move_node ${catkin_LIBRARIES})

