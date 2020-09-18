cmake_minimum_required(VERSION 3.0.2)
project(try)

find_package(catkin REQUIRED COMPONENTS roscpp drone_library geometry_msgs mavros_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package() ## if you dont add this, executables are not found
add_executable(node src/node.cpp)
target_link_libraries(node ${catkin_LIBRARIES})

