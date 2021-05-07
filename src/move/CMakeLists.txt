cmake_minimum_required(VERSION 3.0.2)
project(move)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs mavros_msgs message_generation)

add_service_files(
    FILES
    PositionCommand.srv
    Battery.srv
    Position.srv
    Camera.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosapi_drone rosapi_plane
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(handler src/handler.cpp)
add_executable(commander src/commander.cpp)
add_executable(commander_plane src/commander_plane.cpp)
add_executable(handler_plane src/handler_plane.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})
target_link_libraries(handler ${catkin_LIBRARIES})
target_link_libraries(commander_plane ${catkin_LIBRARIES})
target_link_libraries(handler_plane ${catkin_LIBRARIES})
add_library(rosapi_drone src/handler.cpp)
add_library(rosapi_plane src/handler_plane.cpp)
target_link_libraries(rosapi_drone ${catkin_LIBRARIES})
target_link_libraries(rosapi_plane ${catkin_LIBRARIES})

install(
  TARGETS rosapi_drone rosapi_plane
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
