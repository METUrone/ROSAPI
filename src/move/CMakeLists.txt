cmake_minimum_required(VERSION 3.0.2)
project(move)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs mavros_msgs message_generation)

# Find the path to the pigpio includes.
find_path(pigpio_INCLUDE_DIR 
	NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
	HINTS /usr/local/include)
	
# Find the pigpio libraries.
find_library(pigpio_LIBRARY 
	NAMES libpigpio.so
	HINTS /usr/local/lib)
find_library(pigpiod_if_LIBRARY 
	NAMES libpigpiod_if.so
	HINTS /usr/local/lib)
find_library(pigpiod_if2_LIBRARY 
	NAMES libpigpiod_if2.so
	HINTS /usr/local/lib)

set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pigpio 
    DEFAULT_MSG 
    pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)

add_service_files(
    FILES
    PositionCommand.srv
    ArmDisarmCommand.srv
    TkoffLandCommand.srv
    State.srv
    Battery.srv
    Position.srv
    Camera.srv
)

# add_message_files(
#   FILES
#   /*message files*/
# )

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosapi_drone rosapi_plane
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${pigpio_INCLUDE_DIR})


add_executable(handler src/handler.cpp)
add_executable(commander src/commander.cpp)
add_executable(commander_plane src/commander_plane.cpp)
add_executable(handler_plane src/handler_plane.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})
target_link_libraries(handler ${catkin_LIBRARIES} pigpio)
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
