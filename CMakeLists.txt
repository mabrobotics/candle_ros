cmake_minimum_required(VERSION 3.5)
project(candle_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  message_generation 
  std_msgs 
  sensor_msgs
)

add_message_files(
  DIRECTORY msg  
  FILES
  "ImpedanceCommand.msg"
  "MotionCommand.msg"
  "Pid.msg"
  "VelocityPidCommand.msg"
  "PositionPidCommand.msg"
)

add_service_files(FILES
  "AddMd80s.srv"
  "GenericMd80Msg.srv"
  "SetModeMd80s.srv"
  "SetLimitsMd80.srv"
)

generate_messages(DEPENDENCIES 
  std_msgs 
  sensor_msgs
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_INSTALL_RPATH $ORIGIN)  # sets proper runtime paths for exec to link .so properly

add_executable(${PROJECT_NAME}_node src/md80_node.cpp)
target_link_libraries(${PROJECT_NAME}_node "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/lib/libcandle.so" ${catkin_LIBRARIES}) 
target_include_directories(${PROJECT_NAME}_node PUBLIC ./include)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
