cmake_minimum_required(VERSION 2.8.3)
project(bag_recorder)

add_definitions(-std=c++11)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

# find packages
find_package(catkin REQUIRED COMPONENTS rosbag roscpp topic_tools xmlrpcpp message_generation)
find_package(Boost REQUIRED COMPONENTS date_time regex program_options filesystem)
find_package(BZip2 REQUIRED)

# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

# dependencies
add_message_files(FILES
  Rosbag.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  LIBRARIES rosbag
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag_storage rosconsole roscpp topic_tools xmlrpcpp message_runtime)

add_library(recorder src/bag_recorder.cpp)
target_link_libraries(recorder ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# make executable
add_executable(${PROJECT_NAME}_node src/rosbag_recorder_node.cpp  src/bag_launcher.cpp src/heartbeat.cpp)
target_link_libraries(bag_recorder_node
                      recorder
                      ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)



# Install stuff
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
