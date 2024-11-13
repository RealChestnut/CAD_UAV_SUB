cmake_minimum_required(VERSION 2.8.3)
project(serial_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)


add_service_files(
    FILES
    serial_safety_msg.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs

)


catkin_package(
 
  INCLUDE_DIRS include	
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    serial
    std_msgs
    CATKIN_DEPENDS message_runtime
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




add_executable(serial_magnetic_sub_node src/serial_magnetic_sub_new_node.cpp)
#add_executable(serial_magnetic_sub_node src/serial_test.cpp)

add_dependencies(serial_magnetic_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(serial_magnetic_sub_node
  ${catkin_LIBRARIES}
)


