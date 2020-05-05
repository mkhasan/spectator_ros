cmake_minimum_required(VERSION 2.8.3)
project(spectator)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial_interface
  client_interface

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/spectator.cpp
  src/joint.cpp
)

add_message_files(
   FILES
   info.msg

 )

add_service_files(
  FILES
  get_float_value.srv
  get_uint_value.srv
  set_target.srv
  empty.srv
  get_str.srv
  get_str_list.srv
  set_val.srv
  
)

generate_messages(
   DEPENDENCIES
   std_msgs
   spectator
   
 )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES spectator serial_interface
   CATKIN_DEPENDS roscpp rospy std_msgs serial_interface message_generation message_runtime client_interface
)
 
add_dependencies(${PROJECT_NAME} serial_interface)

target_link_libraries(spectator ${serial_interface_LIBRARIES} ${catkin_LIBRARIES})

add_executable(spectator_node src/spectator_node.cpp src/cmd_handler.cpp src/snap_around.cpp)

add_dependencies(spectator_node serial_interface ${spectator_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(spectator_node spectator ${catkin_LIBRARIES} mosquittopp)

add_executable(spectator_key src/spectator_key.cpp )

add_dependencies(spectator_key serial_interface) 

target_link_libraries(spectator_key ${serial_interface_LIBRARIES} ${catkin_LIBRARIES} mosquittopp)

