
add_executable(${PROJECT_NAME}_node
  mocap_node.cpp
  mocap_config.cpp
  mocap_datapackets.cpp
  socket_multicast.cpp
  socket_unicast.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES
                      OUTPUT_NAME optitrack_node PREFIX "")

add_executable(dummyserver_node
  dummyserver_node.cpp
)
target_link_libraries(dummyserver_node ${catkin_LIBRARIES})

install(TARGETS 
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
