# Install script for directory: /home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/src/withisnew

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/build/withisnew/catkin_generated/installspace/withisnew.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/withisnew/cmake" TYPE FILE FILES
    "/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/build/withisnew/catkin_generated/installspace/withisnewConfig.cmake"
    "/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/build/withisnew/catkin_generated/installspace/withisnewConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/withisnew" TYPE FILE FILES "/home/marco/Dropbox/Manuali_e_Template/ROS/MatlabROS/ProveIsNew/catkin_ws/src/withisnew/package.xml")
endif()

