# Install script for directory: F:/SRC/osgbullet/src/osgbDynamics

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/osgBullet")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/Debug/osgbDynamicsd.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/Release/osgbDynamics.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/MinSizeRel/osgbDynamics.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/RelWithDebInfo/osgbDynamics.lib")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/Debug/osgbDynamicsd.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/Release/osgbDynamics.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/MinSizeRel/osgbDynamics.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/RelWithDebInfo/osgbDynamics.dll")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgbDynamics" TYPE FILE FILES
    "F:/SRC/osgbullet/include/osgbDynamics/Export.h"
    "F:/SRC/osgbullet/include/osgbDynamics/Constraints.h"
    "F:/SRC/osgbullet/include/osgbDynamics/CreationRecord.h"
    "F:/SRC/osgbullet/include/osgbDynamics/GroundPlane.h"
    "F:/SRC/osgbullet/include/osgbDynamics/MotionState.h"
    "F:/SRC/osgbullet/include/osgbDynamics/PhysicsThread.h"
    "F:/SRC/osgbullet/include/osgbDynamics/RigidBody.h"
    "F:/SRC/osgbullet/include/osgbDynamics/Joint.h"
    "F:/SRC/osgbullet/include/osgbDynamics/SoftBody.h"
    "F:/SRC/osgbullet/include/osgbDynamics/RigidBodyAnimation.h"
    "F:/SRC/osgbullet/include/osgbDynamics/TripleBuffer.h"
    "F:/SRC/osgbullet/include/osgbDynamics/World.h"
    )
endif()

