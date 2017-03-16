# Install script for directory: F:/SRC/osgbullet/src/osgbCollision

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
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/Debug/osgbCollisiond.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/Release/osgbCollision.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/MinSizeRel/osgbCollision.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "F:/SRC/osgbullet/lib/RelWithDebInfo/osgbCollision.lib")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/Debug/osgbCollisiond.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/Release/osgbCollision.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/MinSizeRel/osgbCollision.dll")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "F:/SRC/osgbullet/bin/RelWithDebInfo/osgbCollision.dll")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgbCollision" TYPE FILE FILES
    "F:/SRC/osgbullet/include/osgbCollision/BoundingCone.h"
    "F:/SRC/osgbullet/include/osgbCollision/BoundingCylinder.h"
    "F:/SRC/osgbullet/include/osgbCollision/Chart.h"
    "F:/SRC/osgbullet/include/osgbCollision/CollectVerticesVisitor.h"
    "F:/SRC/osgbullet/include/osgbCollision/CollisionShapes.h"
    "F:/SRC/osgbullet/include/osgbCollision/ComputeCylinderVisitor.h"
    "F:/SRC/osgbullet/include/osgbCollision/ComputeShapeVisitor.h"
    "F:/SRC/osgbullet/include/osgbCollision/ComputeTriMeshVisitor.h"
    "F:/SRC/osgbullet/include/osgbCollision/GLDebugDrawer.h"
    "F:/SRC/osgbullet/include/osgbCollision/RefBulletObject.h"
    "F:/SRC/osgbullet/include/osgbCollision/Utils.h"
    "F:/SRC/osgbullet/include/osgbCollision/Version.h"
    "F:/SRC/osgbullet/include/osgbCollision/VertexAggOp.h"
    "F:/SRC/osgbullet/include/osgbCollision/Export.h"
    "F:/SRC/osgbullet/include/osgbCollision/AbsoluteModelTransform.h"
    "F:/SRC/osgbullet/include/osgbCollision/GeometryOperation.h"
    "F:/SRC/osgbullet/include/osgbCollision/ReducerOp.h"
    "F:/SRC/osgbullet/include/osgbCollision/GeometryModifier.h"
    )
endif()

