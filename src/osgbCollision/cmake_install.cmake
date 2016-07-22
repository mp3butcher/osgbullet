# Install script for directory: /home/pascal/SRC/osgbullet/src/osgbCollision

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "DEBUG")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet")
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/x86_64-linux-gnu/libosgbCollision.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/x86_64-linux-gnu/libosgbCollision.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    ENDIF()
  ENDFOREACH()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/x86_64-linux-gnu" TYPE SHARED_LIBRARY FILES
    "/home/pascal/SRC/osgbullet/lib/libosgbCollision.so.3.00.00"
    "/home/pascal/SRC/osgbullet/lib/libosgbCollision.so"
    )
  FOREACH(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/x86_64-linux-gnu/libosgbCollision.so.3.00.00"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/x86_64-linux-gnu/libosgbCollision.so"
      )
    IF(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      FILE(RPATH_REMOVE
           FILE "${file}")
      IF(CMAKE_INSTALL_DO_STRIP)
        EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "${file}")
      ENDIF(CMAKE_INSTALL_DO_STRIP)
    ENDIF()
  ENDFOREACH()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osgbCollision" TYPE FILE FILES
    "/home/pascal/SRC/osgbullet/include/osgbCollision/BoundingCone.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/BoundingCylinder.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/Chart.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/CollectVerticesVisitor.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/CollisionShapes.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/ComputeCylinderVisitor.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/ComputeShapeVisitor.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/ComputeTriMeshVisitor.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/GLDebugDrawer.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/RefBulletObject.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/Utils.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/Version.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/VertexAggOp.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/Export.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/AbsoluteModelTransform.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/GeometryOperation.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/ReducerOp.h"
    "/home/pascal/SRC/osgbullet/include/osgbCollision/GeometryModifier.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "libosgbbullet-dev")

