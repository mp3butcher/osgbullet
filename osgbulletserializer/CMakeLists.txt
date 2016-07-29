SET(LIB_NAME osgdb_serializers_osgbdynamics)
#SET(HEADER_PATH "${osgAudio_SOURCE_DIR}/include/${LIB_NAME}")
SET(osgPhysics_USER_DEFINED_DYNAMIC_OR_STATIC MODULE)

#SET(LIB_PUBLIC_HEADERS ...)

ADD_LIBRARY(
    ${LIB_NAME}
    ${osgPhysics_USER_DEFINED_DYNAMIC_OR_STATIC}
    #${LIB_PUBLIC_HEADERS}
    IO_World.cpp
 IO_SoftBodyElement.cpp
IO_RigidElement.cpp

IO_Joint.cpp
IO_MatrixTransform.cpp

)


add_definitions(
  -DOSGPLUGINosgPhysics_EXPORTS
  # -DWIN32
  # -D_DEBUG
  # -D_USRDLL
  # -D_WINDOWS
)
#INCLUDE_WITH_VARIABLES( ${LIB_NAME} ${SUBSYSTEM_INCLUDES} )
INCLUDE_DIRECTORIES( ${OSG_INCLUDE_DIRS}
  ${OSG_INCLUDE_DIR})

#LINK_WITH_VARIABLES( ${LIB_NAME} ${SUBSYSTEM_LINKS} )
INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIR}/../Extras/Serialize/BulletFileLoader)
TARGET_LINK_LIBRARIES( ${LIB_NAME}  osgbDynamics ${SUBSYSTEM_TARGET_LINKS}  ${OSG_LIBRARIES} ${BULLET_EXTRA_FILELOADER_LIBRARY} ${BULLET_EXTRA_IMPORTER_LIBRARY})

IF(MSVC_IDE)
    # Ugly workaround to remove the "/debug" or "/release" in each output
#    SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES PREFIX "../")
#    SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES IMPORT_PREFIX "../")
ENDIF()
 set_target_properties( ${LIB_NAME} PROPERTIES PREFIX "" )
    set_target_properties( ${LIB_NAME} PROPERTIES PROJECT_LABEL "Plugin ${LIB_NAME}" )
INCLUDE(ModuleInstall OPTIONAL)
