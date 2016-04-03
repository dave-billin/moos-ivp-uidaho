#==============================================================================
# iBSDuC CMake configuration file
#
# Created CREATION_DATE by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iBSDuC)

set( VERSION_MAJOR  1 )
set( VERSION_MINOR  0 )
set( VERSION_PATCH  0 )

GetPatchVersion( ${PROJECT_SOURCE_DIR} VERSION_PATCH )

set( VERSION_TUPLE 
    "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}"
)

set(PROJECT_AUTHOR "Dave Billin")
set(PROJECT_EMAIL "david.billin@vandals.uidaho.edu")



# Set up variables for project directories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( CONFIG_DIR ${PROJECT_SOURCE_DIR}/config )
set( DOCS_DIR ${PROJECT_SOURCE_DIR}/docs )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${DOCS_DIR}/docs} )
endif()



# Process templates to generate common files
configure_file(${CONFIG_DIR}/AppVersion.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/AppVersion.h )
configure_file( "${CONFIG_DIR}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


#===============================
# Source Files
#===============================

include_directories( ${INCLUDE_DIR}
                     ${MOOS_INCLUDE_DIRS} )

set( PROJECT_SOURCE_FILES
     ${SRC_DIR}/ParapetSerialDeviceNode.cpp
     ${SRC_DIR}/BSDuCModule.cpp
     ${SRC_DIR}/BSDuCStartupFSM.cpp
     ${SRC_DIR}/ActuatorController.cpp
     ${SRC_DIR}/iBSDuC.cpp
     ${SRC_DIR}/main.cpp
)


#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} ${PROJECT_SOURCE_FILES} )

# Indicate how to link
target_link_libraries( ${PROJECT_NAME}
                       YellowSubUtils
                       parapet
					   ${MOOS_LIBRARIES}
                       ${MOOS_DEPEND_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
					 )

# Install Targets
install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

