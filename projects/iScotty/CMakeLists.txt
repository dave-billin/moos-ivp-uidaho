#==============================================================================
# iScotty CMake configuration file
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iScotty)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
GetSvnWorkingCopyRevision( ${PROJECT_SOURCE_DIR} VERSION_PATCH )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( DOCS_DIR ${PROJECT_SOURCE_DIR}/docs )
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs} )
endif()


# Fill in fields of templated files
configure_file(${BUILD_DIR}/main.cpp.in
               ${SRC_DIR}/main.cpp)
configure_file(${BUILD_DIR}/AppVersion.h.in
               ${INCLUDE_DIR}/AppVersion.h )
configure_file( "${BUILD_DIR}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


#===============================
# Source Files
#===============================

include_directories( ${MOOS_INCLUDE_DIRECTORIES}
                     ${BUNNYSOCK_INCLUDE_DIRECTORIES}
                     ${INCLUDE_DIR} )

# Create a list of source files
set( SOURCE_FILES
     ${SRC_DIR}/ActuatorPID.cpp
     ${SRC_DIR}/iScotty.cpp
     ${SRC_DIR}/main.cpp
     ${SRC_DIR}/PitchController.cpp
     ${SRC_DIR}/ScottyModule.cpp
   )

#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )

# Indicate how to link
target_link_libraries( ${PROJECT_NAME}
                       BunnySock
					   ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
					 )

add_dependencies( ${PROJECT_NAME} BunnySock )


# Install Targets
install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)
