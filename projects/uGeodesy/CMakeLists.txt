#==============================================================================
# uGeodesy CMake configuration file
#
# Created CREATION_DATE by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(uGeodesy)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
GetPatchVersion( ${PROJECT_SOURCE_DIR} VERSION_PATCH )
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
configure_file(${BUILD_DIR}/AppVersion.h.in
               ${INCLUDE_DIR}/AppVersion.h )
configure_file( "${BUILD_DIR}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


find_package( MOOS 10 REQUIRED )
find_package( MOOSGeodesy REQUIRED )


#===============================
# Source Files
#===============================

include_directories( ${MOOS_INCLUDE_DIRECTORIES} 
                     ${MOOSGeodesy_INCLUDE_DIRS}
                     ${INCLUDE_DIR} )


set( SOURCE_FILES
     ${SRC_DIR}/main.cpp
)



#======================================
# Set up libraries to be linked in
#======================================
set( target_LIBS ${MOOS_LIBRARIES}
                 ${MOOS_DEPEND_LIBRARIES}
                 ${MOOSGeodesy_LIBRARIES}
                 ${MOOSGeodesy_DEPEND_LIBRARIES}
                 ${PLATFORM_LIBRARIES}
   )
list( REMOVE_DUPLICATES target_LIBS )



#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )

# Indicate how to link
target_link_libraries( ${PROJECT_NAME} ${target_LIBS} )

get_target_property( FOO ${PROJECT_NAME} LINK_LIBRARIES )
message( "${PROJECT_NAME}:
LINK_LIBRARIES: ${FOO}" )

# Set the application version
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )

# Install Targets
install( TARGETS ${PROJECT_NAME}
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)



