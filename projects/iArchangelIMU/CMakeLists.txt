#==============================================================================
# iArchangelIMU CMake configuration file
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iArchangelIMU)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH  0 )

set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )


# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( config_dir "${PROJECT_SOURCE_DIR}/config" )
set( docs_dir ${PROJECT_SOURCE_DIR}/docs )
set( test_dir ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs} )
endif()

# Fill in fields of templated files
configure_file(${config_dir}/config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file( "${config_dir}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


#===============================
# Source Files
#===============================

include_directories( ${include_dir}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${MOOS_INCLUDE_DIRS}
                   )


set( SOURCE_FILES
        ${src_dir}/iArchangelIMU.cpp
        ${src_dir}/IMU3Module.cpp
        ${src_dir}/main.cpp
)


#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
#set_target_properties( ${PROJECT_NAME} PROPERTIES 
#                       VERSION ${VERSION_TUPLE} )

# Indicate how to link
target_link_libraries( ${PROJECT_NAME}
                       YellowSubUtils
                       ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                )

set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                        
add_dependencies( ${PROJECT_NAME} YellowSubUtils )

# Install Targets
install(TARGETS ${PROJECT_NAME}
       RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)




