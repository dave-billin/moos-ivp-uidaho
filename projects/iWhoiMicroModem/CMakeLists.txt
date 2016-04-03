#==============================================================================
# iWhoiMicroModem CMake configuration file
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iWhoiMicroModem)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "david.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( config_dir ${PROJECT_SOURCE_DIR}/config )
set( docs_dir ${PROJECT_SOURCE_DIR}/docs )
set( test_dir ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${docs_dir} )
endif()


# Fill in fields of templated files
set( generated_config_h "${CMAKE_CURRENT_BINARY_DIR}/config.h" )
set( generated_manfile_1 "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )
set( generated_files 
     ${generated_config_h} ${generated_manfile_1}
   )
configure_file(${config_dir}/config.h.in
               ${generated_config_h} )
configure_file( ${config_dir}/manfile.in
                ${generated_manfile_1} )


#===============================
# Source Files
#===============================

include_directories( ${include_dir}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${LIBYELLOWSUBUTILS_INCLUDE_DIRS}
                     ${MOOS_INCLUDE_DIRS} 
                   )


# Create a list of source files
set( SOURCE_FILES
     ${src_dir}/QueuedCommandEvent.cpp
     ${src_dir}/WhoiCommandManager.cpp
     ${src_dir}/iWhoiMicroModem.cpp
     ${src_dir}/main.cpp
   )

#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} 
                ${SOURCE_FILES} 
                ${generated_files}
              )
              
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                       
# Indicate how to link
target_link_libraries( ${PROJECT_NAME}
                       YellowSubUtils
                       ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )

# Install Targets
install( TARGETS ${PROJECT_NAME}
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       )
