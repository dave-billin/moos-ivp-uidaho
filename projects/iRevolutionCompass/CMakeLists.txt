#==============================================================================
# iRevolutionCompass CMake configuration file
#
# Created CREATION_DATE by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iRevolutionCompass)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "david.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH  0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Define project directories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( config_dir "${PROJECT_SOURCE_DIR}/config" )
set( docs_dir ${PROJECT_SOURCE_DIR}/docs )
set( test_dir ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${docs_dir} )
endif()

# Generate files from templates
set( generated_config_h  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )
set( generated_manfile_1 "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )
set( generated_files
     ${generated_config_h}
     ${generated_manfile_1} 
   )
configure_file(${config_dir}/config.h.in   "${generated_config_h}" )
configure_file( "${config_dir}/manfile.in" "${generated_manfile_1}" )


#===============================
# Source Files
#===============================

include_directories( ${include_dir}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${MOOS_INCLUDE_DIRS}
                   )


set( SOURCE_FILES
     ${src_dir}/iRevolutionCompass.cpp
     ${src_dir}/RevolutionCompassModule.cpp
     ${src_dir}/main.cpp
)


#============================
# Specify the project target
#============================

# Create an executable file
add_executable( iRevolutionCompass ${SOURCE_FILES} )
set_target_properties( iRevolutionCompass PROPERTIES 
                       VERSION ${VERSION_TUPLE} )

# Indicate how to link
target_link_libraries( iRevolutionCompass
                       YellowSubUtils
                       ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )

set_target_properties( iRevolutionCompass PROPERTIES 
                       VERSION ${VERSION_TUPLE} )


#============================
# Add an install rule
#============================
# Install Targets
install( TARGETS iRevolutionCompass
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       )



