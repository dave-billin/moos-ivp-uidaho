#==============================================================================
# pVehicleID CMake configuration file
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(pVehicleID)

set( PROJECT_AUTHOR "Brandt Pedrow" )
set( PROJECT_EMAIL "pedr5603@vandals.uidaho.edu" )

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
     ${src_dir}/pVehicleID.cpp
     ${src_dir}/pVehicleIDMain.cpp
)

#============================
# Specify the project target
#============================

# Create an executable file
add_executable( pVehicleID ${SOURCE_FILES} ${generated_files} )
set_target_properties( pVehicleID PROPERTIES 
                       VERSION ${VERSION_TUPLE} )

# Indicate how to link
target_link_libraries( pVehicleID
                       ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )

#============================
# Add an install rule
#============================
# Install Targets
install( TARGETS pVehicleID
         RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
       )
