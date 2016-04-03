#==============================================================================
# CMake configuration file for the IvP Helm behavior BHV_ReverseDive
#
# Author: Dave Billin
#
# History:
#   07-25-2011:     DB: Created for University of Idaho IvP build tree
#==============================================================================

#===============================
# Project Settings
#===============================
project(BHV_ReverseDive)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH  0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

set( config_dir "${PROJECT_SOURCE_DIR}/config" )


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

# Create a list of source files
set( BHV_ReverseDive_SOURCES
     BHV_ReverseDive.cpp
   )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${MOOS_INCLUDE_DIRS}
                     ${IVP_INCLUDE_DIRS} 
                   )



#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( ${PROJECT_NAME} SHARED
             ${BHV_ReverseDive_SOURCES}
             ${generated_files}
           )

# Specify linkage order
target_link_libraries( BHV_ReverseDive 
                       ${MOOS_LIBRARIES}
                       ivpbuild
                       ${PLATFORM_LIBRARIES} 
                     )

set_target_properties( BHV_ReverseDive PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                       
#============================
# Add an install rule
#============================
# Install Targets
install( TARGETS BHV_ReverseDive
         LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
       )

