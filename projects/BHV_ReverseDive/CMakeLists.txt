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


set(VERSION_MAJOR  1)
set(VERSION_MINOR  0)
set(VERSION_PATCH  0)

set(VERSION_TUPLE 
    "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"
)

set(PROJECT_AUTHOR "Dave Billin")
set(PROJECT_EMAIL "dave.billin@vandals.uidaho.edu")

set( config_dir "${PROJECT_SOURCE_DIR}/config" )


# Fill in fields of templated files
# NOTE: AppVersion.h is generated and written to the location given by
#       CMAKE_CURRENT_BINARY_DIR
configure_file(${config_dir}/AppVersion.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/AppVersion.h )
configure_file( "${config_dir}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


#===============================
# Source Files
#===============================

set( BHV_ReverseDive_HEADER_FILES
     BHV_ReverseDive.h
)

# Create a list of source files
set( BHV_ReverseDive_SOURCES
     BHV_ReverseDive.cpp
   )


foreach( dir IN LISTS IVP_INCLUDE_DIRS )
   #message( "     ${dir}" )
endforeach()

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
             ${CMAKE_CURRENT_BINARY_DIR}/AppVersion.h
           )


# Specify linkage order
target_link_libraries( ${PROJECT_NAME} 
                       ${MOOS_LIBRARIES}
                       ivpbuild
                       ${PLATFORM_LIBRARIES} 
)


#============================
# Add an install rule
#============================
# Install target
INSTALL( TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

