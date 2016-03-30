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


set(PROJECT_VERSION_MAJOR  1)
set(PROJECT_VERSION_MINOR  0)
GetPatchVersion( ${PROJECT_SOURCE_DIR} PROJECT_VERSION_PATCH )

set(PROJECT_VERSION 
    "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
)

set(PROJECT_AUTHOR "Dave Billin")
set(PROJECT_EMAIL "dave.billin@vandals.uidaho.edu")


# Fill in fields of templated files
configure_file(${PROJECT_SOURCE_DIR}/build/AppVersion.h.in
               ${PROJECT_SOURCE_DIR}/AppVersion.h )
configure_file( "${PROJECT_SOURCE_DIR}/build/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )


#===============================
# Source Files
#===============================

set( ${PROJECT_NAME}_HEADERLIST
     BHV_ReverseDive.h
)

# Create a list of source files
set( ${PROJECT_NAME}_SRCLIST
     BHV_ReverseDive.cpp
)

include_directories( ${MOOS_INCLUDE_DIRS} )
include_directories( ${MOOSIVP_INCLUDE_DIRECTORIES} )



#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( ${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCLIST} )


# Specify linkage order
target_link_libraries( ${PROJECT_NAME} 
                       ${MOOS_LIBRARIES}
                       ${MOOSIVP_LIBRARIES} 
                       ${PLATFORM_LIBRARIES} 
)


#============================
# Add an install rule
#============================
# Install target
INSTALL( TARGETS ${PROJECT_NAME}
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

