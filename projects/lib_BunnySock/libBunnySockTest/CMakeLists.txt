#==============================================================================
# lib_bunnySock CMake configuration File
#
# Created 7-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(libBunnySockTest)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )


#===============================
# Source Files
#===============================

# Create a list of source files
set( SOURCE_FILES
     LibBunnySockTest.cpp
     LibBunnySock_main.cpp
   )


include_directories( ${LIBBUNNYSOCK_INCLUDE_DIRS}
                     ${MOOS_INCLUDE_DIRS}
                     ${INCLUDE_DIR} )


#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_executable( libBunnySockTest ${SOURCE_FILES} )
                       
# Specify linkage order
target_link_libraries( ${PROJECT_NAME}
                       BunnySock 
                       YellowSubUtils
                       ${MOOS_LIBRARIES} 
                       ${PLATFORM_LIBRARIES} 
                     )

# Specify other libraries our target depends on
add_dependencies( ${PROJECT_NAME} BunnySock YellowSubUtils )

