#==============================================================================
# VehicleEKFTest CMake configuration file
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(VehicleEKFTest)


#===============================
# Source Files
#===============================

# Gather all header files and source files in the project directory
file( GLOB ${PROJECT_NAME}_HEADERS *.h *.hpp )
file( GLOB ${PROJECT_NAME}_SRC *.cpp )

include_directories(${NEWMAT_INCLUDE_DIRECTORIES} )
include_directories( ${MOOS_INCLUDE_DIRS} )
include_directories( ${YELLOWSUBNAV_INCLUDE_DIRECTORIES} )



#============================
# Specify the project target
#============================
add_executable( ${PROJECT_NAME} ${${PROJECT_NAME}_SRC} )
target_link_libraries( ${PROJECT_NAME} 
                       Newmat10 
                       YellowSubNav 
                       ${MOOS_LIBRARIES} 
                       ${MOOSGeodesy_LIBRARIES} 
                     )

# Identify libraries that must be built prior to this project
add_dependencies( ${PROJECT_NAME} Newmat10 ${MOOS_LIBRARIES} )



#============================
# Add an install rule
#============================
# Don't install unit tests!


