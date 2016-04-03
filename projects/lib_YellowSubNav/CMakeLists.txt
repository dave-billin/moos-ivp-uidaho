#==============================================================================
# YellowSubNav library CMake Configuration
#
# Created 7-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project( YellowSubNav )

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )


#===============================
# Source Files
#===============================

# *** The YellowSubNav library depends on the NewMat matrix library *** 
include_directories( ${include_dir}
                     ${LIBNEWMAT_INCLUDE_DIRS}
                     ${MOOS_INCLUDE_DIRS}
                     ${MOOSGeodesy_INCLUDE_DIRS}
                     ${PROJ4_INCLUDE_DIRS}
                   )

# Create a list of header files
set( HEADER_FILES
     ${include_dir}/EKF.h
     ${include_dir}/LblBeacon.h
     ${include_dir}/LblPositionEstimator.h
     ${include_dir}/VehicleEKF.h
     ${include_dir}/LibYellowSubNav.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${src_dir}/EKF.cpp
     ${src_dir}/LblBeacon.cpp
     ${src_dir}/LblPositionEstimator.cpp
     ${src_dir}/VehicleEKF.cpp
   )



#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( YellowSubNav SHARED ${SOURCE_FILES} )
set_target_properties( YellowSubNav PROPERTIES 
                       VERSION ${VERSION_TUPLE}
                       SOVERSION ${VERSION_MAJOR}
                     )
                       
# Specify libraries to link to
target_link_libraries( YellowSubNav 
                       ${MOOS_LIBRARIES} 
                       ${PLATFORM_LIBRARIES}
                       ${LIBNEWMAT_LIBRARIES}
                     )

# Pass the target's header file directories one level up in the CMake tree
set( LIBYELLOWSUBNAV_INCLUDE_DIRS ${include_dir} PARENT_SCOPE ) 
set( LIBYELLOWSUBNAV_LIBRARIES YellowSubNav PARENT_SCOPE )



#============================
# Add an install rule
#============================
# Install headers
INSTALL( FILES ${HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include )

# Install target
install( TARGETS YellowSubNav
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


#===============================
# Unit Tests
#===============================
if( UNITTEST_ENABLED )
    # Offer a GUI option to build unit tests
    set( BUILD_VEHICLE_EKF_TEST ON CACHE BOOL 
         "Build the libYellowSubNav Vehicle EKF test application" )
    
    if ( BUILD_VEHICLE_EKF_TEST )
        # Add subdirectories containing unit test projects here
        add_subdirectory(VehicleEKFTest)
    endif()
endif()




