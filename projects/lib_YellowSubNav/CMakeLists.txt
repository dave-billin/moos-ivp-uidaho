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
GetSvnWorkingCopyRevision( ${PROJECT_SOURCE_DIR} VERSION_PATCH )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( DOCS_DIR ${PROJECT_SOURCE_DIR}/docs )
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# *** The YellowSubNav library depends on the NewMat matrix library *** 
include_directories( ${NEWMAT_INCLUDE_DIRECTORIES} 
                     ${MOOS_INCLUDE_DIRECTORIES}
                     ${MOOSGeodesy_INCLUDE_DIRS}
                     ${INCLUDE_DIR}
                   )

# Create a list of header files
set( HEADER_FILES
     ${INCLUDE_DIR}/EKF.h
     ${INCLUDE_DIR}/LblBeacon.h
     ${INCLUDE_DIR}/LblPositionEstimator.h
     ${INCLUDE_DIR}/VehicleEKF.h
     ${INCLUDE_DIR}/LibYellowSubNav.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${SRC_DIR}/EKF.cpp
     ${SRC_DIR}/LblBeacon.cpp
     ${SRC_DIR}/LblPositionEstimator.cpp
     ${SRC_DIR}/VehicleEKF.cpp
   )



#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( ${PROJECT_NAME} STATIC ${SOURCE_FILES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                       
# Specify libraries to link to
target_link_libraries( ${PROJECT_NAME} 
                       ${MOOS_LIBRARIES} 
                       ${PLATFORM_LIBRARIES}
                       Newmat10
                     )

# Identify targets that must be built prior to this project
#add_dependencies( ${PROJECT_NAME} Newmat )



# Pass the target's header file directories one level up in the CMake tree
string( TOUPPER ${PROJECT_NAME} PROJECT_NAME_UCASE )
set( ${PROJECT_NAME_UCASE}_INCLUDE_DIRECTORIES 
     ${INCLUDE_DIR} 
     PARENT_SCOPE 
)



#============================
# Add an install rule
#============================
# Install headers
INSTALL( FILES ${HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include )

# Install target
install( TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)




#===============================
# Unit Tests
#===============================
if( UNITTEST_ENABLED )
    # Offer a GUI option to build unit tests
    set( UNITTEST_VehicleEKFTest_ENABLED ON CACHE BOOL 
         "Build unit tests for the ${PROJECT_NAME} library" )
    
    IF ( UNITTEST_VehicleEKFTest_ENABLED )
        # Add the project's header directories to the search path for unit tests
        set( ${PROJECT_NAME_UCASE}_INCLUDE_DIRECTORIES 
             ${PROJECT_SOURCE_DIR} )
    
        # Add subdirectories containing unit test projects here
        add_subdirectory(VehicleEKFTest)
    ENDIF()
endif()




