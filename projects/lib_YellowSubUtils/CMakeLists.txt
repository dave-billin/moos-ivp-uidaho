#==============================================================================
# lib_yellowsubutils CMake Configuration
#
# Created 7-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project( YellowSubUtils )

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
GetPatchVersion( ${PROJECT_SOURCE_DIR} VERSION_PATCH )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( DOCS_DIR ${PROJECT_SOURCE_DIR}/docs )
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs} )
endif()


#===============================
# Source Files
#===============================

# Create a list of header files
set( HEADER_FILES
     ${INCLUDE_DIR}/WindowedAverage.h
     ${INCLUDE_DIR}/YellowSubUtils.h
     ${INCLUDE_DIR}/DebugFileSerialPort.h
     ${INCLUDE_DIR}/DSPFilter.h
     ${INCLUDE_DIR}/DSPFilterParameters.h
     ${INCLUDE_DIR}/FIRFilter.h
     ${INCLUDE_DIR}/LinuxSerialPortEx.h
     ${INCLUDE_DIR}/MockSerialPort.h
     ${INCLUDE_DIR}/MOOSDBVariableProxy.h
     ${INCLUDE_DIR}/MOOSDBDoubleProxy.h
     ${INCLUDE_DIR}/MOOSDBStringProxy.h
     ${INCLUDE_DIR}/MOOSDBProxyCollection.h
     ${INCLUDE_DIR}/PolyphaseFIRDecimator.h
     ${INCLUDE_DIR}/PrecisionTime.h
     ${INCLUDE_DIR}/Semaphore.h
     ${INCLUDE_DIR}/ScalarPIDController.h
     ${INCLUDE_DIR}/StringUtilities.h
     ${INCLUDE_DIR}/TimedLock.h
)

# Create a list of source files
set( SOURCE_FILES
     ${SRC_DIR}/DebugFileSerialPort.cpp
     ${SRC_DIR}/LinuxSerialPortEx.cpp
     ${SRC_DIR}/MockSerialPort.cpp
     ${SRC_DIR}/MOOSDBVariableProxy.cpp
     ${SRC_DIR}/MOOSDBDoubleProxy.cpp
     ${SRC_DIR}/MOOSDBStringProxy.cpp
     ${SRC_DIR}/MOOSDBProxyCollection.cpp
     ${SRC_DIR}/PrecisionTime.cpp
     ${SRC_DIR}/Semaphore.cpp
     ${SRC_DIR}/ScalarPIDController.cpp
     ${SRC_DIR}/StringUtilities.cpp
     ${SRC_DIR}/TimedLock.cpp
)


include_directories( ${MOOS_INCLUDE_DIRECTORIES} 
                     ${INCLUDE_DIR} )

set( ${PROJECT_NAME}_LIBS  )

#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( ${PROJECT_NAME} SHARED ${SOURCE_FILES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )


# Specify linkage order
target_link_libraries( ${PROJECT_NAME}
                       ${MOOS_LIBRARIES} 
                       ${MOOS_DEPEND_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )

# Pass the target's header file directories one level up in the CMake tree
string( TOUPPER ${PROJECT_NAME} PROJECT_NAME_UCASE )
set( ${PROJECT_NAME_UCASE}_INCLUDE_DIRECTORIES 
     ${INCLUDE_DIR} 
     PARENT_SCOPE 
)


#set(CMAKE_REQUIRED_LIBRARIES ${PROJECT_NAME} ${CMAKE_REQUIRED_LIBRARIES} )




#===================================
# Add a rule to install the target
#===================================
# Install headers
INSTALL( FILES ${HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME} )

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
    add_subdirectory(test)
endif()





