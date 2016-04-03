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
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( test_dir ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files
set( HEADER_FILES
     ${include_dir}/WindowedAverage.h
     ${include_dir}/YellowSubUtils.h
     ${include_dir}/DebugFileSerialPort.h
     ${include_dir}/DSPFilter.h
     ${include_dir}/DSPFilterParameters.h
     ${include_dir}/FIRFilter.h
     ${include_dir}/LinuxSerialPortEx.h
     ${include_dir}/MockSerialPort.h
     ${include_dir}/MOOSDBVariableProxy.h
     ${include_dir}/MOOSDBDoubleProxy.h
     ${include_dir}/MOOSDBStringProxy.h
     ${include_dir}/MOOSDBProxyCollection.h
     ${include_dir}/PolyphaseFIRDecimator.h
     ${include_dir}/PrecisionTime.h
     ${include_dir}/Semaphore.h
     ${include_dir}/ScalarPIDController.h
     ${include_dir}/StringUtilities.h
     ${include_dir}/TimedLock.h
)

# Create a list of source files
set( SOURCE_FILES
     ${src_dir}/DebugFileSerialPort.cpp
     ${src_dir}/LinuxSerialPortEx.cpp
     ${src_dir}/MockSerialPort.cpp
     ${src_dir}/MOOSDBVariableProxy.cpp
     ${src_dir}/MOOSDBDoubleProxy.cpp
     ${src_dir}/MOOSDBStringProxy.cpp
     ${src_dir}/MOOSDBProxyCollection.cpp
     ${src_dir}/PrecisionTime.cpp
     ${src_dir}/Semaphore.cpp
     ${src_dir}/ScalarPIDController.cpp
     ${src_dir}/StringUtilities.cpp
     ${src_dir}/TimedLock.cpp
)


include_directories( ${MOOS_INCLUDE_DIRS} 
                     ${include_dir} )


#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( YellowSubUtils SHARED ${SOURCE_FILES} )
set_target_properties( YellowSubUtils PROPERTIES 
                       VERSION ${VERSION_TUPLE}
                       SOVERSION ${VERSION_MAJOR}
                     )


# Specify linkage order
target_link_libraries( YellowSubUtils
                       ${MOOS_LIBRARIES} 
                       ${MOOS_DEPEND_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )

# Pass the target's header file directories one level up in the CMake tree
set( LIBYELLOWSUBUTILS_INCLUDE_DIRS ${include_dir} PARENT_SCOPE )
set( LIBYELLOWSUBUTILS_LIBRARIES YellowSubUtils PARENT_SCOPE )

#===================================
# Add a rule to install the target
#===================================
# Install headers
INSTALL( FILES ${HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/libYellowSubUtils )

# Install target
install( TARGETS YellowSubUtils
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

#===============================
# Unit Tests
#===============================
if( BUILD_UNIT_TESTS )
    add_subdirectory(test)
endif()





