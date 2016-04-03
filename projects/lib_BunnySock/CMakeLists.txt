#==============================================================================
# lib_bunnySock CMake configuration File
#
# Created 7-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(BunnySock)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files to be installed with the library
set( HEADER_FILES
     ${INCLUDE_DIR}/BunnySockListener.h
     ${INCLUDE_DIR}/BunnySockListenerFIFO.h
     ${INCLUDE_DIR}/BunnySockNode.h
     ${INCLUDE_DIR}/BunnySockPacket.h
     ${INCLUDE_DIR}/BunnySockTcpNode.h
     ${INCLUDE_DIR}/BunnySockUdpNode.h
     ${INCLUDE_DIR}/FileSelector.h
     ${INCLUDE_DIR}/LibBunnySock.h
     ${INCLUDE_DIR}/UAVnet_PacketTypes.h
     ${INCLUDE_DIR}/XPTcpSocketEx.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${SRC_DIR}/BunnySockListenerFIFO.cpp
     ${SRC_DIR}/BunnySockNode.cpp
     ${SRC_DIR}/BunnySockPacket.cpp
     ${SRC_DIR}/BunnySockTcpNode.cpp
     ${SRC_DIR}/BunnySockUdpNode.cpp
     ${SRC_DIR}/FileSelector.cpp
     ${SRC_DIR}/LibBunnySock.cpp
     ${SRC_DIR}/XPTcpSocketEx.cpp
   )


include_directories( ${MOOS_INCLUDE_DIRS}
                     ${INCLUDE_DIR} )


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
                       ${PLATFORM_LIBRARIES} 
                       YellowSubUtils 
                     )

# Specify other libraries our target depends on
add_dependencies( ${PROJECT_NAME} YellowSubUtils )


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
INSTALL( FILES ${${PROJECT_NAME}_HEADERLIST} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME} )

# Install target
INSTALL( TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)



#====================
# Unit Tests
#====================
if( UNITTEST_ENABLED )
    # Offer a GUI option to build unit tests
    set( UNITTEST_libBunnySockTest_ENABLED ON CACHE BOOL 
         "Build unit tests for the ${PROJECT_NAME} library" )
    
    IF ( UNITTEST_libBunnySockTest_ENABLED )
        include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
    
        # Add subdirectories containing unit test projects here
        add_subdirectory(libBunnySockTest)
    ENDIF()
endif()

