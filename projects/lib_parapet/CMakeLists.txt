#==============================================================================
# lib_parapet CMake configuration File
#
# Created January 2013 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(parapet)

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
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files to be installed with the library
set( HEADER_FILES
     ${INCLUDE_DIR}/parapet_config.h
     ${INCLUDE_DIR}/parapet_descriptor.h
     ${INCLUDE_DIR}/parapet_header.h
     ${INCLUDE_DIR}/parapet_node.h
     ${INCLUDE_DIR}/parapet_packet_fifo.h
     ${INCLUDE_DIR}/parapet.h
     ${INCLUDE_DIR}/ParapetDeviceNode.h
     ${INCLUDE_DIR}/ParapetEventBroadcaster.h
     ${INCLUDE_DIR}/ParapetListener.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${SRC_DIR}/parapet_header.c
     ${SRC_DIR}/parapet_node.c
     ${SRC_DIR}/parapet.c
     ${SRC_DIR}/ParapetDeviceNode.cpp
     ${SRC_DIR}/ParapetEventBroadcaster.cpp
     ${SRC_DIR}/ParapetPacketFIFO.cpp
   )


include_directories( ${MOOS_INCLUDE_DIRECTORIES}
                     ${INCLUDE_DIR} )


#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( ${PROJECT_NAME} STATIC ${SOURCE_FILES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                       
# Specify linkage order
target_link_libraries( ${PROJECT_NAME}
                       YellowSubUtils
                       ${MOOS_LIBRARIES} 
                       ${PLATFORM_LIBRARIES} 
                     )


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

    # Offer a GUI option to build unit tests for this module 
    set( UNITTEST_${PROJECT_NAME}_ENABLED ON CACHE BOOL 
         "Build unit tests for the ${PROJECT_NAME} library" )
    
    if ( UNITTEST_${PROJECT_NAME}_ENABLED )
        include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
    
        # Add subdirectories containing unit test projects here
        add_subdirectory(test)
    endif()
    
endif()

