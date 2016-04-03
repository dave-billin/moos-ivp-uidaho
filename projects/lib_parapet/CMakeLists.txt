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
set( VERSION_PATCH 0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( test_dir ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files to be installed with the library
set( HEADER_FILES
     ${include_dir}/parapet_config.h
     ${include_dir}/parapet_descriptor.h
     ${include_dir}/parapet_header.h
     ${include_dir}/parapet_node.h
     ${include_dir}/parapet_packet_fifo.h
     ${include_dir}/parapet.h
     ${include_dir}/ParapetDeviceNode.h
     ${include_dir}/ParapetEventBroadcaster.h
     ${include_dir}/ParapetListener.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${src_dir}/parapet_header.c
     ${src_dir}/parapet_node.c
     ${src_dir}/parapet.c
     ${src_dir}/ParapetDeviceNode.cpp
     ${src_dir}/ParapetEventBroadcaster.cpp
     ${src_dir}/ParapetPacketFIFO.cpp
   )


include_directories( ${include_dir}
                     ${LIBYELLOWSUBUTILS_INCLUDE_DIRS}
                     ${MOOS_INCLUDE_DIRS}
                   )


#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_library( parapet STATIC ${SOURCE_FILES} )
set_target_properties( parapet PROPERTIES 
                       VERSION ${VERSION_TUPLE} 
                     )
                       
# Specify linkage order
target_link_libraries( parapet
                       YellowSubUtils
                       ${MOOS_LIBRARIES} 
                       ${PLATFORM_LIBRARIES} 
                     )


# Pass the target's header file directories one level up in the CMake tree
set( PARAPET_INCLUDE_DIRS ${include_dir} PARENT_SCOPE )
set( PARAPET_LIBRARIES parapet PARENT_SCOPE )


#============================
# Add an install rule
#============================
# Install headers
INSTALL( FILES ${HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/libparapet )

# Install target
INSTALL( TARGETS parapet
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)



#====================
# Unit Tests
#====================

if( UNITTEST_ENABLED )

    # Offer a GUI option to build unit tests for this module 
    set( BUILD_LIBPARAPET_UNIT_TESTS ON CACHE BOOL "Build libparapet automated unit tests" )
    
    if ( BUILD_LIBPARAPET_UNIT_TESTS )
        # Add subdirectories containing unit test projects here
        add_subdirectory(test)
    endif()
    
endif()

