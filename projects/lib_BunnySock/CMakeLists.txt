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
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )


#===============================
# Source Files
#===============================

# Create a list of header files to be installed with the library
set( INSTALLED_HEADER_FILES
     ${include_dir}/BunnySockListener.h
     ${include_dir}/BunnySockListenerFIFO.h
     ${include_dir}/BunnySockNode.h
     ${include_dir}/BunnySockPacket.h
     ${include_dir}/BunnySockTcpNode.h
     ${include_dir}/BunnySockUdpNode.h
     ${include_dir}/FileSelector.h
     ${include_dir}/LibBunnySock.h
     ${include_dir}/UAVnet_PacketTypes.h
     ${include_dir}/XPTcpSocketEx.h
   )


# Create a list of source files
set( SOURCE_FILES
     ${src_dir}/BunnySockListenerFIFO.cpp
     ${src_dir}/BunnySockNode.cpp
     ${src_dir}/BunnySockPacket.cpp
     ${src_dir}/BunnySockTcpNode.cpp
     ${src_dir}/BunnySockUdpNode.cpp
     ${src_dir}/FileSelector.cpp
     ${src_dir}/LibBunnySock.cpp
     ${src_dir}/XPTcpSocketEx.cpp
   )


include_directories( ${include_dir}
                     ${LIBYELLOWSUBUTILS_INCLUDE_DIRS}
                     ${MOOS_INCLUDE_DIRS}
                   )


#============================
# Specify the project target
#============================

# Define the shared library target this project will build
add_library( ${PROJECT_NAME} SHARED ${SOURCE_FILES} )
set_target_properties( BunnySock PROPERTIES 
                       VERSION ${VERSION_TUPLE}
                       SOVERSION ${VERSION_MAJOR} 
                     )
                       
# Specify linkage order
target_link_libraries( BunnySock
                       YellowSubUtils
                       ${MOOS_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )


# Pass the target's header file directories one level up in the CMake tree
string( TOUPPER ${PROJECT_NAME} PROJECT_NAME_UCASE )
set( LIBBUNNYSOCK_INCLUDE_DIRS ${include_dir} PARENT_SCOPE )
set( LIBBUNNYSOCK_LIBRARIES BunnySock PARENT_SCOPE )



#============================
# Add an install rule
#============================
# Install headers
INSTALL( FILES ${INSTALLED_HEADER_FILES} 
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/BunnySock )

# Install target
INSTALL( TARGETS BunnySock
         RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
         ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
       )



#====================
# Unit Tests
#====================
if( BUILD_UNIT_TESTS )
   # Offer a cache option to build unit tests
   set( BUILD_BUNNYSOCK_TESTS ON CACHE BOOL "Build the libBunnySock test application" )
   
   IF ( BUILD_BUNNYSOCK_TESTS )
      include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
   
      # Add subdirectories containing unit test projects here
      add_subdirectory(libBunnySockTest)
   ENDIF()
endif()

