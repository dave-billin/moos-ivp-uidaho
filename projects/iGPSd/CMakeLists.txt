#==============================================================================
# iGPSd CMake configuration file
#
# Created CREATION_DATE by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project(iGPSd)

set( PROJECT_AUTHOR "Dave Billin" )
set( PROJECT_EMAIL "dave.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
GetPatchVersion( ${PROJECT_SOURCE_DIR} VERSION_PATCH )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}${VERSION_PATCH}" )



#---------------------------------------
# Verify that libgps is installed
#---------------------------------------
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/build )

# If libgps is found, the following variables are populated:
#  GPS_FOUND - system has gps
#  GPS_INCLUDE_DIR - the gps include directory
#  GPS_LIBRARY - Link this to use gps library
find_package( gps MODULE )

if ( NOT GPS_FOUND )
    message( WARNING "libgps could not be found in the system, so iGPSd will not be built" )
    return()
endif()

# Un-comment this line after the project is placed under svn source control
GetPatchVersion( ${PROJECT_SOURCE_DIR} PROJECT_VERSION_PATCH )


set(PROJECT_VERSION 
    "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}"
)

set(PROJECT_AUTHOR "Dave Billin")
set(PROJECT_EMAIL "david.billin@vandals.uidaho.edu")


# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( DOCS_DIR ${PROJECT_SOURCE_DIR}/docs )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs} )
endif()



#=========================================================
# If the Google protocol buffers library is available in 
# the system:
#   - Enable protobuf features in iGPSd
#   - Build & install a shared library containing
#     routines for accessing iGPSd serialized data 
#=========================================================
find_package( Protobuf QUIET MODULE )

# If the protobuffer library is found, the following variables are populated:
#  PROTOBUF_FOUND - Found libprotobuf & header files
#  PROTOBUF_INCLUDE_DIR - Include directories for Google Protocol Buffers
#  PROTOBUF_LIBRARIES - Link this to use protobuf libraries
if ( PROTOBUF_FOUND )
    message(STATUS "iGPS will be built with Google protobuf support" )
    
    add_definitions( -DUSE_GOOGLE_PROTOCOL_BUFFERS )


    ADD_CUSTOM_COMMAND(
            OUTPUT ${BUILD_DIR}/iGPSd_FixData.pb.cc ${BUILD_DIR}/iGPSd_FixData.pb.h
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            ARGS --cpp_out=${BUILD_DIR} iGPSd_FixData.proto
            DEPENDS ${BUILD_DIR}/iGPSd_FixData.proto
            WORKING_DIRECTORY ${BUILD_DIR}
            COMMENT "Invoking Protobuf compiler: ${BUILD_DIR}/iGPSd_FixData.proto"
             )


    # Create a rule to build the shared library containing routines 
    # for working with serialized iGPSD protobuffer data structures 
    add_library( iGPSdProtobuf SHARED ${BUILD_DIR}/iGPSd_FixData.pb.cc )
    set_target_properties( iGPSdProtobuf PROPERTIES
                           VERSION 1.0.0
                           SOVERSION 1 
                         )


    # Create installation targets for the shared library and header files
    install( TARGETS iGPSdProtobuf
             RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
             LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
             ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
           )

    install( FILES ${PROTOBUF_HEADERS}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/include )
          
else()
    message( "Google Protocol Buffers libraries not found.  iGPSd will be built without Google protobuf support." )
    message( "Protocol Buffers can be installed in Debian distributions using the libprotobuf-dev package." )
    set( PROTOBUF_LIBRARIES "" )
    set( PROTOBUF_INCLUDE_DIR "" )
endif()



# Process templates to generate common files
configure_file(${BUILD_DIR}/AppVersion.h.in
               ${INCLUDE_DIR}/AppVersion.h )
configure_file( "${BUILD_DIR}/manfile.in"
                "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )



#===============================
# iGPSd executable Source Files
#===============================

include_directories( ${MOOS_INCLUDE_DIRECTORIES} 
                     ${GPS_INCLUDE_DIR} 
                     ${INCLUDE_DIR}
                     ${BUILD_DIR}
                     ${PROJECT_SOURCE_DIR} )

if ( PROTOBUF_FOUND )
    include_directories( ${PROTOBUF_INCLUDE_DIRS} )
endif()


set( SOURCE_FILES 
     ${SRC_DIR}/iGPSd.cpp
     ${SRC_DIR}/main.cpp
)



#============================
# Specify the project target
#============================

# Create an executable file
add_executable( ${PROJECT_NAME} ${SOURCE_FILES} )
set_target_properties( ${PROJECT_NAME} PROPERTIES 
                       VERSION ${VERSION_TUPLE} )
                       
set( IGPSD_LIBRARIES ${MOOS_LIBRARIES}
                     ${PLATFORM_LIBRARIES}
                     ${GPS_LIBRARY}
   )
  
if ( PROTOBUF_FOUND )
    set( IGPSD_LIBRARIES ${IGPSD_LIBRARIES} ${PROTOBUF_LIBRARIES} )
endif()
  
  
   
if( PROTOBUF_FOUND )
    set( IGPSD_LIBRARIES ${IGPSD_LIBRARIES} iGPSdProtobuf )
endif()


# Indicate how to link the iGPSd executable
target_link_libraries( ${PROJECT_NAME}
                       ${IGPSD_LIBRARIES}
					 )

# iGPSd install rule 
install( TARGETS ${PROJECT_NAME}
	     RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
       )




