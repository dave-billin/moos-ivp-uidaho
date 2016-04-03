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
set( PROJECT_EMAIL "david.billin@vandals.uidaho.edu" )

# Set project version info
set( VERSION_MAJOR 1 )
set( VERSION_MINOR 0 )
set( VERSION_PATCH  0 )
set( VERSION_TUPLE "${VERSION_MAJOR}.${VERSION_MINOR}-${VERSION_PATCH}" )

# Define project directories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( config_dir "${PROJECT_SOURCE_DIR}/config" )
set( docs_dir ${PROJECT_SOURCE_DIR}/docs )
set( test_dir ${PROJECT_SOURCE_DIR}/test )
if ( NOT DEFINED MANFILE_OUTPUT_DIRECTORY )
    set( MANFILE_OUTPUT_DIRECTORY ${docs_dir} )
endif()

#---------------------------------------
# Verify that libgps is installed
#---------------------------------------
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${config_dir} )

# If libgps is found, the following variables are populated:
#  GPS_FOUND - system has gps
#  GPS_INCLUDE_DIR - the gps include directory
#  GPS_LIBRARY - Link this to use gps library
find_package( gps MODULE )

if ( NOT GPS_FOUND )
    set( gps_not_found_msg_list
         "libgps could not be found in this system, so iGPSd will not be built.  "
         "Debian- and Ubuntu-based systems provide the necessary files in the 'libgps-dev' package"
       )
    message( WARNING ${gps_not_found_msg_list} )

    return()
endif()


#===========================================================================
# If Google protocol buffers are available in the system, build a shared
# library, libiGPSdProtobuf, that can be used to serialize and de-serialize
# data from gpsd for sending it through the MOOS database as binary data.
#
# Other applications can tie into this library in their CMake configuration
# using the following variables:
#   IGPSD_PROTOBUF_INCLUDE_DIRS
#     A lists of directories containing libiGPSdProtobuf header files.  Set
#     to "IGPSD_PROTOBUF-NOTFOUND" if protocol buffers are not available
#
#   IGPSD_PROTOBUF_LIBRARIES
#     The libiGPSdProtobuf target for calls to target_link_libraries(). Set
#     to "IGPSD_PROTOBUF-NOTFOUND" if protocol buffers are not available
#
# The availability of the libiGPSdProtobuf will cause iGPSd to be built with
# support for the functionality it supplies.  Otherwise, iGPSd will only
# publish gpsd data to 'regular' MOOS variables.
#===========================================================================
find_package( Protobuf QUIET MODULE )

# If the protobuffer library is found, the following variables are populated:
#  PROTOBUF_FOUND - Found libprotobuf & header files
#  PROTOBUF_INCLUDE_DIR - Include directories for Google Protocol Buffers
#  PROTOBUF_LIBRARIES - Link this to use protobuf libraries
if ( PROTOBUF_FOUND )
   message( STATUS "Found Google Protocol Buffers!" )
   message( STATUS "iGPSd will be built with protocol buffer support" )
   
   set( generated_iGPSd_FixData_pb_h "${CMAKE_CURRENT_BINARY_DIR}/iGPSd_FixData.pb.h" )
   set( generated_iGPSd_FixData_pb_cc "${CMAKE_CURRENT_BINARY_DIR}/iGPSd_FixData.pb.cc" )
   set( protobuf_generated_files "${generated_iGPSd_FixData_pb_h}" "${generated_iGPSd_FixData_pb_cc}" )
   ADD_CUSTOM_COMMAND(
           OUTPUT  ${generated_iGPSd_FixData_pb_h} ${generated_iGPSd_FixData_pb_cc}
           COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
           ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR} iGPSd_FixData.proto
           DEPENDS ${config_dir}/iGPSd_FixData.proto
           WORKING_DIRECTORY ${config_dir}
           COMMENT "Invoking Protobuf compiler on 'iGPSd_FixData.proto'"
            )
   
   # Create a rule to build the shared library containing routines 
   # for working with serialized iGPSD protobuffer data structures 
   add_library( iGPSdProtobuf SHARED
                ${generated_iGPSd_FixData_pb_h}
                ${generated_iGPSd_FixData_pb_cc} 
              )
              
   set_target_properties( iGPSdProtobuf PROPERTIES
                          VERSION 1.0.0
                          SOVERSION 1 
                        )

   # Export the location of generated header files into parent scope to make 
   # them available to other moos-ivp-uidaho subprojects
   set( IGPSD_PROTOBUF_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE )
   set( IGPSD_PROTOBUF_LIBRARIES iGPSdProtobuf PARENT_SCOPE )
   
   # Create installation targets for the shared library and header files
   install( TARGETS iGPSdProtobuf
            RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
            LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
            ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
          )

   install( FILES ${generated_iGPSd_FixData_pb_h}
            DESTINATION ${CMAKE_INSTALL_PREFIX}/include )
         
else()
    message( "Google Protocol Buffers could not be found in this system.  iGPSd will be built without protobuf features." )
    message( "Debian and Ubuntu-based distributions provide the necessary files in the 'libprotobuf-dev' and 'protobuf-compiler' packages." )
    set( IGPSD_PROTOBUF_INCLUDE_DIRS "IGPSD_PROTOBUF-NOTFOUND" )
    set( IGPSD_PROTOBUF_LIBRARIES    "IGPSD_PROTOBUF-NOTFOUND" )
endif()



# Generate files from templates
set( generated_config_h  "${CMAKE_CURRENT_BINARY_DIR}/config.h" )
set( generated_manfile_1 "${MANFILE_OUTPUT_DIRECTORY}/${PROJECT_NAME}.1" )
set( generated_files
     ${generated_config_h}
     ${generated_manfile_1} 
   )
configure_file(${config_dir}/config.h.in   "${generated_config_h}" )
configure_file( "${config_dir}/manfile.in" "${generated_manfile_1}" )


#===============================
# iGPSd executable Source Files
#===============================

include_directories( ${include_dir}
                     ${CMAKE_CURRENT_BINARY_DIR}
                     ${MOOS_INCLUDE_DIRS} 
                     ${GPS_INCLUDE_DIR} 
                     ${PROJECT_SOURCE_DIR} )

set( SOURCE_FILES 
     ${src_dir}/iGPSd.cpp
     ${src_dir}/main.cpp
)

set( IGPSD_LIBRARIES ${MOOS_LIBRARIES}
                     ${GPS_LIBRARY}
                     ${PLATFORM_LIBRARIES}
   )

if ( IGPSD_PROTOBUF_LIBRARIES )
   add_definitions( -DUSE_GOOGLE_PROTOCOL_BUFFERS )
   include_directories( ${IGPSD_PROTOBUF_INCLUDE_DIRS} )
   list( APPEND IGPSD_LIBRARIES ${IGPSD_PROTOBUF_LIBRARIES} )
endif()


#============================
# Specify the project target
#============================

# Create an executable file
add_executable( iGPSd ${SOURCE_FILES} ${generated_files} )

# Indicate how to link the iGPSd executable
target_link_libraries( iGPSd
                       ${IGPSD_LIBRARIES}
                     )

set_target_properties( iGPSd PROPERTIES 
                       VERSION ${VERSION_TUPLE} 
                     )

#============================
# Add an install rule
#============================
install( TARGETS iGPSd
	      RUNTIME DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
       )
