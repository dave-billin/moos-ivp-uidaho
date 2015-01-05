#==============================================================================
# FindMOOSIvP.cmake
#   A CMake module used to find static libraries provided by the MOOS-IvP robot
#   autonomy software suite developed at the Computer Science and Artificial
#   Intelligence (CSAIL) laboratory at MIT in Cambridge, Massachussets.
#
# Module created 7-2011 by Dave Billin
#
# Description:
#   This module will attempt to locate all of the (non-GUI-oriented) static 
#   libraries shipped by the MOOS-IvP software suite and their respective 
#   platform-specific dependencies.  The target libraries include:
#       libbehaviors.a
#       libbehaviors-marine.a
#       libbhvutil.a
#       libgenutil.a
#       libgeometry.a
#       libivpbuild.a
#       libivpcore.a
#       liblogic.a
#       liblogutils.a
#       libmbutil.a
#       proj4 (library built for MOOSGeodesy implementation)
#
#   To use this module in your CMake configuration file, just add the two
#   lines:
#     set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "<path of FindMOOSIvP.cmake>")
#     find_package(MOOSIvP)
#
#  ===============
#  *   OPTIONS   *
#  ===============
#
# This module supports the standard REQUIRED and QUIET options.  In addition,
# the following CMake variables may be set before calling find_package(MOOSIvP) 
# to change specific functionality:
#
#   MOOSIVP_SOURCE_BASE_DIR
#       If the MOOS-IvP libraries have not been installed on the system, or are
#       being built from source, this variable may be set to specify the path 
#       of the "moos-ivp" directory (base directory of the IvP source tree).
#
#   MOOSIVP_MARK_ADVANCED
#       If this variable is set to TRUE, all cache variables produced by this
#       module will be marked as advanced
#
#   MOOSIVP_INCLUDE_SEARCH_PATHS
#       This variable may be used to specify a list of paths that will be 
#       searched (prior to standard search paths) for MOOS-IvP header files.
#
#   MOOSIVP_LIBRARY_SEARCH_PATHS
#       This variable may be used to specify a list of paths that will be 
#       searched (prior to standard search paths) for MOOS-IvP library files.
#
#
#
#  ===============
#  *   OUTPUT    *
#  ===============
# If the MOOS-IvP libraries are found, they are imported as external targets, 
# allowing other CMake modules to refer to them directly (e.g. by calling
# target_link_libraries(<target_name> behaviors bhvutil genutil).  In
# addition, the following symbols are defined:
#
#   LIBBEHAVIORS_FOUND          TRUE if libbehaviors.a was found
#   LIBBEHAVIORS_INCLUDE_DIR    Path of the libbehaviors header files
#   LIBBEHAVIORS_LIBRARY        Full path of libbehaviors.a
#
#   LIBBEHAVIORS_MARINE_FOUND         TRUE if libbehaviors-marine.a was found
#   LIBBEHAVIORS_MARINE_INCLUDE_DIR   Path of the libbehaviors-marine headers
#   LIBBEHAVIORS_MARINE_LIBRARY       Full path of libbehaviors-marine.a
#
#   LIBBHVUTIL_FOUND            TRUE if libbhvutil.a was found
#   LIBBHVUTIL_INCLUDE_DIR      Path of the libbhvutil header files
#   LIBBHVUTIL_LIBRARY          Full path of libbhvutil.a
#
#   LIBGENUTIL_FOUND            TRUE if libgenutil.a was found
#   LIBGENUTIL_INCLUDE_DIR      Path of the libgenutil header files
#   LIBGENUTIL_LIBRARY          Full path of libgenutil.a
#
#   LIBGEOMETRY_FOUND           TRUE if libgeometry.a was found
#   LIBGEOMETRY_INCLUDE_DIR     Path of the libgeometry header files
#   LIBGEOMETRY_LIBRARY         Full path of libgeometry.a
#
#   LIBHELMIVP_FOUND            TRUE if libhelmivp.a was found
#   LIBHELMIVP_INCLUDE_DIR      Path of the libhelmivp header files
#   LIBHELMIVP_LIBRARY          Full path of libhelmivp.a
#
#   LIBIVPBUILD_FOUND           TRUE if libivpbuild.a was found
#   LIBIVPBUILD_INCLUDE_DIR     Path of the libivpbuild header files
#   LIBIVPBUILD_LIBRARY         Full path of libivpbuild.a
#
#   LIBIVPCORE_FOUND            TRUE if libivpcore.a was found
#   LIBIVPCORE_INCLUDE_DIR      Path of the libivpcore header files
#   LIBIVPCORE_LIBRARY          Full path of libivpcore.a
#
#   LIBLOGIC_FOUND              TRUE if liblogic.a was found
#   LIBLOGIC_INCLUDE_DIR        Path of the liblogic header files
#   LIBLOGIC_LIBRARY            Full path of liblogic.a
#
#   LIBLOGUTILS_FOUND           TRUE if liblogutils.a was found
#   LIBLOGUTILS_INCLUDE_DIR     Path of the liblogutils header files
#   LIBLOGUTILS_LIBRARY         Full path of liblogutils.a
#
#   LIBMBUTIL_FOUND             TRUE if libmbutil.a was found
#   LIBMBUTIL_INCLUDE_DIR       Path of the libmbutil header files
#   LIBMBUTIL_LIBRARY           Full path of libmbutil.a
#
#   MOOSIVP_INCLUDE_DIRECTORIES A list of the directories containing MOOS-IvP 
#                               header files.
#
#   MOOSIVP_LIBRARIES           A list of MOOS library (target) names that may
#                               be used with a target_link_libraries() command.
#
#==============================================================================
include(FindPackageHandleStandardArgs)

set( MOOSIVP_LIB_LIST BEHAVIORS
                      BEHAVIORS_MARINE
                      BHVUTIL
                      GENUTIL 
                      GEOMETRY
                      HELMIVP
                      IVPBUILD
                      IVPCORE 
                      LOGIC 
                      LOGUTILS 
                      MBUTIL 
)

set(MOOSIVP_LIBRARY_NAME_LIST 
        behaviors
        behaviors-marine
        bhvutil
        genutil
        geometry
        helmivp
        ivpbuild
        ivpcore
        logic
        logutils
        mbutil
)

set( MOOSIVP_HEADER_NAME_LIST 
        IvPBehavior.h
        BHV_Loiter.h
        LoiterEngine.h
        MOOSAppRunnerThread.h
        GeomUtils.h
        BehaviorReport.h
        ZAIC_PEAK.h
        IvPFunction.h
        LogicUtils.h
        LogUtils.h
        MBUtils.h
)


#====================================
# Initialize CMake variables
#====================================
foreach( TARGET ${IVP_LIB_LIST} )
    set( LIB${TARGET}_FOUND FALSE )
    set( LIB${TARGET}_INCLUDE_DIR "LIB${TARGET}_INCLUDE_DIR-NOTFOUND" )
    set( LIB${TARGET}_LIBRARY "LIB${TARGET}_LIBRARY-NOTFOUND" )
endforeach( TARGET ${IVP_LIB_LIST} )

unset(MOOSIVP_INCLUDE_DIRECTORIES)
unset(MOOSIVP_LIBRARIES)


#=============================================
# CASE: Caller has specified the path of the
# "moos-ivp" directory, so search it first.
#=============================================
if (MOOSIVP_SOURCE_BASE_DIR)
    
    # Look for a signature file expected in the MOOS base directory
    find_path( MOOSIVP_ROOT_PATH NAMES build-ivp.sh
               DOC "Base directory of the MOOS-IvP source tree"
               PATHS ${MOOSIVP_SOURCE_BASE_DIR}
               NO_DEFAULT_PATH
    )

    mark_as_advanced(MOOSIVP_ROOT_PATH)
    
    # Populate additional directories to search for headers and libraries
    set( MOOSIVP_LIBRARY_SEARCH_PATHS 
                ${MOOSIVP_ROOT_PATH}/lib 
                ${MOOSIVP_LIBRARY_SEARCH_PATHS} )
    set( MOOSIVP_INCLUDE_SEARCH_PATHS 
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_behaviors
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_behaviors-marine
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_bhvutil
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_genutil
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_geometry
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_helmivp
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_ivpbuild
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_ivpcore
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_logic
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_logutils
                ${MOOSIVP_ROOT_PATH}/ivp/src/lib_mbutil
                ${MOOSIVP_INCLUDE_SEARCH_PATHS}
        )

    message(STATUS "Found MOOS-IvP source tree: ${MOOS_ROOT_PATH}")
    #message("MOOSIVP_LIBRARY_SEARCH_PATHS = ${MOOSIVP_LIBRARY_SEARCH_PATHS}")
    #message("MOOSIVP_INCLUDE_SEARCH_PATHS")
    #foreach( DIR ${MOOSIVP_INCLUDE_SEARCH_PATHS} )
    #    message(\t${DIR})
    #endforeach( DIR ${MOOSIVP_INCLUDE_SEARCH_PATHS} )

endif (MOOSIVP_SOURCE_BASE_DIR)



#=============================================
# Locate libraries and include directories
#=============================================
# Here, we iterate through the entries of IVP_LIB_LIST to search for
# all of the MOOS-IvP libraries.

foreach( TARGET ${MOOSIVP_LIB_LIST} )

    # Get the index of TARGET in MOOSIVP_LIB_LIST    
    list( FIND MOOSIVP_LIB_LIST ${TARGET} LIST_INDEX )
    
    # Get a header file name to search for from MOOSIVP_HEADER_NAME_LIST
    list( GET MOOSIVP_HEADER_NAME_LIST ${LIST_INDEX} TARGET_INCLUDE_NAME )

    # Get the library file name to search for from MOOS_LIBRARY_NAME_LIST
    list( GET MOOSIVP_LIBRARY_NAME_LIST ${LIST_INDEX} TARGET_LIBRARY_NAME )

    #message("LIST_INDEX = ${LIST_INDEX}")
    #message("TARGET_INCLUDE_NAME = ${TARGET_INCLUDE_NAME}")
    #message("TARGET_LIBRARY_NAME = ${TARGET_LIBRARY_NAME}")

    #------------------------------------------
    # Find the path to the target library's 
    # header files
    #------------------------------------------  
    find_path( LIB${TARGET}_INCLUDE_DIR 
               NAMES ${TARGET_INCLUDE_NAME}
               PATHS ${MOOSIVP_INCLUDE_SEARCH_PATHS}
             )

    #message("LIB\${TARGET}_INCLUDE_DIR = LIB${TARGET}_INCLUDE_DIR = ${LIB${TARGET}_INCLUDE_DIR}")


    #------------------------------------------
    # Find the target library itself
    #------------------------------------------
    #message("Locating MOOS-IvP library: ${TARGET_LIBRARY_NAME}")
    find_library( LIB${TARGET}_LIBRARY
                  ${TARGET_LIBRARY_NAME}
                  DOC "${TARGET} header file directory"
                  PATHS ${MOOSIVP_LIBRARY_SEARCH_PATHS} 
                )

    #message("LIB\${TARGET}_LIBRARY = LIB${TARGET}_LIBRARY = ${LIB${TARGET}_LIBRARY}")

    #-----------------------------------
    # Verify we found both library 
    # and header file directory
    #-----------------------------------
    if ( LIB${TARGET}_INCLUDE_DIR AND LIB${TARGET}_LIBRARY )
        message(STATUS "Found library: ${TARGET_LIBRARY_NAME}")
        set( LIB${TARGET}_FOUND TRUE )
    endif ( LIB${TARGET}_INCLUDE_DIR AND LIB${TARGET}_LIBRARY )


    #-----------------------------------
    # Add header and library 
    # to the master sets
    #-----------------------------------
    set( MOOSIVP_INCLUDE_DIRECTORIES 
         ${MOOSIVP_INCLUDE_DIRECTORIES} ${LIB${TARGET}_INCLUDE_DIR} )

    set( MOOSIVP_LIBRARIES
         ${MOOSIVP_LIBRARIES} ${TARGET_LIBRARY_NAME} )

    # Import the library as an external target
    add_library( ${TARGET_LIBRARY_NAME} STATIC IMPORTED)
    set_property( TARGET ${TARGET_LIBRARY_NAME} 
                  PROPERTY IMPORTED_LOCATION ${LIB${TARGET}_LIBRARY})

endforeach( TARGET ${MOOSIVP_LIB_LIST} )



#=============================================
# Build a list of variables to be verified 
# in order to return MOOS_FOUND
#=============================================
unset(MOOSIVP_CHECKLIST)
foreach( TARGET ${MOOSIVP_LIB_LIST} )

    # Handle the MOOS_MARK_ADVANCED option
    if (MOOSIVP_MARK_ADVANCED)
        mark_as_advanced( LIB${TARGET}_INCLUDE_DIR LIB${TARGET}_LIBRARY )
    endif (MOOSIVP_MARK_ADVANCED)

    #set( MOOSIVP_CHECKLIST "${MOOSIVP_CHECKLIST} LIB${TARGET}_INCLUDE_DIR LIB${TARGET}_LIBRARY" )
    list( APPEND MOOSIVP_CHECKLIST 
          LIB${TARGET}_INCLUDE_DIR LIB${TARGET}_LIBRARY )

endforeach( TARGET ${MOOSIVP_LIB_LIST} )

#message("MOOSIVP_CHECKLIST = ${MOOSIVP_CHECKLIST}")
#message( "MOOSIVP_CHECKLIST =" )
#foreach( TARGET ${MOOSIVP_CHECKLIST} )
#    message(\t${TARGET})
#endforeach( TARGET ${MOOSIVP_CHECKLIST} )

FIND_PACKAGE_HANDLE_STANDARD_ARGS( MOOSIVP 
     "The IvP libraries do not appear to be installed on this system.  If you have built them from source, but have not installed them, you may specify the path of the base directory (\"moos-ivp\") of the MOOS-IvP source tree in the CMake variable MOOSIVP_SOURCE_BASE_DIR\n"
      ${MOOSIVP_CHECKLIST} )


if (MOOSIVP_FOUND)
    # Add the IvP root directory to include paths to enable inclusion
    # of arbitrary paths in the IvP sources
    message(STATUS "All MOOS-IvP libraries successfully located")
endif (MOOSIVP_FOUND)

