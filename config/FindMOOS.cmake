#==============================================================================
# FindMOOS.cmake
#   A CMake module used to find static libraries needed for the MOOS software
#   framework developed by the Oxford Mobile Robotics Group.
#
# Module created 7-2011 by Dave Billin
#
# Description:
#   This module will attempt to locate the static libraries shipped by the MOOS 
#   framework and their platform-specific dependencies.
#       libMOOS
#       libMOOSGen (optional)
#       libMOOSUtility (optional)
#
#   To use this module in your CMake configuration file, use the following
#   command sequence:
#       set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "<this module's path>")
#       find_package(MOOS)
#
# This module accepts the following optional variables:
#
#   MOOS_SOURCE_BASE_DIR
#       If the MOOS libraries have not been installed on the system, or are
#       being built from source, this variable may be set to specify the path 
#       of the base directory in the MOOS source tree.  Typically this is a 
#       folder named "MOOS-<version>-<date>" (or just "MOOS" on UNIX-like 
#       systems that support symbolic links).
#
#   MOOS_INCLUDE_SEARCH_PATHS
#       This variable may be used to specify a list of paths that will be 
#       searched (prior to standard search paths) for MOOS header files.
#
#   MOOS_LIBRARY_SEARCH_PATHS
#       This variable may be used to specify a list of paths that will be 
#       searched (prior to standard search paths) for MOOS library files.
#
#   MOOS_SKIP_MOOSGEN
#       If this variable is set to TRUE, this module will not attempt to find
#       the library libMOOSGen
#
#   MOOS_SKIP_MOOSUTILITY
#       If this variable is set to TRUE, this module will not attempt to find
#       the MOOS utility library, libMOOSUtility
#
#   MOOS_MARK_ADVANCED
#       If this variable is set to TRUE, all cache variables produced by this
#       module will be marked as advanced
#
#
# If the MOOS-IvP libraries are found, they are imported as external targets, 
# allowing other CMake modules to refer to them directly (e.g. to link to them,
# call target_link_libraries(<target_name> MOOS MOOSGen MOOSUtility).  In
# addition, the following symbols are defined:
#
#   MOOS_INCLUDE_DIRECTORIES    A list of directories containing header files
#   MOOS_LIBRARIES              A list of MOOS library (target) names that may
#                               be used with a target_link_libraries() command.
#
#   MOOSLIB_FOUND               TRUE if libMOOS.a was found
#   MOOSLIB_INCLUDE_DIR         Path to libMOOS.a headers
#   MOOSLIB_LIBRARY             Full path to libMOOS.a
#
#   MOOSGENLIB_FOUND            TRUE if libMOOSGen.a was found
#   MOOSGENLIB_INCLUDE_DIR      Path to libMOOSGen.a headers
#   MOOSGENLIB_LIBRARY          Full path to libMOOSGen.a
#
#   MOOSUTILITYLIB_FOUND        TRUE if libMOOSGen was found
#   MOOSUTILITYLIB_INCLUDE_DIR  Path to libMOOSUtility.a utility headers
#   MOOSUTILITYLIB_LIBRARY      Full path of libMOOSUtility.a
#
# In addition, 
#
#==============================================================================
include(FindPackageHandleStandardArgs)

set(MOOS_LIB_LIST MOOS MOOSGEN MOOSUTILITY)
set(MOOS_LIBRARY_NAME_LIST MOOS MOOSGen MOOSUtility)
set(MOOS_HEADER_NAME_LIST MOOSLib.h MOOSGenLib.h MOOSGeodesy.h)

set(MOOS_SKIP_MOOSCORE FALSE)   # This value should always be false


#====================================
# Initialize CMake variables
#====================================
foreach( TARGET ${MOOS_LIB_LIST} )
    set( ${TARGET}LIB_FOUND FALSE)
    unset( ${TARGET}LIB_INCLUDE_DIR )
    set( ${TARGET}LIB_LIBRARY "${TARGET}LIB_LIBRARY-NOTFOUND" )
endforeach( TARGET ${MOOS_LIB_LIST} )


unset(MOOS_INCLUDE_DIRECTORIES)
unset(MOOS_LIBRARIES)


#=============================================
# If a directory is specified as the base of 
# a MOOS source tree, add its relevant
# subdirectories to the search paths
#=============================================
if (MOOS_SOURCE_BASE_DIR)
    
    # Look for a signature file expected in the MOOS base directory
    find_path( MOOS_ROOT_PATH NAMES MOOSConfig.cmake
               DOC "Base directory of the MOOS source tree"
               PATHS ${MOOS_SOURCE_BASE_DIR}
               NO_DEFAULT_PATH
    )

    mark_as_advanced(MOOS_ROOT_PATH)

    if (MOOS_ROOT_PATH)
        set( MOOS_LIBRARY_SEARCH_PATHS 
                        ${MOOS_ROOT_PATH}/MOOSBin 
                        ${MOOS_LIBRARY_SEARCH_PATHS} )
        set( MOOS_INCLUDE_SEARCH_PATHS
                        ${MOOS_ROOT_PATH}/Core/MOOSLIB
                        ${MOOS_ROOT_PATH}/Core/MOOSGenLib
                        ${MOOS_ROOT_PATH}/Essentials/MOOSUtilityLib 
                        ${MOOS_INCLUDE_SEARCH_PATHS} )

    message(STATUS "Found MOOS source tree: ${MOOS_ROOT_PATH}")
    #message("MOOS_LIBRARY_SEARCH_PATHS = ${MOOS_LIBRARY_SEARCH_PATHS}")
    #message("MOOS_INCLUDE_SEARCH_PATHS = ${MOOS_INCLUDE_SEARCH_PATHS}")

    endif (MOOS_ROOT_PATH)

endif (MOOS_SOURCE_BASE_DIR)



#=============================================
# Locate libraries and include directories
#=============================================
# Here, we iterate through the entries of MOOS_LIB_LIST to search for
# all of the MOOS libraries.


foreach( TARGET ${MOOS_LIB_LIST} )
    
    if (NOT MOOS_SKIP_${TARGET})
        # Get the index of TARGET in MOOS_LIB_LIST    
        list( FIND MOOS_LIB_LIST ${TARGET} LIST_INDEX )
        
        # Get a header file name to search for from MOOS_HEADER_NAME_LIST
        list( GET MOOS_HEADER_NAME_LIST ${LIST_INDEX} TARGET_INCLUDE_NAME )

        # Get the library file name to search for from MOOS_LIBRARY_NAME_LIST
        list( GET MOOS_LIBRARY_NAME_LIST ${LIST_INDEX} TARGET_LIBRARY_NAME )

        #------------------------------------------
        # Find the path to the target library's 
        # header files
        #------------------------------------------
        find_path( ${TARGET}LIB_INCLUDE_DIR 
                   NAMES ${TARGET_INCLUDE_NAME}
                   PATHS ${MOOS_INCLUDE_SEARCH_PATHS}
                 )

        #message("\${TARGET}LIB_INCLUDE_DIR = ${TARGET}LIB_INCLUDE_DIR = ${${TARGET}LIB_INCLUDE_DIR}")
        #message("TARGET_LIBRARY_NAME = ${TARGET_LIBRARY_NAME}")

        #------------------------------------------
        # Find the target library itself
        #------------------------------------------
        #message("Locating MOOS library: ${TARGET_LIBRARY_NAME}")
        find_library( ${TARGET}LIB_LIBRARY
                      ${TARGET_LIBRARY_NAME}
                      DOC "${TARGET}LIB header file directory"
                      PATHS ${MOOS_LIBRARY_SEARCH_PATHS}
                    )

        #message("\${TARGET}LIB_LIBRARY = ${TARGET}LIB_LIBRARY = ${${TARGET}LIB_LIBRARY}")

        #-----------------------------------
        # Verify we found both library 
        # and header file directory
        #-----------------------------------
        if ( ${TARGET}LIB_INCLUDE_DIR AND ${TARGET}LIB_LIBRARY )
            message(STATUS "Found library: ${TARGET_LIBRARY_NAME}")
            set( ${TARGET}LIB_FOUND TRUE )
        endif ( ${TARGET}LIB_INCLUDE_DIR AND ${TARGET}LIB_LIBRARY )


        #-----------------------------------
        # Add header and library 
        # to the master sets
        #-----------------------------------
        set( MOOS_INCLUDE_DIRECTORIES 
             ${${TARGET}LIB_INCLUDE_DIR} ${MOOS_INCLUDE_DIRECTORIES}  )

        set( MOOS_LIBRARIES
             ${MOOS_LIBRARIES} ${TARGET_LIBRARY_NAME} )

    
        # Import the library as an external target
        add_library( ${TARGET_LIBRARY_NAME} STATIC IMPORTED)
        set_property( TARGET ${TARGET_LIBRARY_NAME} 
                      PROPERTY IMPORTED_LOCATION ${${TARGET}LIB_LIBRARY})


    endif (NOT MOOS_SKIP_${TARGET})

endforeach( TARGET ${MOOS_LIB_LIST} )


#message("MOOS_INCLUDE_DIRECTORIES = ${MOOS_INCLUDE_DIRECTORIES}")
#message("MOOS_LIBRARIES = ${MOOS_LIBRARIES}")



#=============================================
# Build a list of variables to be verified 
# in order to return MOOS_FOUND
#=============================================
unset(MOOS_CHECKLIST)
foreach( TARGET ${MOOS_LIB_LIST} )
    if (NOT MOOS_SKIP_${TARGET})
        # Handle the MOOS_MARK_ADVANCED option
        if (MOOS_MARK_ADVANCED)
            mark_as_advanced( ${TARGET}LIB_INCLUDE_DIR ${TARGET}LIB_LIBRARY )
        endif (MOOS_MARK_ADVANCED)

        #set( MOOS_CHECKLIST "${MOOS_CHECKLIST} ${TARGET}LIB_INCLUDE_DIR ${TARGET}LIB_LIBRARY" )
        list( APPEND MOOS_CHECKLIST 
              ${TARGET}LIB_INCLUDE_DIR ${TARGET}LIB_LIBRARY )
    endif (NOT MOOS_SKIP_${TARGET})
endforeach( TARGET ${IVP_LIB_LIST} )


#message( "MOOS_CHECKLIST = ${MOOS_CHECKLIST}" )


#foreach( TARGET ${MOOS_LIB_LIST} )
#    message( "${TARGET}LIB_FOUND = ${${TARGET}LIB_FOUND}" )
#    message( "${TARGET}LIB_INCLUDE_DIR = ${${TARGET}LIB_INCLUDE_DIR}" )
#    message( "${TARGET}LIB_LIBRARY = ${${TARGET}LIB_LIBRARY}" )
#endforeach( TARGET ${MOOS_LIB_LIST} )

FIND_PACKAGE_HANDLE_STANDARD_ARGS( MOOS 
     "The MOOS libraries do not appear to be installed on this system.  If you have built them from source, but have not installed them, set the CMake variable MOOS_SOURCE_BASE_DIR to the path of the MOOS source tree base (e.g. \"MOOS\" or \"MOOS-2300-Dec0309\")\n"
      "${MOOS_CHECKLIST}" )

if (MOOS_FOUND)
    # Since a great many of Paul's #include statements are hard-coded as
    # rooted to a subdirectory of "Core" in the MOOS source tree, we 
    # prepend this path to the MOOS_INCLUDE_DIRECTORIES list as a 
    # convenience to callers
    if (MOOS_ROOT_PATH)
        set( MOOS_INCLUDE_DIRECTORIES ${MOOS_INCLUDE_DIRECTORIES} ${MOOS_ROOT_PATH}/Core )
    endif (MOOS_ROOT_PATH)       

    message(STATUS "All MOOS libraries successfully located")
endif (MOOS_FOUND)


