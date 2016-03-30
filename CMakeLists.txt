#==============================================================================
# U of I YellowSub MOOS Software
# Top-level CMake configuration
#
# Maintained by Dave Billin <david.billin@vandals.uidaho.edu>
#==============================================================================

#==========================
# CMAKE OPTIONS
#==========================
cmake_minimum_required(VERSION 2.8)


#==========================
# CMAKE PROJECT SETTINGS
#==========================
project( moos-ivp-uidaho )

# Project version
set( MOOS_IVP_UIDAHO_VERSION_MAJOR 1 )
set( MOOS_IVP_UIDAHO_VERSION_MINOR 0 )
set( MOOS_IVP_UIDAHO_VERSION_PATCH 0 )

# Set up a version string
set( MOOS_IVP_UIDAHO_VERSION_TUPLE "${MOOS_IVP_UIDAHO_VERSION_MAJOR}.${MOOS_IVP_UIDAHO_VERSION_MINOR}.${MOOS_IVP_UIDAHO_VERSION_PATCH}" )
message( STATUS "Configuring moos-ivp-uidaho version ${MOOS_IVP_UIDAHO_VERSION_TUPLE}"  )


# Define local folders
set( bin_dir ${PROJECT_SOURCE_DIR}/bin )
set( cmake_modules_dir ${PROJECT_SOURCE_DIR}/cmake-modules )
set( data_dir ${PROJECT_SOURCE_DIR}/data )
set( docs_dir ${PROJECT_SOURCE_DIR}/docs )
set( lib_dir ${PROJECT_SOURCE_DIR}/lib )
set( projects_dir ${PROJECT_SOURCE_DIR}/projects )


# Helpful functions for CMake debugging
#include(${cmake_modules_dir}/CMakeUtils.cmake)

# Tell CMake to look in the cmake-modules directory for find modules
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${cmake_modules_dir}" )


#==============================
# *** PROJECT OPTIONS ***
#==============================

# Allow only Linux builds
if( NOT ${LINUX} )
    message("Sorry.  Only Linux targets are supported!")
    return()
endif()


#===============================================
# SET OUTPUT DIRECTORIES
#===============================================
if ( NOT DEFINED CMAKE_RUNTIME_OUTPUT_DIRECTORY )
    set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${bin_dir}" 
         CACHE PATH "Directory where executables built by the project will be written"
       )
endif()
if ( NOT DEFINED CMAKE_ARCHIVE_OUTPUT_DIRECTORY )
    set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${lib_dir}" 
         CACHE PATH "Directory where static libraries built by the project will be written"
       )
endif()
if ( NOT DEFINED CMAKE_LIBRARY_OUTPUT_DIRECTORY )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${lib_dir}" 
         CACHE PATH "Directory where shared libraries built by the project will be written"
       )
endif()

# Add the static library output directory to the linker search path
link_directories( ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )

# Set the directory where man files should be placed prior to installation
set(MANFILE_OUTPUT_DIRECTORY ${docs_dir}/man)

message( STATUS "Building for target CPU: ${CMAKE_SYSTEM_PROCESSOR}" )



#==============================
# Find the IvP libraries
#==============================
message( STATUS "MOOSIVP_SOURCE_TREE_DIR = ${MOOSIVP_SOURCE_TREE_DIR}" )

# Search for the moos-ivp folder
if (NOT DEFINED MOOSIVP_SOURCE_TREE_DIR)
   find_path( MOOSIVP_SOURCE_TREE_DIR
              NAMES build-ivp.sh build-moos.sh configure-ivp.sh
              PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/"
              DOC "Base directory of the MOOS-IvP source tree"
              NO_DEFAULT_PATH
            )
endif()

if (NOT MOOSIVP_SOURCE_TREE_DIR)
    message("Please set MOOSIVP_SOURCE_TREE_DIR to the path of the MOOS-IvP source tree")
    return()
endif()

#============================
# Find the MOOS libraries
#============================
# If the MOOS libraries are found, the following variables will be populated:
#   MOOS_INCLUDE_DIRS           - Path(s) to MOOS library header files
#   MOOS_LIBRARIES              - A list of MOOS libraries
#   MOOS_DEPEND_INCLUDE_DIRS    - Include directories for MOOS dependencies
#   MOOS_DEPEND_LIBRARIES       - Names of libraries that MOOS depends on
find_package( MOOS 10 REQUIRED )
message( STATUS "Found libMOOS" )

#==============================
# Find MOOS Geodesy library
#==============================
# If the MOOSGeodesy library is found, the following variables will be populated:
#   MOOSGeodesy_INCLUDE_DIRS           - Path(s) to MOOSGeodesy header files
#   MOOSGeodesy_LIBRARIES              - Path of the MOOSGeodesy library
#   MOOSGeodesy_DEPEND_INCLUDE_DIRS    - Include directories for MOOSGeodesy dependencies
#   MOOSGeodesy_DEPEND_LIBRARIES       - Names of libraries that MOOSGeodesy depends on
find_package( MOOSGeodesy REQUIRED )
message( STATUS "Found libMOOSGeodesy" )

#==============================
# Import IvP artifacts
#==============================
include( "${cmake_modules_dir}/import-IvP.cmake" )

import_ivp_static_libraries( "${MOOSIVP_SOURCE_TREE_DIR}" )
string( REPLACE ";" ", " comma_list "${IVP_STATIC_LIBS}" )
message( STATUS "  { ${comma_list} }" )

import_ivp_shared_libraries( "${MOOSIVP_SOURCE_TREE_DIR}" )
string( REPLACE ";" ", " comma_list "${IVP_SHARED_LIBS}" )
message( STATUS "  { ${comma_list} }" )

set( ivp_all_libnames "${IVP_STATIC_LIBS};${IVP_SHARED_LIBS}" )
import_ivp_include_dirs( "${MOOSIVP_SOURCE_TREE_DIR}" "${ivp_all_libnames}" )
include_directories( ${IVP_INCLUDE_DIRS} )

foreach( dir IN LISTS IVP_INCLUDE_DIRS )
   #message( "     ${dir}" )
endforeach()

#=================================
# Find libproj4
#=================================
# Set up cache variables specifying paths to search for libproj4
set( PROJ4_INCLUDE_SEARCH_PATHS "${MOOSIVP_SOURCE_TREE_DIR}/MOOS/proj-4.8.0/src" 
     CACHE INTERNAL "Path to search for proj4 headers" )
set( PROJ4_LIBRARY_SEARCH_PATHS "${MOOSIVP_SOURCE_TREE_DIR}/lib" "${MOOSIVP_SOURCE_TREE_DIR}/bin" 
     CACHE INTERNAL "Path to search for proj4 libraries" )

# If the MOOSGeodesy library is found, the following variables will be populated:
#   PROJ4_FOUND                        - Evaluates logical true if libproj4 components were found
#   PROJ4_INCLUDE_DIRS           - Path(s) to libproj4 header files
#   PROJ4_LIBRARIES              - Path of libproj4 
find_package(Proj4 REQUIRED)
add_library( proj SHARED IMPORTED )
set_target_properties( proj
                       PROPERTIES IMPORTED_LOCATION
                       "${PROJ4_LIBRARIES}"
                     )


#====================================================
# Set up a CMake macro for determining version info
#====================================================
macro( GetPatchVersion TARGET_DIR OUTPUT_VAR )
   # DESCRIPTION:  
   #   Returns an integer patch version for a specified directory
   #
   # param TARGET_DIR:  Directory whose version should be returned
   # param OUTPUT_VAR:  CMake variable to populate with a version tuple
   #
   # REMARKS:
   #   In the past, version tuples were created using a subversion 
   #   revision number.  Since version control is now implemented using
   #   git, such a distinct revision number is no longer available.
   #   Therefore, until a more clever alternative is available (or 
   #   even deemed necessary), the global patch revision is returned.
   set( ${OUTPUT_VAR} "-${MOOS_IVP_UIDAHO_VERSION_PATCH}" )
endmacro()





#===========================
# COMPILER WARNINGS
#===========================
set( COMPILER_WARNING_LEVEL "All" CACHE STRING "Compiler warning level to enforce: Normal/All/Extra" )
set_property( CACHE COMPILER_WARNING_LEVEL
              PROPERTY STRINGS Normal All Extra
            )

get_property( CURRENT_WARNING_LEVEL CACHE COMPILER_WARNING_LEVEL PROPERTY VALUE )

if ( CMAKE_COMPILER_IS_GNUCXX OR MSVC )
    if ( ${CURRENT_WARNING_LEVEL} MATCHES "All" )
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" )
    elseif( ${CURRENT_WARNING_LEVEL} MATCHES "Extra" )
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra" )
    endif()
endif()    




#===================================
# PLATFORM-SPECIFIC SETTINGS
#===================================
if(WIN32)
    add_definitions( -DWINDOWS_NT 
                     -D_CRT_SECURE_NO_DEPRECATE 
                     -D_SCL_SECURE_NO_DEPRECATE
                   )
                   
    set(PLATFORM_LIBRARIES wsock32 comctl32 ${PLATFORM_LIBRARIES})
    
elseif(UNIX)
    add_definitions( -DUNIX )
    
    find_package( Threads REQUIRED )    # verify pthread support
    
    #include_directories("/usr/include")

    set( PLATFORM_LIBRARIES rt m pthread ${PLATFORM_LIBRARIES} )

    if( "${CMAKE_SYSTEM_NAME}" MATCHES "SunOS" )
        set( PLATFORM_LIBRARIES socket nsl rt ${PLATFORM_LIBRARIES} )
    endif()
    
    if( "${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" )
        add_definitions( -DPLATFORM_DARWIN )
        set( PLATFORM_DARWIN 1 ) 
    endif()
    
    if( "${CMAKE_SYSTEM_NAME}" MATCHES "Linux" )
        add_definitions( -DPLATFORM_LINUX )
        set( PLATFORM_LINUX 1 ) 
    endif()
endif()





#================================
# UNIT TESTING
#================================
set( UNITTEST_ENABLED OFF CACHE BOOL "Enable building unit tests" )
     
if( UNITTEST_ENABLED )
	message( STATUS "Unit tests will be built.  Run ctest to execute." )
	ENABLE_TESTING()
endif()

#===================================
# Recurse into source directories
#===================================
ADD_SUBDIRECTORY( ${projects_dir} )



#================================
# INSTALLER (PACKAGE) GENERATION
#================================
# This section will set up a configuration file that may be used with the 
# utility CPack to generate installer packages.  To create these packages,
# simply navigate to the folder where this CMakeLists.txt file is located and
# execute the command "cpack".
#
# TODO: Only a small subset of CPack's capabilities are exercised here. Future
#       revisions of this CMake code could add useful things like:
#           - Creation of a Windows installer (NSIS)
#           - Creation of a MacOS drag-and-drop style (DMG) installer
if ( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    set( CPACK_GENERATOR "DEB" )

    set( CPACK_DEBIAN_PACKAGE_NAME "moos-ivp-uidaho" )
    set( CPACK_DEBIAN_PACKAGE_VERSION ${MOOS_IVP_UIDAHO_VERSION_TUPLE} )
    set( CPACK_DEBIAN_PACKAGE_DESCRIPTION "University of Idaho MOOS-IvP apps and libraries" )
    set( CPACK_DEBIAN_PACKAGE_SECTION "devel" )
    set( CPACK_DEBIAN_PACKAGE_PRIORITY "optional" )
    set( CPACK_DEBIAN_PACKAGE_MAINTAINER "david.billin@vandals.uidaho.edu" )

    # Identify debian package dependencies
    set( CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.6), libgcc1 (>= 1:4.1)" )
    if ( UNITTEST_ENABLED )
        set( CPACK_DEBIAN_PACKAGE_DEPENDS "${CPACK_DEBIAN_PACKAGE_DEPENDS},libgtest-devi,cmake" )
    endif()


    #determine the target architecture
    find_program( DPKG_PROGRAM dpkg DOC "dpkg program of Debian-based systems" )
    if( DPKG_PROGRAM )
        execute_process( COMMAND ${DPKG_PROGRAM} --print-architecture
                         OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
                         OUTPUT_STRIP_TRAILING_WHITESPACE
                       )
    endif()

    include(CPack)

    message( "Run 'make package' to generate the moos-ivp-uidaho debian package" )
endif()





#===================================
# DOCUMENTATION
#===================================
#if (UNIX)
#    option( INSTALL_MAN_FILES "Add a rule to install man files" ${UNIX} )
#else()
#    set( INSTALL_MAN_FILES FALSE )
#endif()


#option(INSTALL_HTML_DOCUMENTATION "Add a rule to install HTML documentation" ON)

# These variables are used to enable/disable generation
# of the different types of documentation produced by
# Doxygen.  They are used to fill in values when the
# Doxyfile.in file is processed by CMake
set(MAN_DOCS_ENABLED "NO")
set(HTML_DOCS_ENABLED "YES")


# Add installation of man pages to the main install rule
#if (INSTALL_MAN_FILES)
#    install( DIRECTORY ${DOC_OUTPUT_DIRECTORY}/man/man3
#             DESTINATION /usr/local/man/man1/
#           )
#endif()


# Set the directory that generated documentation will be stored in
set( DOC_OUTPUT_DIRECTORY "${docs_dir}")

find_package(Doxygen QUIET)
     
if (DOXYGEN_FOUND)

    # Generate a Doxygen configuration file using project info
    configure_file (${cmake_modules_dir}/Doxyfile.in 
                    ${cmake_modules_dir}/Doxyfile )

    # Add doxygen as target
    add_custom_command( OUTPUT documentation 
                       COMMAND ${DOXYGEN_EXECUTABLE} ${cmake_modules_dir}/Doxyfile
                       DEPENDS ${cmake_modules_dir}/Doxyfile
                       WORKING_DIRECTORY ${docs_dir}
                     )


    # Clean up docs directory on "make clean"
    set_property( DIRECTORY APPEND 
                  PROPERTY ADDITIONAL_MAKE_CLEAN_FILES 
                  ${docs_dir}/html
                )

    # Add doxygen as dependency to doc-target
    get_target_property(DOC_TARGET doc TYPE)
    if(NOT DOC_TARGET)
        add_custom_target(doc)
    endif()
    add_dependencies(doc doxygen)


    # Set up installation path base for documentation
    set(DOC_PATH "share/doc/${CPACK_PACKAGE_NAME}-${VERSION}")
    
    
    # Install HTML documentation
#    if (INSTALL_HTML_DOCUMENTATION)
#        install( DIRECTORY ${DOC_OUTPUT_DIRECTORY}/html
#                 DESTINATION ${DOC_PATH}
#               )
#    endif()

    if (UNIX)
        message( "run \"make documentation\" to generate Doxygen documentation" )
    endif()

else()
    message( "Doxygen was not found on this computer.  Source code documentation "
             "will not be generated."
           )
endif() 

