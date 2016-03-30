#==============================================================================
# import-IvP.cmake
#   A CMake module providing functions used to import libraries and header 
#   files supplied in the MOOS-IvP source tree.
#
# Module maintained by Dave Billin <david.billin@vandals.uidaho.edu>
#
# Description:
#   This module attempts to locate all static and shared libraries produced by
#   the IvP sources.  It is assumed that these libraries reside in either the
#   bin or lib subdirectory of the IvP source tree base directory.  Each
#   library found is imported as a CMake IMPORT target. 
# 
#   To use this module in your CMake build, add the following line to your
#   CMake configuration file (typically CMakeLists.txt):
#     set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "<directory containing import-IvP.cmake>")
#     include( Import-IvP.cmake )
#

#==============================================================================
# DESCRIPTION: 
#   This function locates static libraries in the MOOS-IvP source tree and
#   imports them as targets.  The MOOS, MOOSGeodesy, and proj libraries are
#   ignored by this function if they are found
#
# PARAM ivp_base_dir    Path of the base directory of the MOOS-IvP source tree
#
# OUTPUTS: 
#   IVP_STATIC_LIBS   A list of imported static library target names
#==============================================================================
function( import_ivp_static_libraries  ivp_base_dir )

   # Exclude the MOOS and MOOSGeodesy libraries
   set( EXCLUDED_LIBS "MOOS" "MOOSGeodesy" "proj" )
   
   message( STATUS "Importing static libraries from the IvP source tree" )
   
   set( IVP_STATIC_LIBS "" )
   set( search_dir "${ivp_base_dir}/${dir}" )
   
   # glob for static libraries in the search directory
   file(GLOB_RECURSE static_libs "${search_dir}/*.a")
   foreach( libpath IN LISTS static_libs )
      # Get the name of the library (e.g. no 'lib' prefix or file extension)
      get_filename_component( libname_we "${libpath}" NAME_WE )
      string( REPLACE "lib" "" libname "${libname_we}" )
      
      # Implement excluded libraries
      list(FIND EXCLUDED_LIBS ${libname} is_excluded)
      if ( ${is_excluded} EQUAL "-1" ) 
         add_library( ${libname} STATIC IMPORTED )
         set_target_properties( ${libname} 
                                PROPERTIES IMPORTED_LOCATION 
                                "${libpath}" 
                              )
         list( APPEND IVP_STATIC_LIBS "${libname}" )
      endif()
   endforeach()

   set( IVP_STATIC_LIBS "${IVP_STATIC_LIBS}" PARENT_SCOPE)
endfunction()

#==============================================================================
# DESCRIPTION: 
#   This function locates shared libraries in the MOOS-IvP source tree and
#   imports them as targets.  The MOOS, MOOSGeodesy, and proj libraries are
#   ignored by this function if they are found
#
# PARAM ivp_base_dir    Path of the base directory of the MOOS-IvP source tree
#
# OUTPUTS: 
#   IVP_SHARED_LIBS   A list of imported shared library target names
#==============================================================================
function( import_ivp_shared_libraries  ivp_base_dir )

   # Exclude the MOOS and MOOSGeodesy libraries
   set( EXCLUDED_LIBS "MOOS" "MOOSGeodesy" "proj" )
   
   message( STATUS "Importing shared libraries from the IvP source tree" )
   # Search IvP source tree directories for libraries

   set( IVP_SHARED_LIBS "" )
   set( search_dir "${ivp_base_dir}/${dir}" )
   
   # glob for shared libraries in the search directory
   file(GLOB_RECURSE shared_libs "${search_dir}/*.so")
   foreach( libpath IN LISTS shared_libs )
      # Get the name of the library (e.g. no 'lib' prefix or file extension)
      get_filename_component( libname_we "${libpath}" NAME_WE )
      string( REPLACE "lib" "" libname "${libname_we}" )
      
      # Implement excluded libraries
      list(FIND EXCLUDED_LIBS ${libname} is_excluded)
      if ( ${is_excluded} EQUAL "-1" ) 
         add_library( ${libname} SHARED IMPORTED )
         set_target_properties( ${libname} 
                                PROPERTIES IMPORTED_LOCATION 
                                "${libpath}" 
                              )
         list( APPEND IVP_SHARED_LIBS "${libname}" )
      endif()
   endforeach()

   set( IVP_SHARED_LIBS "${IVP_SHARED_LIBS}" PARENT_SCOPE)
endfunction()


#==============================================================================
# DESCRIPTION: 
#   This function adds include directories for libraries in the MOOS-IvP source
#    tree.
#
# PARAM ivp_base_dir    Path of the base directory of the MOOS-IvP source tree
# PARAM ivp_lib_names   List of ivp libraries to find include directories for
#
# OUTPUTS: 
#   IVP_INCLUDE_DIRS   A list of include directories for IvP libraries
#==============================================================================
function( import_ivp_include_dirs  ivp_base_dir ivp_lib_names )
   message( STATUS "Locating include directories in the IvP source tree" )
   
   set( ivp_src_dir "${ivp_base_dir}/ivp/src" )
   set( IVP_INCLUDE_DIRS "" )
   
   foreach( libname IN LISTS ivp_lib_names )
      set( dir "${ivp_src_dir}/lib_${libname}" )
      if(IS_DIRECTORY "${dir}" )
         list( APPEND IVP_INCLUDE_DIRS "${dir}" )
      endif()
      
   endforeach()
   
   set( IVP_INCLUDE_DIRS "${IVP_INCLUDE_DIRS}" PARENT_SCOPE )
   
endfunction()