#==============================================================================
# Copyright (c) 2015  Dave Billin <david.billin@vandals.uidaho.edu>
# 
# This module is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#==============================================================================
#
# FindMOOS.cmake
#   A CMake module for locating the proj4 library
#
# Description:
#   This module attempts to locate the proj4 library and its associated
#   header file directories.
#
#   To use this module in your CMake configuration file, use the following
#   command sequence:
#       set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} <path of this module>)
#       find_package(Proj4)
#
# This module accepts the following optional variables:
#
#   PROJ4_INCLUDE_SEARCH_PATHS
#       This variable may be used to specify a list of directories that 
#       will be searched prior to standard search paths for libproj4 
#       header files.
#
#   PROJ4_LIBRARY_SEARCH_PATHS
#       This variable may be used to specify a list of directories that 
#       will be searched prior to standard search paths for libproj4 
#       libraries.
#
# After executing routines in this module, the following variables will
# be populated:
#
#   PROJ4_FOUND
#      Evaluates logical TRUE if proj4 libraries and headers were
#      successfully located
#
#   PROJ4_INCLUDE_DIRS
#      If proj4 header files were found, this variable contains paths
#      of directories containing proj4 header files.  Otherwise, this
#      variable is populated with PROJ4-NOTFOUND
#
#   PROJ4_LIBRARIES
#      If proj4 libraries were found, this variable contains the paths
#      of those libraries.   Otherwise, this variable is populated with
#      PROJ4-NOTFOUND
#
#==============================================================================
include(FindPackageHandleStandardArgs)

#------------------------------
# Find proj4 libraries
#------------------------------
find_library( PROJ4_LIBRARY NAMES proj
              HINTS ${PROJ4_LIBRARY_SEARCH_PATHS}
            )
   
# Find MOOS include directories
find_path( PROJ4_INCLUDE_DIR
           NAMES proj_api.h
           HINTS ${PROJ4_INCLUDE_SEARCH_PATHS}
         )
   
# Handle the QUIETLY and REQUIRED arguments and set PROJ4_FOUND to TRUE
# if all elements were found
find_package_handle_standard_args( Proj4
                                   DEFAULT_MSG
                                   PROJ4_LIBRARY
                                   PROJ4_INCLUDE_DIR
                                 )

mark_as_advanced( PROJ4_INCLUDE_DIR PROJ4_LIBRARY )
set( PROJ4_LIBRARIES ${PROJ4_LIBRARY} )
set( PROJ4_INCLUDE_DIRS ${PROJ4_INCLUDE_DIR} )

