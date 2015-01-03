#==============================================================================
# Gumstix OMAP CMake toolchain file
#
# Created August 2011 by Dave Billin
#
# Description
#   This file may be passed to CMake to configure it for cross-compiling to the 
#   TI OMAP3 CPU on Gumstix OVERO modules.  To use it with a CMake build tree
#   from the command line, type:
#
#     cmake -DCMAKE_TOOLCHAIN_FILE=<path to toolchain-gumstix-omap3.cmake> .
#
# If using one of the CMake GUI wrappers, this file may be specified as the
# toolchain file to use when initially configuring the project.
#==============================================================================

# Set the name of the OS being built for
SET(CMAKE_SYSTEM_NAME Linux)

# Set the version fo the OS being built for
SET(CMAKE_SYSTEM_VERSION 2.6.33)

# Set the type of processor being built for
set(CMAKE_SYSTEM_PROCESSOR "omap3")


# specify the cross-compiler tools
SET(CMAKE_C_COMPILER   /usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER /usr/local/angstrom/arm/bin/arm-angstrom-linux-gnueabi-g++)

# Add some flags for OMAP and NEON optimizations
SET(OMAP_CXX_FLAG_LIST -march=armv7-a 
                    -ffast-math 
                    -fno-math-errno 
                    -mtune=cortex-a8 
                    -mfloat-abi=softfp 
                    -mfpu=neon 
                    -ftree-vectorize 
                    -fomit-frame-pointer 
                    -funroll-loops
)

# Convert our list of flags to a space-delimited string
FOREACH( ITEM ${OMAP_CXX_FLAG_LIST})
    SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ITEM}" )
ENDFOREACH()


# Tell CMake where to find the target environment for FIND_xxx commands
SET(CMAKE_FIND_ROOT_PATH  /usr/local/angstrom/arm/bin/ 
                          /usr/local/angstrom/arm/arm-angstrom-linux-gnueabi
)

# Tell CMake how to search for programs in the target/system
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)

# Tell CMake how to search for libraries in the target/system
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)

# Tell CMake how to search for header files in the target/system
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

# Add some include directories specific to the OMAP compilers
include_directories( SYSTEM "/usr/local/angstrom/arm/arm-angstrom-linux-gnueabi/usr/include/" )
include_directories( SYSTEM "/usr/local/angstrom/arm/arm-angstrom-linux-gnueabi/include/c++/4.3.3" )

# Add the omap3 version of zlib
include_directories( SYSTEM "${CMAKE_CURRENT_SOURCE_DIR}/src/zlib-omap3/include" )
link_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src/zlib-omap3/lib )


