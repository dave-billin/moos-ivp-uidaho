#==============================================================================
# Newmat matrix library CMake Configuration
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project( Newmat10 )

# Set up convenience variables for project subdirectories
set( src_dir ${PROJECT_SOURCE_DIR}/src )
set( include_dir ${PROJECT_SOURCE_DIR}/include )
set( config_dir ${PROJECT_SOURCE_DIR}/config )
set( test_dir ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files
set( HEADER_FILES
    ${include_dir}/boolean.h
    ${include_dir}/controlw.h
    ${include_dir}/include.h
    ${include_dir}/myexcept.h
    ${include_dir}/newmat.h
    ${include_dir}/newmatap.h
    ${include_dir}/newmatio.h
    ${include_dir}/newmatnl.h
    ${include_dir}/newmatrc.h
    ${include_dir}/newmatrm.h
    ${include_dir}/precisio.h
    ${include_dir}/solution.h
)

# Create a list of source files
set( SOURCE_FILES
    ${src_dir}/bandmat.cpp
    ${src_dir}/cholesky.cpp
    ${src_dir}/evalue.cpp
    ${src_dir}/example.cpp
    ${src_dir}/fft.cpp
    ${src_dir}/hholder.cpp
    ${src_dir}/jacobi.cpp
    ${src_dir}/myexcept.cpp
    ${src_dir}/newfft.cpp
    ${src_dir}/newmat1.cpp
    ${src_dir}/newmat2.cpp
    ${src_dir}/newmat3.cpp
    ${src_dir}/newmat4.cpp
    ${src_dir}/newmat5.cpp
    ${src_dir}/newmat6.cpp
    ${src_dir}/newmat7.cpp
    ${src_dir}/newmat8.cpp
    ${src_dir}/newmat9.cpp
    ${src_dir}/newmatex.cpp
    ${src_dir}/newmatnl.cpp
    ${src_dir}/newmatrm.cpp
    ${src_dir}/solution.cpp
    ${src_dir}/sort.cpp
    ${src_dir}/submat.cpp
    ${src_dir}/svd.cpp
)






#=====================================
# Set matrix element data type
#=====================================
# Set the precision of elements in Newmat matrices
#   - If building for Gumstix, use 32-bit (single-precision) floats
#   - If building for desktop, use 64-bit (double-precision) floats 
option( NEWMAT_USE_SINGLE_PRECISION
        "Use single-precision floating-point for matrix elements (always true when building for Gumstix)" 
        ${BUILD_FOR_GUMSTIX} 
)


if ( (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "omap3") OR NEWMAT_USE_SINGLE_PRECISION)
    set(MATRIX_TYPEDEF "USING_FLOAT")
    message(STATUS "NEWMAT: Using single-precision for matrix elements")
else()
    set(MATRIX_TYPEDEF "USING_DOUBLE")
    message(STATUS "NEWMAT: Using double-precision for matrix elements")
endif()

set( generated_include_h ${include_dir}/include.h )
configure_file( ${config_dir}/include.h.in 
                ${generated_include_h} )


#============================
# Specify the project target
#============================

include_directories( ${include_dir} )

# Define the static library target this project will build
add_library( Newmat10 SHARED ${SOURCE_FILES} ${generated_include_h} )

# Specify linkage order
target_link_libraries( Newmat10 )

# Pass the target's header file directories one level up in the CMake tree
set( LIBNEWMAT_INCLUDE_DIRS ${include_dir} PARENT_SCOPE )
set( LIBNEWMAT_LIBRARIES Newmat10 PARENT_SCOPE )


#============================
# Add a rule to install
#============================
# Install headers
INSTALL( FILES ${HEADER_FILES}  
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/Newmat10 )

# Install target
install( TARGETS Newmat10
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


