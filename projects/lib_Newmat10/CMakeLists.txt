#==============================================================================
# Newmat matrix library CMake Configuration
#
# Created 8-2011 by Dave Billin
#==============================================================================

#===============================
# Project Settings
#===============================
project( Newmat10 )

set( ${PROJECT_NAME}_MAJOR_VERSION 0 )
set( ${PROJECT_NAME}_MINOR_VERSION 1 )
set( ${PROJECT_NAME}_VERSION
        ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION} )


# Set up convenience variables for project subdirectories
set( SRC_DIR ${PROJECT_SOURCE_DIR}/src )
set( INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )
set( BUILD_DIR ${PROJECT_SOURCE_DIR}/build )
set( TEST_DIR ${PROJECT_SOURCE_DIR}/test )


#===============================
# Source Files
#===============================

# Create a list of header files
set( HEADER_FILES
    ${INCLUDE_DIR}/boolean.h            
    ${INCLUDE_DIR}/controlw.h            
    ${INCLUDE_DIR}/include.h
    ${INCLUDE_DIR}/myexcept.h
    ${INCLUDE_DIR}/newmat.h
    ${INCLUDE_DIR}/newmatap.h
    ${INCLUDE_DIR}/newmatio.h
    ${INCLUDE_DIR}/newmatnl.h
    ${INCLUDE_DIR}/newmatrc.h
    ${INCLUDE_DIR}/newmatrm.h
    ${INCLUDE_DIR}/precisio.h
    ${INCLUDE_DIR}/solution.h
)

# Create a list of source files
set( SOURCE_FILES
    ${SRC_DIR}/bandmat.cpp       
    ${SRC_DIR}/cholesky.cpp  
    ${SRC_DIR}/evalue.cpp    
    ${SRC_DIR}/example.cpp     
    ${SRC_DIR}/fft.cpp           
    ${SRC_DIR}/hholder.cpp     
    ${SRC_DIR}/jacobi.cpp
    ${SRC_DIR}/myexcept.cpp
    ${SRC_DIR}/newfft.cpp
    ${SRC_DIR}/newmat1.cpp
    ${SRC_DIR}/newmat2.cpp
    ${SRC_DIR}/newmat3.cpp
    ${SRC_DIR}/newmat4.cpp
    ${SRC_DIR}/newmat5.cpp
    ${SRC_DIR}/newmat6.cpp
    ${SRC_DIR}/newmat7.cpp
    ${SRC_DIR}/newmat8.cpp
    ${SRC_DIR}/newmat9.cpp
    ${SRC_DIR}/newmatex.cpp
    ${SRC_DIR}/newmatnl.cpp
    ${SRC_DIR}/newmatrm.cpp
    ${SRC_DIR}/solution.cpp
    ${SRC_DIR}/sort.cpp
    ${SRC_DIR}/submat.cpp
    ${SRC_DIR}/svd.cpp
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
configure_file( ${BUILD_DIR}/include.h.in 
                ${INCLUDE_DIR}/include.h )


#============================
# Specify the project target
#============================

include_directories( ${INCLUDE_DIR} )

# Define the static library target this project will build
add_library( ${PROJECT_NAME} STATIC ${SOURCE_FILES} )

# Specify linkage order
target_link_libraries( ${PROJECT_NAME} )

# Pass the target's header file directories one level up in the CMake tree
string( TOUPPER ${PROJECT_NAME} PROJECT_NAME_UCASE )
set( ${PROJECT_NAME_UCASE}_INCLUDE_DIRECTORIES 
     ${INCLUDE_DIR} 
     PARENT_SCOPE 
)



#============================
# Add a rule to install
#============================
# Install headers
INSTALL( FILES ${HEADER_FILES}  
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME} )

# Install target
install( TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)


