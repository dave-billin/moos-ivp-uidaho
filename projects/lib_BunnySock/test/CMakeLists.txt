#==============================================================================
# CMake configuration File for libBunnySock unit tests
#
# Created January 2016 by Dave Billin
#==============================================================================

#===============================
# Source Files
#===============================

# Create a list of source files
set( UNIT_TEST_SOURCE_FILES
     test_address_info.cpp
   )

include_directories( ${LIBBUNNYSOCK_INCLUDE_DIRS}
                     ${src_dir} 
                   )

enable_testing()
include_directories( ${GTEST_INCLUDE_DIRS} )

#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_executable( test_libbunnysock ${UNIT_TEST_SOURCE_FILES} )
                       
# Specify linkage order
target_link_libraries( test_libbunnysock
                       BunnySock
                       ${GTEST_BOTH_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )


#============================
# Add an install rule
#============================
# Install target
INSTALL( TARGETS test_libbunnysock
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	)


