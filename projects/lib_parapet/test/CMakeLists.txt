#==============================================================================
# CMake configuration File for lib_parapet unit tests
#
# Created January 2013 by Dave Billin
#==============================================================================

#===============================
# Source Files
#===============================

# Create a list of source files
set( UNIT_TEST_SOURCE_FILES
     test_parapet_descriptor.cpp
     test_parapet_header.cpp
     test_parapet_node.cpp
     test_parapet.cpp
   )


include_directories( ${INCLUDE_DIR} 
                     ${PARAPET_INCLUDE_DIRS} 
                   )


enable_testing()
include_directories( ${GTEST_INCLUDE_DIRS} )

#============================
# Specify the project target
#============================

# Define the static library target this project will build
add_executable( gtest_${PROJECT_NAME} ${UNIT_TEST_SOURCE_FILES} )
                       
# Specify linkage order
target_link_libraries( gtest_${PROJECT_NAME}
                       parapet
                       ${GTEST_BOTH_LIBRARIES}
                       ${PLATFORM_LIBRARIES}
                     )


#============================
# Add an install rule
#============================
# Install target
INSTALL( TARGETS gtest_${PROJECT_NAME}
	RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
	)


