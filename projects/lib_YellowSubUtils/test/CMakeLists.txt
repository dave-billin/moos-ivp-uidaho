#==============================================================================
# lib_YellowSubUtils Unit Tests
#==============================================================================

include_directories( ${MOOS_INCLUDE_DIRS} )

#===============================================
# Unit test executables and test tasks
#===============================================


#================================
# DebugFileSerialPort class
#================================

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

# Offer a GUI option to build unit tests
set( UNITTEST_DebugFileSerialPort_ENABLED ON CACHE BOOL 
     "Build DebugFileSerialPort unit test" )

if ( UNITTEST_DebugFileSerialPort_ENABLED )
    add_executable( UT_DebugFileSerialPort UT_DebugFileSerialPort.cpp )
    target_link_libraries( UT_DebugFileSerialPort
    					   YellowSubUtils
    					   ${MOOS_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
    					 )

    # Add a CTest task
    ADD_TEST( NAME CTEST_DebugFileSerialPort 
              COMMAND UT_DebugFileSerialPort 
              ${CMAKE_CURRENT_BINARY_DIR}/VirtualSerialPortFile
            )
endif()




##================================
## MockSerialPort class
##================================
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
#
## Offer a GUI option to build unit tests
#set( UNITTEST_MockSerialPort_ENABLED ON CACHE BOOL 
#     "Build MockSerialPort unit test" )
#
#if ( UNITTEST_MockSerialPort_ENABLED )
#
#    find_package( GTest REQUIRED )
#    include_directories( ${GTEST_INCLUDE_DIRS} )
#    
#    add_executable( gtest_MockSerialPort UT_MockSerialPort.cpp )
#    target_link_libraries( gtest_MockSerialPort
#                           YellowSubUtils
#                           ${GTEST_BOTH_LIBRARIES}
#                           ${MOOS_LIBRARIES}
#                           ${PLATFORM_LIBRARIES}
#                         )
#
#    # Add a CTest task
#    ADD_TEST( NAME CTEST_MockSerialPort 
#              COMMAND gtest_MockSerialPort 
#            )
#endif()





#================================
# FIRFilter class
#================================
# Offer a GUI option to build unit tests
set( UNITTEST_FIRFilter_ENABLED ON CACHE BOOL 
     "Build FIRFilter unit test" )
     
if( UNITTEST_FIRFilter_ENABLED )
    
    add_executable( UT_FIRFilter UT_FIRFilter.cpp )
    target_link_libraries( UT_FIRFilter
    					   YellowSubUtils
    					   ${MOOS_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
    					 )
    
    # Add a CTest task
    ADD_TEST( NAME CTEST_FIRFilter 
    		  COMMAND UT_FIRFilter
    		)
endif()


#================================
# PolyphaseFIRDecimator class
#================================

# Offer a GUI option to build unit tests
set( UNITTEST_PolyphaseFIRDecimator_ENABLED ON CACHE BOOL 
     "Build PolyphaseFIRDecimator unit test" )
     
if( UNITTEST_PolyphaseFIRDecimator_ENABLED )
    
    add_executable( UT_PolyphaseFIRDecimator UT_PolyphaseFIRDecimator.cpp )
    target_link_libraries( UT_PolyphaseFIRDecimator
    					   YellowSubUtils
    					   ${MOOS_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
    					 )
    
    # Add a CTest task
    ADD_TEST( NAME CTEST_PolyphaseFIRDecimator 
    		  COMMAND UT_PolyphaseFIRDecimator
    		)
endif()



#================================
# WindowedAverage class
#================================

# Offer a GUI option to build the unit test
set( UNITTEST_WindowedAverage_ENABLED ON CACHE BOOL 
     "Build WindowedAverage unit test" )
     
if( UNITTEST_WindowedAverage_ENABLED )
    
    find_package( GTest REQUIRED )
    include_directories( ${GTEST_INCLUDE_DIRS} )
    
    add_executable( gtest_WindowedAverage UT_WindowedAverage.cpp )
    target_link_libraries( gtest_WindowedAverage
                           YellowSubUtils
                           ${GTEST_BOTH_LIBRARIES}
                           ${MOOS_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
                         )
    
    # Add a CTest task
    ADD_TEST( NAME CTEST_WindowedAverage 
              COMMAND gtest_WindowedAverage
            )
endif()




#================================
# ScalarPIDController class
#================================

# Offer a GUI option to build the unit test
set( UNITTEST_ScalarPIDController_ENABLED ON CACHE BOOL 
     "Build ScalarPIDController unit test" )
     
if( UNITTEST_ScalarPIDController_ENABLED )
    
    find_package( GTest REQUIRED )
    include_directories( ${GTEST_INCLUDE_DIRS} )
    
    add_executable( gtest_ScalarPIDController UT_ScalarPIDController.cpp )
    target_link_libraries( gtest_ScalarPIDController
                           YellowSubUtils
                           ${GTEST_BOTH_LIBRARIES}
                           ${MOOS_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
                         )
    
    # Add a CTest task
    ADD_TEST( NAME CTEST_ScalarPIDController 
              COMMAND gtest_ScalarPIDController
            )
endif()


#================================
# StringUtilities class
#================================

# Offer a GUI option to build the unit test
set( UNITTEST_StringUtilities_ENABLED ON CACHE BOOL 
     "Build StringUtilities unit test" )
     
if( UNITTEST_StringUtilities_ENABLED )
    
    find_package( GTest REQUIRED )
    include_directories( ${GTEST_INCLUDE_DIRS} )
    
    add_executable( gtest_StringUtilities UT_StringUtilities.cpp )
    target_link_libraries( gtest_StringUtilities
                           YellowSubUtils
                           ${GTEST_BOTH_LIBRARIES}
                           ${PLATFORM_LIBRARIES}
                         )
    
    # Add a CTest task
    ADD_TEST( NAME CTEST_StringUtilities 
              COMMAND gtest_StringUtilities
            )
endif()