//=============================================================================
/* @file UT_ScalarPIDController.cpp
 *
 * @brief
 *   Google Test (gtest) unit tests of the YellowSubUtils::ScalarPIDController
 *   class
 */
//=============================================================================
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <gtest/gtest.h>
#include "ScalarPIDController.h"

using namespace std;
using namespace YellowSubUtils;

//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDParameters_default_ctor )
{
    ScalarPIDController::ScalarPIDParameters params;
    EXPECT_EQ( 1.0f, params.Kp() );
    EXPECT_EQ( 0, params.Ki() );
    EXPECT_EQ( 0, params.Kd() );
    EXPECT_TRUE( params.LimitsAreEnabled() );
    EXPECT_EQ( 0, params.IntegralLimit() );
    EXPECT_EQ( 0, params.OutputLimit() );
    EXPECT_EQ( ScalarPIDController::ScalarPIDParameters
                                  ::DEFAULT_AVERAGING_WINDOW_SIZE,
               params.DerivativeFilterWindowSize() );
    EXPECT_FALSE( params.OutputDeadbandIsEnabled() );
    EXPECT_EQ( 0.0f, params.OutputDeadband() );
}


//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDParameters_parameterized_ctor )
{
    ScalarPIDController::ScalarPIDParameters params(
            1.0f, 2.0f, 3.0f, false, 4.0f, 5.0f, 10, true, 6.0f );

    EXPECT_EQ( 1.0f, params.Kp() );
    EXPECT_EQ( 2.0f, params.Ki() );
    EXPECT_EQ( 3.0f, params.Kd() );
    EXPECT_FALSE( params.LimitsAreEnabled() );
    EXPECT_EQ( 4.0f, params.IntegralLimit() );
    EXPECT_EQ( 5.0f, params.OutputLimit() );
    EXPECT_EQ( 10, params.DerivativeFilterWindowSize() );
    EXPECT_TRUE( params.OutputDeadbandIsEnabled() );
    EXPECT_EQ( 6.0f, params.OutputDeadband() );
}


//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDController_default_ctor )
{
    ScalarPIDController uut;
    ScalarPIDController::ScalarPIDParameters params( uut.GetParameters() );

    // Verify default parameters
    EXPECT_EQ( 1.0f, params.Kp() );
    EXPECT_EQ( 0, params.Ki() );
    EXPECT_EQ( 0, params.Kd() );
    EXPECT_TRUE( params.LimitsAreEnabled() );
    EXPECT_EQ( 0, params.IntegralLimit() );
    EXPECT_EQ( 0, params.OutputLimit() );
    EXPECT_EQ( ScalarPIDController::ScalarPIDParameters
                                  ::DEFAULT_AVERAGING_WINDOW_SIZE,
               params.DerivativeFilterWindowSize() );

    // Verify initial state
    EXPECT_EQ( 0.0, uut.Error() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Derivative() );
    EXPECT_EQ( 0.0, uut.Output() );
    EXPECT_EQ( 0, uut.NumIterations() );
    EXPECT_FALSE( uut.LoggingIsEnabled() );
    EXPECT_TRUE( uut.GetLogDirectory().empty() );
    EXPECT_TRUE( uut.GetLogFileBaseName().empty() );

    WindowedAverage<float>& AvgFilter = uut.GetDerivativeFilter();
    DSPFilterParameters<float> FilterParams( AvgFilter.GetParameters() );
    EXPECT_EQ( 1.0f, FilterParams.PreGain );
    EXPECT_EQ( 1.0f, FilterParams.PostGain );
    EXPECT_EQ( ScalarPIDController::ScalarPIDParameters
                                  ::DEFAULT_AVERAGING_WINDOW_SIZE,
               FilterParams.Numerator.size() );
    EXPECT_TRUE( FilterParams.ProcessingIsEnabled );
}


//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDController_parameterized_ctor )
{
    ScalarPIDController::ScalarPIDParameters params( 1.0f, 2.0f, 3.0f, false,
                                                     4.0f, 5.0f, 10 );
    ScalarPIDController uut( params );

    // Verify controller parameters
    ScalarPIDController::ScalarPIDParameters& uut_params(uut.GetParameters());
    EXPECT_EQ( 1.0f, uut_params.Kp() );
    EXPECT_EQ( 2.0f, uut_params.Ki() );
    EXPECT_EQ( 3.0f, uut_params.Kd() );
    EXPECT_FALSE( uut_params.LimitsAreEnabled() );
    EXPECT_EQ( 4.0f, uut_params.IntegralLimit() );
    EXPECT_EQ( 5.0f, uut_params.OutputLimit() );
    EXPECT_EQ( 10, uut_params.DerivativeFilterWindowSize() );

    // Verify initial state
    EXPECT_EQ( 0.0, uut.Error() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Derivative() );
    EXPECT_EQ( 0.0, uut.Output() );
    EXPECT_EQ( 0, uut.NumIterations() );
    EXPECT_FALSE( uut.LoggingIsEnabled() );
    EXPECT_TRUE( uut.GetLogDirectory().empty() );
    EXPECT_TRUE( uut.GetLogFileBaseName().empty() );

    WindowedAverage<float>& AvgFilter = uut.GetDerivativeFilter();
    DSPFilterParameters<float> FilterParams( AvgFilter.GetParameters() );
    EXPECT_EQ( 1.0f, FilterParams.PreGain );
    EXPECT_EQ( 1.0f, FilterParams.PostGain );
    EXPECT_EQ( 10, FilterParams.Numerator.size() );
    EXPECT_TRUE( FilterParams.ProcessingIsEnabled );
}



//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDController_Configure )
{
    ScalarPIDController::ScalarPIDParameters params( 1.0f, 2.0f, 3.0f, false,
                                                     4.0f, 5.0f, 10 );

    // Configure a default-constructed controller with specified parameters
    ScalarPIDController uut;
    uut.Configure( params );

    // Verify controller parameters
    ScalarPIDController::ScalarPIDParameters& uut_params(uut.GetParameters());
    EXPECT_EQ( 1.0f, uut_params.Kp() );
    EXPECT_EQ( 2.0f, uut_params.Ki() );
    EXPECT_EQ( 3.0f, uut_params.Kd() );
    EXPECT_FALSE( uut_params.LimitsAreEnabled() );
    EXPECT_EQ( 4.0f, uut_params.IntegralLimit() );
    EXPECT_EQ( 5.0f, uut_params.OutputLimit() );
    EXPECT_EQ( 10, uut_params.DerivativeFilterWindowSize() );

    // Verify initial state
    EXPECT_EQ( 0.0, uut.Error() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Integral() );
    EXPECT_EQ( 0.0, uut.Derivative() );
    EXPECT_EQ( 0.0, uut.Output() );
    EXPECT_EQ( 0, uut.NumIterations() );
    EXPECT_FALSE( uut.LoggingIsEnabled() );
    EXPECT_TRUE( uut.GetLogDirectory().empty() );
    EXPECT_TRUE( uut.GetLogFileBaseName().empty() );

    WindowedAverage<float>& AvgFilter = uut.GetDerivativeFilter();
    DSPFilterParameters<float> FilterParams( AvgFilter.GetParameters() );
    EXPECT_EQ( 1.0f, FilterParams.PreGain );
    EXPECT_EQ( 1.0f, FilterParams.PostGain );
    EXPECT_EQ( 10, FilterParams.Numerator.size() );
    EXPECT_TRUE( FilterParams.ProcessingIsEnabled );
}




//=============================================================================
// Set up a PID controller with all gains set at 1.0, and integral + output
// limiting enabled and set to 3.0
//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDController_Process )
{
    const float INTEGRAL_LIMIT = 10.0f;
    const float OUTPUT_LIMIT = 10.0f;

    // Set up a PID controller whose gains are all 1 and disable averaging
    // of error applied to the derivative by using an averaging window size
    // of 1
    ScalarPIDController::ScalarPIDParameters params(
                                                1.0f, 1.0f, 1.0f, // gains
                                                true,   // Enable limiting
                                                INTEGRAL_LIMIT,
                                                OUTPUT_LIMIT,
                                                1 );    // dE/dt Window size
    ScalarPIDController uut( params );

    // Set up a time increment
    PrecisionTimeInterval dt( 1, PrecisionTimeInterval::SECONDS );

    WindowedAverage<float> Averager( 1 );   // Local averager
    float error = 0.0f;
    float expected_integral = 0.0f;
    float expected_derivative = 0.0f;

    // Test controller with positive error
    for ( uint32_t i = 1; i <= 100; i++ )
    {
        error += 1.0f;

        // Process a new setpoint
        uut.Process( dt, error );

        if ( fabs(expected_integral) < INTEGRAL_LIMIT )
        {
            expected_integral += error;
        }
        expected_derivative = 1.0f;
        float expected_output = error + expected_integral +
                                + expected_derivative;
        if ( fabs(expected_output) >= OUTPUT_LIMIT )
        {
            float sign = (expected_output < 0) ? -1.0f : 1.0f;
            expected_output = OUTPUT_LIMIT * sign;
        }

        EXPECT_EQ( 1.0f, uut.Delta_t() );
        EXPECT_EQ( expected_output, uut.Output() );
        EXPECT_EQ( error, uut.Error() );
        EXPECT_EQ( expected_integral, uut.Integral() );
        EXPECT_EQ( expected_derivative, uut.Derivative() );
        EXPECT_EQ( i, uut.NumIterations() );
    }

    uut.Reset();    // Reset the controller

    error = 0.0f;
    expected_integral = 0.0f;
    expected_derivative = 0.0f;

    // Test controller with negative error
    for ( uint32_t i = 1; i <= 3; i++ )
    {
        error -= 1.0f;

        // Process a new setpoint
        uut.Process( dt, error );

        if ( fabs(expected_integral) < INTEGRAL_LIMIT )
        {
            expected_integral += error;
        }
        expected_derivative = -1.0f;
        float expected_output = error + expected_integral +
                                + expected_derivative;
        if ( fabs(expected_output) >= OUTPUT_LIMIT )
        {
            float sign = (expected_output < 0) ? -1.0f : 1.0f;
            expected_output = OUTPUT_LIMIT * sign;
        }

        EXPECT_EQ( 1.0f, uut.Delta_t() );
        EXPECT_EQ( expected_output, uut.Output() );
        EXPECT_EQ( error, uut.Error() );
        EXPECT_EQ( expected_integral, uut.Integral() );
        EXPECT_EQ( expected_derivative, uut.Derivative() );
        EXPECT_EQ( i, uut.NumIterations() );
    }
}



//=============================================================================
// A helper function that returns the path of this executable using the Linux
// proc file system
string GetPathToSelf( void )
{
    string sPath;
    char path_buffer[1024];
    ssize_t len = ::readlink( "/proc/self/exe", path_buffer,
                              ( sizeof(path_buffer) - 1 ) );
    if ( len != -1 )
    {
        path_buffer[len] = '\0';
        sPath = path_buffer;
    }

    return sPath;
}



//=============================================================================
TEST( Test_ScalarPIDController, ScalarPIDController_ConfigureLogging )
{
    // Get the path of the executable
    string ExecutablePath = GetPathToSelf();
    string::size_type LastSlashOffset = ExecutablePath.find_last_of( "/" );
    string ExecutableDir = ExecutablePath.substr( 0, LastSlashOffset );

    ScalarPIDController controller;    // Default controller
    string BaseName( "TestLogFile" );

    ASSERT_TRUE( controller.ConfigureLogging( BaseName, ExecutableDir, true ) );
    EXPECT_TRUE( controller.LoggingIsEnabled() );
    string LogFilePath = controller.GetLogFilePath();
    EXPECT_FALSE( LogFilePath.empty() );

    // Disabled logging to close the current log file
    controller.SetLoggingEnabled(false);
    ifstream LogFileStream( LogFilePath.c_str(), ifstream::ate );
    ASSERT_FALSE( LogFileStream.fail() );
    EXPECT_GT( LogFileStream.tellg(), 0 );

    // Close and delete the test log file
    LogFileStream.close();
    EXPECT_EQ( 0, ::unlink( LogFilePath.c_str() ) );
}
