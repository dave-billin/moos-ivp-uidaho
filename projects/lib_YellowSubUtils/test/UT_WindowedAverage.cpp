//=============================================================================
/* @file UT_WindowedAverage.cpp
 *
 * @brief
 *   Google Test (gtest) unit tests of the YellowSubUtils::WindowedAverage
 *   class
 */
//=============================================================================
#include <stdint.h>

#include <gtest/gtest.h>
#include "DSPFilterParameters.h"
#include "WindowedAverage.h"

using namespace std;
using namespace YellowSubUtils;


//=============================================================================
/** @class WindowedAverage_TestFixture
 *  @brief
 *     A Google Test (gtest) test fixture used for unit testing the
 *     WindowedAverage class
 */
//=============================================================================
class WindowedAverage_TestFixture : public ::testing::Test
{
protected:

    enum { WINDOW_SIZE = 10 };  // Number of samples to average in tests

    /** Constructor */
    WindowedAverage_TestFixture()
      : uut( WINDOW_SIZE, WINDOW_SIZE, 1 )
    {}

    /** Destructor */
    virtual ~WindowedAverage_TestFixture()
    {}


    //=========================================================================
    /** Called after constructor runs and just before a test begins */
    virtual void SetUp( void )
    {
    }

    //=========================================================================
    /** Called after a test finishes */
    virtual void TearDown( void )
    {
    }

    // Note:
    //  Tests are performed using an integral data type to
    //  avoid rounding errors during validation
    WindowedAverage<int> uut;
};


//=============================================================================
// Tests the WindowedAverage default constructor
//=============================================================================
TEST( Test_WindowedAverage, test_default_constructor )
{
    // Verify configuration
    WindowedAverage<int> uut_default;
    DSPFilterParameters<int> ExpectedParams = uut_default.GetParameters();

    EXPECT_EQ( 1, ExpectedParams.PreGain );
    EXPECT_EQ( 1, ExpectedParams.PostGain );
    EXPECT_TRUE( ExpectedParams.ProcessingIsEnabled );
    EXPECT_EQ( 1, ExpectedParams.Numerator.size() );
    EXPECT_EQ( 0, ExpectedParams.Denominator.size() );
}


//=============================================================================
// Tests the WindowedAverage parameterized constructor
//=============================================================================
TEST( Test_WindowedAverage, test_parameterized_constructor )
{
    WindowedAverage<float> uut( 10, 2.0f, 3.0f );

    // Verify configuration
    DSPFilterParameters<float> ExpectedParams = uut.GetParameters();

    EXPECT_EQ( 2.0f, ExpectedParams.PreGain );
    EXPECT_EQ( 3.0f, ExpectedParams.PostGain );
    EXPECT_TRUE( ExpectedParams.ProcessingIsEnabled );
    EXPECT_EQ( 10, ExpectedParams.Numerator.size() );
    EXPECT_EQ( 0, ExpectedParams.Denominator.size() );

    // Pre-gain of the filter is the same as the window size, so individual
    // coefficients should equal 1
    for ( vector<float>::iterator coeff = ExpectedParams.Numerator.begin();
          coeff != ExpectedParams.Numerator.end();
          coeff++ )
    {
        EXPECT_EQ( 1.0f / 10.0f, *coeff );
    }
}


//=============================================================================
// Tests WindowedAverage sample processing
//=============================================================================
TEST_F( WindowedAverage_TestFixture, test_ProcessSamples )
{
    int accum = 0;
    int output = 0;
    int expected_average = 0;

    // Make pre-gain equal to the window size.  This way, applying an
    // input sample value of 1 should cause the output to increase
    // by 1
    for ( int i = 0; i < 10; i++ )
    {
        accum += 10;
        expected_average = accum / 10;
        output = uut.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
        EXPECT_EQ( uut.Output(), output );
    }

    // Verify that the average persists with a constant input
    for ( int i = 0; i < 10; i++ )
    {
        output = uut.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
    }
}


//=============================================================================
// Tests sample processing with pre-gain
//=============================================================================
TEST( Test_WindowedAverage, test_ProcessSamples_PreGain )
{
    // We're just testing pre-gain, so set averaging
    // window size to 1 to bypass averaging
    // Set pre-gain to 10 and post-gain to 1.  This
    // way, applying an input sample value of 1 should
    // cause the output to increase by 10's
    WindowedAverage<int> uut( 1, 10 );

    int accum = 0;
    int output = 0;

    for ( int i = 0; i < 10; i++ )
    {
        accum += 10;
        output = uut.ProcessSample( accum / 10 );
        EXPECT_EQ( accum, output );
        EXPECT_EQ( accum, uut.Output() );
    }
}



//=============================================================================
// Tests sample processing with post-gain
//=============================================================================
TEST( Test_WindowedAverage, test_ProcessSamples_PostGain )
{
    // We're just testing pre-gain, so set averaging
    // window size to 1 to bypass averaging
    // Set pre-gain to 1 and post-gain to 10.  This
    // way, applying an input sample value of 1 should
    // cause the output to increase by 10's
    WindowedAverage<int> uut( 1, 1, 10 );

    int accum = 0;
    int output = 0;

    for ( int i = 0; i < 10; i++ )
    {
        accum += 10;
        output = uut.ProcessSample( accum / 10 );
        EXPECT_EQ( accum, output );
        EXPECT_EQ( accum, uut.Output() );
    }
}



//=============================================================================
// Verifies sample processing of a WindowedAverage populated using copy
// constructor
//=============================================================================
TEST_F( WindowedAverage_TestFixture, test_CopyConstructor )
{
    WindowedAverage<int> clone( uut );
    int accum = 0;
    int output = 0;
    int expected_average = 0;

    // Pre-gain is 10, and 'coefficients' are 1/10, so applying an
    // input sample value of 1 should cause the output to increase
    // by 1
    for ( int i = 0; i < 10; i++ )
    {
        accum += 10;
        expected_average = accum / 10;
        output = clone.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
        EXPECT_EQ( clone.Output(), output );
    }

    // Verify that the average persists with a constant input
    for ( int i = 0; i < 10; i++ )
    {
        output = clone.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
    }
}



//=============================================================================
// Verifies sample processing of a WindowedAverage populated using assignment
// operator
//=============================================================================
TEST_F( WindowedAverage_TestFixture, test_AssignmentOperator )
{
    WindowedAverage<int> clone = uut;
    int accum = 0;
    int output = 0;
    int expected_average = 0;

    // Pre-gain is 10, and 'coefficients' are 1/10, so applying an
    // input sample value of 1 should cause the output to increase
    // by 1
    for ( int i = 0; i < 10; i++ )
    {
        accum += 10;
        expected_average = accum / 10;
        output = clone.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
        EXPECT_EQ( clone.Output(), output );
    }

    // Verify that the average persists with a constant input
    for ( int i = 0; i < 10; i++ )
    {
        output = clone.ProcessSample( 1 );
        EXPECT_EQ( expected_average, output );
    }
}

