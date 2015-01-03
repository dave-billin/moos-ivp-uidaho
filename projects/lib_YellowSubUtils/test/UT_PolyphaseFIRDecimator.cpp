//=============================================================================
/** @file UT_PolyphaseFIRDecimator.cpp
 *
 * @brief
 *  Unit test for the PolyphaseFIRDecimator class
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#include <iostream>
#include <fstream>
#include <cstring>

#include "../include/YellowSubUtils.h"

using namespace std;
using namespace YellowSubUtils;

#define TYPE                float
#define NUM_TAPS            500
#define DECIMATION_FACTOR   10
#define OUTPUTBUFFERSIZE    (2*NUM_TAPS)




//=============================================================================
int main( int argc, char* argv[] )
{
    bool bSuccess = true;

    //------------------------------------------
    // Create filter coefficient array, impulse
    // and step signals, and impulse and step
    // test reference ouputs
    //------------------------------------------
    TYPE* FIRCoefficients = new TYPE[ NUM_TAPS ];
    TYPE* Impulse = new TYPE[ 2*NUM_TAPS ];
    TYPE* Step = new TYPE[ 2*NUM_TAPS ];
    TYPE* ImpulseResponseRef = new TYPE[OUTPUTBUFFERSIZE];
    TYPE* StepResponseRef = new TYPE[OUTPUTBUFFERSIZE];
    TYPE* OutputBuffer = new TYPE[OUTPUTBUFFERSIZE];

    // Initialize filter coefficients
    for (int i = 0; i < NUM_TAPS; i++)
    {
        FIRCoefficients[i] = i + 1;
    }

    // Set up filter parameters
    DSPFilterParameters<TYPE> Params(true,       /* enable processing */
                                     1,          /* Pre-filter gain */
                                     1,          /* Post-filter gain */
                                     FIRCoefficients, /* Numerator coeffs */
                                     NUM_TAPS,   /* # Numerator coeffs */
                                     NULL,       /* Denominator coefficients */
                                     0);         /* # Denom coefficients */

    // Calculate (non-decimated) reference outputs using an equivalent filter
    FIRFilter<TYPE> ReFIR(Params);

    ReFIR.ProcessSamples(Impulse, OutputBuffer, OUTPUTBUFFERSIZE);
    // Decimate the reference output
    int j = 0;
    for (int i = DECIMATION_FACTOR-1; i < OUTPUTBUFFERSIZE;
             i += DECIMATION_FACTOR)
    {
        ImpulseResponseRef[j++] = OutputBuffer[i];
    }

    ReFIR.Reset();
    ReFIR.ProcessSamples(Step, OutputBuffer, OUTPUTBUFFERSIZE);
    // Decimate the reference output
    for (int i = DECIMATION_FACTOR-1; i < OUTPUTBUFFERSIZE;
             i += DECIMATION_FACTOR)
    {
        StepResponseRef[j++] = OutputBuffer[i];
    }

    //------------------------------------
    // Test the PolyphaseFIRDecimator
    //------------------------------------
    string sBar = string(78, '=') + '\n';
    cout << sBar
         << "<< PolyphaseFIRDecimator class unit test >>\n"
         << "Testing a " << NUM_TAPS << " filter\n"
         << "Decimation factor = " << DECIMATION_FACTOR << "\n"
         << sBar << endl;


    //----------------------------
    // Validate impulse response
    //----------------------------
    cout << "Validating impulse response...";

    // Create the filter from parameters
    PolyphaseFIRDecimator<TYPE> UUT(Params, DECIMATION_FACTOR);
    UUT.ProcessSamples(Impulse, OutputBuffer, OUTPUTBUFFERSIZE);
    if ( memcmp( OutputBuffer, ImpulseResponseRef,
                 (OUTPUTBUFFERSIZE / DECIMATION_FACTOR)*sizeof(TYPE) ) )
    {
        cout << "FAILED!" << endl;

        cout << "Reference:  ";
        for (int i = 0; i < OUTPUTBUFFERSIZE / DECIMATION_FACTOR; i++)
        {
            cout << ImpulseResponseRef[i] << ' ';
        }
        cout << "\n" << endl;


        cout << "UUT Output: ";
        for (int i = 0; i < OUTPUTBUFFERSIZE / DECIMATION_FACTOR; i++)
        {
            cout << OutputBuffer[i] << ' ';
        }
        cout << "\n" << endl;

        bSuccess = false;
    }
    else
    {
        cout << "PASSED!" << endl;
    }


    //----------------------------
    // Validate step response
    //----------------------------
    if (bSuccess)
    {
        cout << "Validating step response...";
        UUT.Reset();    // Reset the filter
        UUT.ProcessSamples(Step, OutputBuffer, OUTPUTBUFFERSIZE);
        if ( memcmp( OutputBuffer, StepResponseRef,
                     (OUTPUTBUFFERSIZE / DECIMATION_FACTOR)*sizeof(TYPE) ) )
        {
            cout << "FAILED!" << endl;

            cout << "Reference:  ";
            for (int i = 0; i < OUTPUTBUFFERSIZE / DECIMATION_FACTOR; i++)
            {
                cout << StepResponseRef[i] << ' ';
            }
            cout << "\n" << endl;


            cout << "UUT Output: ";
            for (int i = 0; i < OUTPUTBUFFERSIZE / DECIMATION_FACTOR; i++)
            {
                cout << OutputBuffer[i] << ' ';
            }
            cout << "\n" << endl;

            return -2;
        }
        else
        {
            cout << "PASSED!" << endl;
        }
    }

    if (bSuccess)
    {
        cout << "\nAll PolyphaseFIRDecimator unit tests PASSED" << endl;
    }

    delete[] FIRCoefficients;
    delete[] Impulse;
    delete[] Step;
    delete[] ImpulseResponseRef;
    delete[] StepResponseRef;
    delete[] OutputBuffer;

    return 0;
}
