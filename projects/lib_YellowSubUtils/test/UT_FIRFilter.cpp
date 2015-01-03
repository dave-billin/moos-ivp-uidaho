//=============================================================================
/** @file UT_FIRFilter.cpp
 *
 * @brief
 *	Unit test for the FIRFilter class
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


#define TYPE        float

enum { NUM_TAPS = 500 };


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
    TYPE* ImpulseResponseRef = new TYPE[ 2*NUM_TAPS ];
    TYPE* StepResponseRef = new TYPE[ 2*NUM_TAPS ];
    TYPE* OutputBuffer = new TYPE[ 2*NUM_TAPS ];
    TYPE Accum = 0;
    for (int i = 0; i < NUM_TAPS; i++)
    {
        FIRCoefficients[i] = i+1;
        Impulse[i] = 0;
        Step[i] = 1;

        if (i < NUM_TAPS)
        {
            ImpulseResponseRef[i] = FIRCoefficients[i];
            Accum += FIRCoefficients[i];
        }
        else
        {
            ImpulseResponseRef[i] = 0;
        }
        StepResponseRef[i] = Accum;
    }
    Impulse[0] = 1;



    string sBar = string(78, '=') + '\n';
    cout << sBar <<
         "<< FIRFilter class unit test >>\n"
         << sBar;

    cout << "Validating impulse response of a " << NUM_TAPS
         << "-tap FIR filter" << endl;
    DSPFilterParameters<TYPE> Params(true,       /* enable processing */
                                     1,          /* Pre-filter gain */
                                     1,          /* Post-filter gain */
                                     FIRCoefficients, /* Numerator coeffs */
                                     NUM_TAPS,   /* # Numerator coeffs */
                                     NULL,       /* Denominator coefficients */
                                     0);         /* # Denom coefficients */


    //----------------------------
    // Validate impulse response
    //----------------------------
    FIRFilter<TYPE> UUT(Params);  // Create a FIR filter from parameters
    UUT.ProcessSamples(Impulse, OutputBuffer, 2*NUM_TAPS);
    if ( memcmp(OutputBuffer, ImpulseResponseRef, 2*NUM_TAPS) )
    {
        cout << "Impulse response test FAILED!" << endl;

        cout << "Reference:  ";
        for (int i = 0; i < 2*NUM_TAPS; i++)
        {
            cout << ImpulseResponseRef[i] << ' ';
        }
        cout << "\n" << endl;


        cout << "UUT Output: ";
        for (int i = 0; i < 2*NUM_TAPS; i++)
        {
            cout << OutputBuffer[i] << ' ';
        }
        cout << "\n" << endl;

        bSuccess = false;
    }


    //----------------------------
    // Validate step response
    //----------------------------
    if (bSuccess)
    {
        cout << "Validating step response of a " << NUM_TAPS
             << "-tap FIR filter" << endl;
        UUT.Reset();    // Reset the filter
        UUT.ProcessSamples(Step, OutputBuffer, 2*NUM_TAPS);
        if ( memcmp(OutputBuffer, StepResponseRef, 2*NUM_TAPS) )
        {
            cout << "Step response test FAILED!" << endl;

            cout << "Reference:  ";
            for (int i = 0; i < 2*NUM_TAPS; i++)
            {
                cout << StepResponseRef[i] << ' ';
            }
            cout << "\n" << endl;


            cout << "UUT Output: ";
            for (int i = 0; i < 2*NUM_TAPS; i++)
            {
                cout << OutputBuffer[i] << ' ';
            }
            cout << "\n" << endl;

            return -2;
        }
    }

    if (bSuccess)
    {
        cout << "\nAll FIRFilter unit tests PASSED" << endl;
    }

    delete[] FIRCoefficients;
    delete[] Impulse;
    delete[] Step;
    delete[] ImpulseResponseRef;
    delete[] StepResponseRef;
    delete[] OutputBuffer;

    return 0;
}
