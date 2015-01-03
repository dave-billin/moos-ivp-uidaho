//=============================================================================
/** @file UT_DebugFileSerialPort.cpp
 *
 * @brief
 *	Unit test for the DebugFileSerialPort class
 *
 * @author Dave Billin
 */
//=============================================================================

#include <iostream>
#include <fstream>

#include "../include/DebugFileSerialPort.h"

using namespace std;

using namespace YellowSubUtils;


#define NUM_TEST_BYTES 256     // Number of Bytes read and written in testing


//=============================================================================
int main( int argc, char* argv[] )
{
    cout << "<< Testing DebugFileSerialPort >>\n" << endl;

    if (argc < 2)
    {
        cout << "ERROR: Missing required file path argument\n"
                "       USAGE: UT_DebugFileSerialPort <FILE>\n" << endl;
        return -1;
    }

    // Create the test file if necessary
    ofstream VirtualPortFile(argv[1]);
    if ( VirtualPortFile.is_open() )
    {
        VirtualPortFile.close();
    }
    else
    {
        cout << "Failed to create virtual serial port file: " << argv[1]
             << "\n" << endl;
        return -1;
    }

    DebugFileSerialPort WritePort;
    if ( !WritePort.Create( argv[1], 0) )
    {
        cout << "Failed to create write port using file: " << argv[1] << "\n"
             << endl;
        return -2;
    }

    // Initialize an array of test Bytes to read and write
    char pWriteBytes[NUM_TEST_BYTES];
    for (int i = 0; i < NUM_TEST_BYTES; i++)
    {
        pWriteBytes[i] = i;
    }

    // Write test Bytes to the virtual serial port
    double t = 0.0;
    int NumBytesWritten = WritePort.Write(pWriteBytes, NUM_TEST_BYTES, &t);

    if ( NumBytesWritten != NUM_TEST_BYTES )
    {
        cout << "ERROR: DebugFileSerialPort::Write() failed\n" << endl;
        return -3;
    }

    // Verify that the time stamp was populated
    if (t == 0.0)
    {
        cout << "ERROR: DebugFileSerialPort::Write() missed timestamp\n"
             << endl;
        return -4;
    }

    // Close the write port used for writing.
    // This should flush all written data
    if ( !WritePort.Close() )
    {
        cout << "ERROR: DebugFileSerialPort::Close() failed\n" << endl;
        return -5;
    }

    // Open a new virtual port for testing reads
    DebugFileSerialPort ReadPort;
    if ( !ReadPort.Create( argv[1], 0) )
    {
        cout << "Failed to create read port using file: " << argv[1] << "\n"
             << endl;
        return -6;
    }

    // Read back Bytes that the Write port sent to our file
    char pReadBytes[NUM_TEST_BYTES];
    int NumBytesRead = ReadPort.ReadNWithTimeOut( pReadBytes, NUM_TEST_BYTES );

    if ( NumBytesRead != NUM_TEST_BYTES )
    {
        cout << "ERROR: DebugFileSerialPort::GrabN() failed using call to"
                "CMOOSSerialPort::ReadNWithTimeOut()\n" << endl;
        return -7;
    }

    // Verify that read data matches written data
    for (int i = 0; i < NUM_TEST_BYTES; i++)
    {
        if ( pReadBytes[i] != pWriteBytes[i] )
        {
            cout << "ERROR: written and read data disagree at file offset of"
                    << i << "Bytes\n" << endl;
            return -8;
        }
    }

    return 0;   // All tests pass
}
