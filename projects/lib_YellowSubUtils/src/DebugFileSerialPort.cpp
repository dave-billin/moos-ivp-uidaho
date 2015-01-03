//=============================================================================
/*    Copyright (C) 2012  Dave Billin

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-----------------------------------------------------------------------------
/** @file DebugFileSerialPort.cpp
 *
 * @brief
 *  Implementation of the DebugFileSerialPort class
 *
 * @author Dave Billin
 */
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "DebugFileSerialPort.h"

using std::ios_base;
using std::string;
using std::fstream;
using std::endl;

namespace YellowSubUtils
{


//=============================================================================
DebugFileSerialPort::DebugFileSerialPort( void )
{

}


//=============================================================================
DebugFileSerialPort::~DebugFileSerialPort()
{
}




//=============================================================================
bool DebugFileSerialPort::Close( void )
{
    PortFileStream.close(); // Closing the file flushes output
    return true;
}





//=============================================================================
bool DebugFileSerialPort::Create( const char* szPortName, int nBaudRate )
{
    bool Rc = false;

    if ( szPortName != NULL )
    {
        sPortFilePath = szPortName;
        if ( !sPortFilePath.empty() )
        {
            // Close the file if it is open (shouldn't be...)
            if ( PortFileStream.is_open() )
            {
                PortFileStream.close();
            }

            // Open the file for read/write access in text mode
            PortFileStream.open( szPortName, (ios_base::in | ios_base::out) );

            Rc =  PortFileStream.is_open();
        }
    }

    return Rc;
}





//=============================================================================
int DebugFileSerialPort::Write( const char* pSerialBytes, int NumBytes,
                                double* pTime )
{
    if ( (pSerialBytes == NULL) || (NumBytes < 1) ||
         !PortFileStream.is_open() || PortFileStream.fail() )
    {
        return 0;
    }

    // Write data and check the stream's failure bits
    PortFileStream.write( pSerialBytes, NumBytes );
    if ( PortFileStream.fail() )
    {
        MOOSTrace( "<ERROR> Error writing to DebugFileSerialPort at: %s\n"
                   "%s\n", sPortFilePath.c_str() );
        return 0;
    }

    // Populate timestamp
    if ( pTime != NULL )
    {
        *pTime = MOOSTime();
    }

    return NumBytes;

}




//=============================================================================
int DebugFileSerialPort::GrabN( char* pBuffer, int NumBytesToRead )
{
    if ( (pBuffer == NULL) || (NumBytesToRead < 1) )
    {
        return 0;
    }

    PortFileStream.read( pBuffer, NumBytesToRead );
    return PortFileStream.gcount();
}


} // END namespace YellowSubUtils
