//=============================================================================
/*  Copyright (C) 2013  Dave Billin

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
/** @file MockSerialPort.cpp
 *
 * @brief
 *   A short description of the file
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <cassert>
#include <cstring>      // Needed for memcpy
#include <algorithm>
#include "MockSerialPort.h"

namespace YellowSubUtils
{

const char* MockSerialPort::s_DEFAULT_DEVICE = "MockSerialPort";
const int MockSerialPort::s_DEFAULT_BAUDRATE = 0;


//=============================================================================
MockSerialPort::MockSerialPort()
  : m_IsOpen( false )
{
}


//=============================================================================
MockSerialPort::~MockSerialPort()
{
}


//=============================================================================
bool MockSerialPort::Create(const char* SerialDeviceName, int BaudRate)
{
    assert( m_IsOpen == false );
    assert( SerialDeviceName != NULL );

    m_sPort = SerialDeviceName;
    m_nBaudRate = BaudRate;
    m_IsOpen = true;

    // Clear the Tx and Rx buffers
    m_TxBuffer.clear();
    m_RxBuffer.clear();
    return true;
}


//=============================================================================
bool MockSerialPort::Close()
{
    m_IsOpen = false;
    m_sPort.clear();
    m_nBaudRate = 0;
    m_TxBuffer.clear();
    m_RxBuffer.clear();
    return true;
}


//=============================================================================
int MockSerialPort::Write(const char* TxData, int NumBytes,
        double* TxTimeStamp)
{
    assert( TxData != NULL );
    m_TxBuffer.append( TxData, NumBytes );
    return NumBytes;
}


//=============================================================================
void MockSerialPort::AddRxData(const char* RxData, int NumBytes)
{
    assert( RxData != NULL );
    m_RxBuffer.append( RxData, NumBytes );
}


//=============================================================================
int MockSerialPort::ReadDataFromString(std::string& SourceString,
        char* TargetBuffer, int NumBytes)
{
    assert( TargetBuffer != NULL );
    NumBytes = std::min( static_cast<std::string::size_type>( NumBytes ),
                         SourceString.size() );

    if ( NumBytes > 0 )
    {
        memcpy( TargetBuffer, SourceString.c_str(), NumBytes );
        SourceString.erase( 0, NumBytes );
    }

    return NumBytes;
}

} /* namespace YellowSubUtils */
