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
/** @file ParapetSerialMaster.cpp
 *
 * @brief
 *   Implementation of the ParapetSerialMaster class
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <stdint.h>
#include <cassert>
#include <string>

#include "PrecisionTime.h"
#include "ParapetSerialDeviceNode.h"

using namespace std;
using namespace parapet;
using namespace BSDuC_parapet_packets;

//=============================================================================
ParapetSerialDeviceNode::ParapetSerialDeviceNode( uint8_t DeviceID )
  :  ParapetDeviceNode( DeviceID ),
     m_SerialPortIsOpen( false ),
     m_SequenceID( 0 )
{
}

//=============================================================================
ParapetSerialDeviceNode::~ParapetSerialDeviceNode()
{
}


//=============================================================================
int8_t ParapetSerialDeviceNode::Transmit( parapet::parapet_packet_t& Packet )
{
    int8_t TransmittedSequenceID = -1;

    if ( m_SerialPortIsOpen )
    {
        // Populate SourceID, DestID, and SequenceID in the header
        parapet_header_t& Header = Packet.Header;
        Header.u32 &= ~( HEADER_MASK_SOURCE_ID |
                                HEADER_MASK_DEST_ID |
                                HEADER_MASK_SEQUENCE_ID );
        Header.u32 |=
                ( ( (m_Node.DeviceID & 0x0f) << HEADER_OFFSET_SOURCE_ID) |
                  ( BSDuC_parapet_packets::BSDuC_PARAPET_DEVICE_ID
                                          << HEADER_OFFSET_DEST_ID ) |
                  ( m_SequenceID << HEADER_OFFSET_SEQUENCE_ID ) );

        // Write packet data to the object's serial port
        int NumPacketBytes = parapet_PacketSize( &Packet );
        int NumBytesWritten = m_SerialPort.Write(
                                        reinterpret_cast<char*>( &Packet.u8 ),
                                        NumPacketBytes );

        if ( NumBytesWritten == NumPacketBytes )
        {
            TransmittedSequenceID = static_cast<int8_t>( m_SequenceID );
            // Increment and mod sequence ID
            m_SequenceID = (m_SequenceID + 1) & 0x0f;
        }
    }

    return TransmittedSequenceID;
}


//=============================================================================
bool ParapetSerialDeviceNode::Receive( void )
{
    bool rc = false;

    if ( m_SerialPortIsOpen )
    {
        int NumBytesReceived =
              m_SerialPort.ReadNWithTimeOut( m_RxBuffer, RX_BUFFER_SIZE,
                                             10 );

        if ( NumBytesReceived > 0 )
        {
            rc = process( m_RxBuffer,
                          static_cast<uint16_t>( NumBytesReceived ) );
        }
    }

    return rc;
}





