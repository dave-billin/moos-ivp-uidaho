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
/*
 * @file ParapetSerialDeviceNode.h
 *
 * @brief
 *   Declaration of a class that supplies a parapet device node that
 *   communicates over a MOOS serial port
 */
//=============================================================================
#ifndef PARAPETSERIALMASTER_H_
#define PARAPETSERIALMASTER_H_

#include "parapet.h"
#include "MOOS/libMOOS/Utils/MOOSLinuxSerialPort.h"
#include "PrecisionTime.h"
#include "ParapetDeviceNode.h"
#include "BSDuC_parapet_packets.h"


//=============================================================================
/** @class ParapetSerialDeviceNode
 * @brief
 *   A class that implements a ParapetMaster node that communicates using a
 *   CMOOSSerialPort object
 */
//=============================================================================
class ParapetSerialDeviceNode : public parapet::ParapetDeviceNode
{
public:

    //=========================================================================
    /** Creates an instance of the object and associates it with a specified
     *  CMOOSSerialPort object
     *
     * @param [in] DeviceID
     *   parapet DeviceID of the node
     */
    ParapetSerialDeviceNode( uint8_t DeviceID );

    /** Called when the object goes out of scope */
    virtual ~ParapetSerialDeviceNode();

    //=============================
    // ParapetMaster API Methods
    //=============================

    //=========================================================================
    /** Transmits a parapet packet and registers it to receive a corresponding
     *  response
     *
     * @remarks
     *   This function automatically populates the SourceID and SequenceID
     *   fields of outgoing packets
     *
     * @param [in] Packet
     *   Reference to the packet to transmit
     *
     * @return
     *   Non-negative sequence ID of the packet on successful transmission;
     *   else -1 on failure to transmit the packet
     */
    int8_t Transmit( parapet::parapet_packet_t& Packet );


    //=========================================================================
    /** Called to receive and process data from parapet Slave devices
     *
     * @return
     *  true if one or more packets were received and processed
     */
    bool Receive( void );


    //=========================================================================
    /** Assigns the serial port device the object should use and attempts to
     *  open it
     *
     * @param [in] SerialPortDeviceName
     *   Name of the serial port device the object should use to communicate
     *   (e.g. "/dev/ttyS0/")
     *
     * @param [in] BaudRate
     *   Serial baud rate to use for communications
     *
     * @return
     *   true if the specified serial device was opened and the object is ready
     *   to send and receive data; else false on failure to open the serial
     *   device or if the serial device is already open
     */
    bool Open( std::string const& SerialPortDeviceName, uint32_t BaudRate );


    //=========================================================================
    /** Closes the object's serial device if it is open */
    void Close( void );

    //=========================================================================
    /** @return true if the object's serial device is open; else false */
    bool IsOpen( void ) const { return m_SerialPortIsOpen; }

    //=========================================================================
    std::string GetSerialPortDeviceName( void );

private:
    CMOOSLinuxSerialPort m_SerialPort;  /**< Serial port object used to
                                             communicate with the BSD
                                             microcontroller */

    bool m_SerialPortIsOpen;    /**< true if m_SerialPort is open */

    uint8_t m_SequenceID;   /**< Used to generate an incrementing sequence ID
                                 for transmitted parapet packets */

    enum { RX_BUFFER_SIZE = sizeof(parapet::parapet_packet_t) };
    char m_RxBuffer[RX_BUFFER_SIZE];    /**< Buffer used to receive data */

    //------------------------------------------------------------------
    // Disallow default constructor, copy constructor and
    // assignment operator
    ParapetSerialDeviceNode( void );
    ParapetSerialDeviceNode( ParapetSerialDeviceNode const& );
    ParapetSerialDeviceNode& operator=( ParapetSerialDeviceNode const& );
    //------------------------------------------------------------------
};


//=============================================================================
inline bool ParapetSerialDeviceNode::Open( std::string const& SerialPortDeviceName,
                                       uint32_t BaudRate )
{
    return m_SerialPort.Create( SerialPortDeviceName.c_str(),
                                              BaudRate );
}


//=============================================================================
inline void ParapetSerialDeviceNode::Close( void )
{
    if ( m_SerialPortIsOpen )
    {
        m_SerialPort.Close();
    }
}


//=============================================================================
inline std::string ParapetSerialDeviceNode::GetSerialPortDeviceName( void )
{
    return (m_SerialPortIsOpen == true) ? m_SerialPort.GetPortName() : "";
}

#endif /* PARAPETSERIALMASTER_H_ */
