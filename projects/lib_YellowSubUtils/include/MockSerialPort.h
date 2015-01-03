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
/** @file MockSerialPort.h
 *
 * @brief
 *   A short description of the file
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef MOCKSERIALPORT_H_
#define MOCKSERIALPORT_H_

#include <cstddef>
#include <stdint.h>
#include "MOOS/libMOOS/Utils/MOOSLinuxSerialPort.h"


namespace YellowSubUtils
{

//=============================================================================
/** @class MockSerialPort
 *
 * @brief
 *   A class supplying a mock serial port device for debugging and unit
 *   testing
 *
 * @details
 *   The MockSerialPort object acts like a normal CMOOSSerialPort object, with
 *   the exception that transmitted data is written to an internal data buffer,
 *   that can be read directly.  Similarly, data can be written directly to the
 *   MockSerialPort object's Rx buffer, whereupon it will be 'read' in calls
 *   to the object's GrabN() and associated read methods.  The object is
 *   intended as a means for debugging and unit testing, rather than to be
 *   deployed in an actual operating environment.
 */
//=============================================================================
class MockSerialPort : public CMOOSSerialPort
{
public:

    /// Default serial port device name
    static const char* s_DEFAULT_DEVICE;

    /// Default serial baud rate
    static const int s_DEFAULT_BAUDRATE;

    //=========================================================================
    /** Creates an instance of the object but does not associate it with any
     *  serial port device.  */
    MockSerialPort();

    //=========================================================================
    /** Called when the object goes out of scope */
    virtual ~MockSerialPort();

    /** Called to associate the object with a specified serial port and set the
     * baud rate used for communications.
     *
     * @param [in] SerialDeviceName
     *   Name of the serial port device this object will open and manage
     *
     * @param [in] BaudRate
     *   Serial baud rate to open the device with
     */
    virtual bool Create( const char* SerialDeviceName = s_DEFAULT_DEVICE,
                         int BaudRate = s_DEFAULT_BAUDRATE );

    //=========================================================================
    /** Closes the object's virtual serial port and erases the contents of its
     *  Tx and Rx buffers.
     *
     * @return
     *   false if the object's serial port is not open; else true
     */
    virtual bool Close();

    //=========================================================================
    /** @return true if the virtual serial port is 'open' */
    bool IsOpen( void ) const { return m_IsOpen; }

    //=========================================================================
    /** Writes data to the object's virtual serial port
     *
     * @remarks
     *   Transmitted data is placed in the MockSerialPort object's Tx buffer,
     *   and can be read via the ReadTxBuffer() method
     *
     * @param [in] TxData
     *   The data to be transmitted
     *
     * @param [in] NumBytes
     *   The number of Bytes in TxData to transmit
     *
     * @param [in] TxTimeStamp
     *   Timestamp to be populated on Tx completion
     */
    int Write( const char* TxData, int NumBytes,
               double* TxTimeStamp = NULL );

    //=========================================================================
    /** Flushes the port (has no effect)
     *
     * @return
     *   Returns zero (success) if the object has been 'opened', else -1 if
     *   the object is 'closed'
     */
    virtual int Flush() { return ( m_IsOpen ) ? 0 : -1; }




    //=========================================================================
    /** Writes data to the object's Rx buffer where it will be 'read' from the
     *  object's virtual serial port
     *
     * @param [in] RxData
     *   Data to append to the end of the object's Rx buffer
     *
     * @param [in] NumBytes
     *   Number of Bytes in RxData to copy into the Rx buffer
     */
    void AddRxData( char const* RxData, int NumBytes );

    /** @return The number of Bytes in the object's Rx buffer */
    int NumBytesInRxBuffer( void ) const { return m_RxBuffer.size(); }

    /** Erases the contents of the object's Rx data buffer */
    void ClearRxBuffer( void ) { m_RxBuffer.clear(); }


    //=========================================================================
    /** Reads data that has been 'transmitted' by the object's virtual serial
     *  port
     *
     * @param [out] TargetBuffer
     *   Address where data from the object's Tx buffer should be placed
     *
     * @param [in] NumBytes
     *   The number of Bytes to read from the object's Tx buffer
     *
     * @return
     *   The number of Bytes placed in TargetBuffer
     */
    int GetTxData( char* TargetBuffer, int NumBytes );

    int NumBytesInTxBuffer( void ) const { return m_TxBuffer.size(); }

    /** Erases the contents of the objects Tx data buffer */
    void ClearTxBuffer( void ) { m_TxBuffer.clear(); }


protected:
    bool m_IsOpen;  /**< true if the mock serial port is 'open'; else false */

    /// Data available to be read from the object's virtual serial port
    std::string m_RxBuffer;

    /// Data that has been written to the object's virtual serial port
    std::string m_TxBuffer;


    //=========================================================================
    /** Implementation of the CMOOSSerialPort interface method for reading data
     *  from the object's virtual serial port
     *
     * @remarks
     *   Data will be read from the object's RxBuffer
     *
     * @param [in] TargetBuffer
     *   Buffer to populate with received data
     *
     * @param NumBytes
     *   Maximum number of Bytes to read into TargetBuffer
     *
     * @return
     *   The number of Bytes placed in TargetBuffer
     */
    virtual int GrabN( char* TargetBuffer, int NumBytes );


    //=========================================================================
    /** Helper function to move data from the beginning of a std::string into a
     *  specified buffer
     *
     * @param [in,out] SourceString
     *   String to copy data from
     *
     * @param [out] TargetBuffer
     *   Buffer to copy data to
     *
     * @param [in] NumBytes
     *   Maximum number of Bytes to read into TargetBuffer
     *
     * @return
     *   The number of characters read into TargetBuffer
     *
     * @post
     *   Characters read into TargetBuffer are removed from SourceString
     */
    int ReadDataFromString( std::string& SourceString, char* TargetBuffer,
                            int NumBytes );


    //-------------------------------------------------------------------------
    // Disallow copy constructor and assignment operator
    MockSerialPort( MockSerialPort const& );
    MockSerialPort& operator=( MockSerialPort const& );
    //-------------------------------------------------------------------------
};



//=============================================================================
inline int MockSerialPort::GetTxData( char* TargetBuffer, int NumBytes )
{
    return ReadDataFromString( m_TxBuffer, TargetBuffer, NumBytes );
}

//=============================================================================
inline int MockSerialPort::GrabN( char* TargetBuffer, int NumBytes )
{
    return ReadDataFromString( m_RxBuffer, TargetBuffer, NumBytes );
}


} /* namespace YellowSubUtils */
#endif /* MOCKSERIALPORT_H_ */
