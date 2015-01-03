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
/** @file DebugFileSerialPort.h
 *
 * @brief
 *  An extension of the CMOOSSerialPort class that reads/writes using a regular
 *  file
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef DEBUGFILESERIALPORT_H_
#define DEBUGFILESERIALPORT_H_

#include <iostream>
#include <fstream>

#include "MOOS/libMOOS/Utils/MOOSSerialPort.h"


namespace YellowSubUtils
{


//=============================================================================
/** This class implements the CMOOSSerialPort interface, but reads and writes
 * using regular files instead of an actual serial port.  This can be very
 * useful for debugging and prototyping.
 */
class DebugFileSerialPort: public CMOOSSerialPort
{
public:

    //=========================================================================
    /** Creates an instance of the object */
    DebugFileSerialPort( void );

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~DebugFileSerialPort();

	//=========================================================================
	/** Closes the file attached to the virtual serial port */
    virtual bool Close( void );

    //=========================================================================
    /** Creates and sets up the port
     *
     * @param szPortName
     *  Full path of a regular file to open for read/write access
     *
     * @param nBaudRate
     *  This argument is ignored, and is included only for consistency with
     *  CMOOSSerialPort.
     *
     * @returns
     *  false if the file specified could not be opened for read and write
     *  access; else true
     */
    virtual bool Create( const char* szPortName=DEFAULT_PORT,
                         int nBaudRate=DEFAULT_BAUDRATE );


    //=========================================================================
    /** Writes one or more Bytes to the virtual serial port file
     *
     * @param pSerialBytes
     *  Serial data Bytes to be written
     *
     * @param NumBytes
     *  The number of Bytes to write
     *
     * @param pTime
     *  Pointer to a double that will be populated with a system time stamp
     *  indicating when the Bytes were written, or NULL to skip time stamping
     *
     * @returns
     *  The number of Bytes written to the virtual serial port
     */
    int Write( const char* pSerialBytes, int NumBytes, double* pTime=NULL );


    //=========================================================================
    /** This method does nothing, and is included only for consistency with the
     * CMOOSSerialPort interface
     */
    virtual void Break( void ) {;}

    //=========================================================================
    /** This method does nothing, and is included only for consistency with the
     * CMOOSSerialPort interface
     */
    virtual int Flush( void ) { return 0;}


    //=========================================================================
    /** Returns the full path of the file associated with the virtual
     *  serial port
     */
    std::string GetFilePath( void ) { return sPortFilePath; }


protected:
    std::string sPortFilePath;   /**< File attached to the serial port */
    std::fstream PortFileStream; /**< Stream for accessing the serial port */


    /** Just grab N characters NOW */
    virtual int GrabN( char* pBuffer, int NumBytesToRead );

private:
    //-------------------------------------------------
    // Disable automatically-generated functions
    //-------------------------------------------------
    DebugFileSerialPort (const DebugFileSerialPort&);
    const DebugFileSerialPort& operator= (const DebugFileSerialPort&);

};

}	// END namespace YellowSubUtils

#endif /* DEBUGFILESERIALPORT_H_ */
