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
/* @file LinuxSerialPortEx.h
 *
 * @brief
 * 	Declaration of an extension to MOOSLinuxSerialPort that adds a 480600 bps
 * 	baud rate.
 */
//=============================================================================

#ifndef LINUXSERIALPORTEX_H_
#define LINUXSERIALPORTEX_H_

#include "MOOS/libMOOS/Utils/MOOSLinuxSerialPort.h"


namespace YellowSubUtils
{

/** This object extends the CMOOSLinuxSerialPort class to provide support for
 * additional baud rates such as the 460800 bps rate used by the Archangel IMU3
 */
class LinuxSerialPortEx : public CMOOSLinuxSerialPort
{
public:

    static const char* s_DEFAULT_PORT;
    static const int s_DEFAULT_BAUDRATE;

	/** Creates an instance of the object but does not associate it with any
	 *  serial port device.	 */
	LinuxSerialPortEx();

	/** Called when the object goes out of scope */
	virtual ~LinuxSerialPortEx();

	/** Called to associate the object with a specified serial port and set the
	 * baud rate used for communications. */
	virtual bool Create( const char* pPortNum=s_DEFAULT_PORT,
						 int nBaudRate=s_DEFAULT_BAUDRATE );

};

} /* END namespace YellowSubUtils */
#endif /* LINUXSERIALPORTEX_H_ */
