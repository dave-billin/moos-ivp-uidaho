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
 * @file LinuxSerialPortEx.cpp
 *
 * @brief
 *  Implementation of the LinuxSerialPortEx class
 *
 * @author Dave Billin
 */

#include "LinuxSerialPortEx.h"

#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"

namespace YellowSubUtils
{


//=============================================================================
const char* s_DEFAULT_PORT = "/dev/ttyS0";

//=============================================================================
const int s_DEFAULT_BAUDRATE = 19200;

//=============================================================================
LinuxSerialPortEx::LinuxSerialPortEx()
 : CMOOSLinuxSerialPort::CMOOSLinuxSerialPort()
{
	// Just invoke base class implementation
}



//=============================================================================
LinuxSerialPortEx::~LinuxSerialPortEx()
{
	Close();
}



//=============================================================================
/** Create and set up the port */
bool LinuxSerialPortEx::Create(const char * sPort, int nBaudRate)
{
    if (m_nPortFD >= 0)
    {
        MOOSTrace("Serial Port already open.\n");
        return false;
    }

#ifndef _WIN32
    int nLinuxBaudRate = B9600;
    switch(nBaudRate)
    {
		case 1000000:	nLinuxBaudRate = B1000000; break;
		case 921600:	nLinuxBaudRate = B921600; break;
		case 576000:	nLinuxBaudRate = B576000; break;
		case 500000:    nLinuxBaudRate = B500000; break;
		case 460800:	nLinuxBaudRate = B460800; break;
		case 230400:	nLinuxBaudRate = B230400; break;
		case 115200:    nLinuxBaudRate = B115200; break;
		case 38400:     nLinuxBaudRate = B38400;  break;
		case 19200:     nLinuxBaudRate = B19200;  break;
		case 9600:      nLinuxBaudRate = B9600;   break;
		case 4800:      nLinuxBaudRate = B4800;   break;
		case 2400:      nLinuxBaudRate = B2400;   break;
		case 1200:      nLinuxBaudRate = B1200;   break;
		case 600:       nLinuxBaudRate = B600;    break;
		case 300:       nLinuxBaudRate = B300;    break;
		default :
			printf("Unsupported baud rate\n");
			return false;
			break;
    }

    // open and configure the serial port
    m_nPortFD = open(sPort, O_RDWR | O_NOCTTY | O_NDELAY);

    if (m_nPortFD <0)
    {
        perror(sPort);
        return false;
    }

    //save the current configuration
    tcgetattr(m_nPortFD,&m_OldPortOptions);

    //zero the buffers
    //bzero(&m_PortOptions, sizeof(m_PortOptions));
    memset(&m_PortOptions,0,sizeof(m_PortOptions));
    m_PortOptions.c_cflag = nLinuxBaudRate | CS8 | CLOCAL | CREAD;
    m_PortOptions.c_iflag = IGNPAR;
    m_PortOptions.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    m_PortOptions.c_lflag = 0;

    // inter-character timer unused
    m_PortOptions.c_cc[VTIME]    = 0;
    // blocking read until 0 chars received, i.e. don't block
    m_PortOptions.c_cc[VMIN]     = 0;

    //save the new settings
    tcflush(m_nPortFD, TCIFLUSH);
    tcsetattr(m_nPortFD,TCSANOW,&m_PortOptions);

#endif

	if(m_nPortFD!=0)
		m_sPort = sPort;

    return  m_nPortFD!=0;
}

} /* namespace YellowSubUtils */
