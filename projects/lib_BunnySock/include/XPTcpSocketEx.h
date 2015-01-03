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
/** @file XPTcpSocketEx.h
 *
 * @brief
 *	An extension of the MOOS XPCTcpSocket class
 *
 * @author	Dave Billin
 */
//=============================================================================

#ifndef _XPTCPSOCKETEX_H_
#define _XPTCPSOCKETEX_H_

// Currently, the platform isn't getting defined in an OpenEmbedded CMake build
// To fix this, we'll add this hack...
#ifndef UNIX
	#ifndef _WIN32
		#define UNIX
	#endif
#endif

#include <stdint.h>
#include <string>
#include "MOOS/libMOOS/Comms/XPCTcpSocket.h"



//=============================================================================
/** This class extends the MOOS XPCTcpSocket to provide a working Bind()
 *  implementation.
*/
class XPTcpSocketEx : public XPCTcpSocket
{
public:
    // Constructor.  Used to create a new TCP socket given a port
	XPTcpSocketEx(const std::string& sTargetHost, uint16_t Port);

	// Binds the socket to an address and port number
	//void BindSocket(string& sTargetHost, uint16_t Port);
};


#endif /* #ifndef _XPTCPSOCKETEX_H_ */
