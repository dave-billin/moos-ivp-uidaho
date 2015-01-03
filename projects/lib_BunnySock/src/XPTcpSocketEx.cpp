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
/** @file XPTcpSocketEx.cpp
 *
 * @brief
 *	Implementation of the XPTcpSocketEx class
 *
 * @author	Dave Billin
 */
//=============================================================================

#include "XPTcpSocketEx.h"

using namespace::std;


XPTcpSocketEx::XPTcpSocketEx(const string& sTargetHost, uint16_t Port)
: XPCTcpSocket(static_cast<long>(Port))
{
	struct sockaddr_in TargetHostAddress;
	const char* szTargetHost = sTargetHost.c_str();

    TargetHostAddress.sin_family = AF_INET;
    TargetHostAddress.sin_port = htons(iPort);

    // Resolve the IP address of the given host name
    hostType HostType;
    if(sTargetHost.find_first_not_of("0123456789. ") != std::string::npos)
    {
        HostType = NAME;
        XPCGetHostInfo getHostInfo(szTargetHost, HostType);

        // Store the IP address and socket port number
        TargetHostAddress.sin_addr.s_addr = inet_addr(getHostInfo.sGetHostAddress());
    }
    else
    {
        HostType = ADDRESS;
        // Store the IP address and socket port number
        TargetHostAddress.sin_addr.s_addr =inet_addr(szTargetHost);
    }

    clientAddress = TargetHostAddress;
}


// Binds the socket to an address and port number
/*void XPTcpSocketEx::BindSocket(string& sTargetHost, uint16_t Port)
{

}
*/
