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
/** @file LibBunnySock.h

@brief
	Master header file for LibBunnySock

@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the
	University of Idaho, USA.
*/
//=============================================================================

#ifndef _LIBBUNNYSOCK_H_
#define _LIBBUNNYSOCK_H_

// Currently, the platform isn't getting defined in an OpenEmbedded CMake build
// To fix this, we'll add this hack...
#ifndef UNIX
	#ifndef _WIN32
		#define UNIX
	#endif
#endif

//---------------------------------------------------
/** @def BUNNYSOCK_VERSION
 * @brief
 *  BunnySock protocol version reported to peers.
 */
#define LIBBUNNYSOCK_VERSION    1.0f

#include "MOOS/libMOOS/Utils/MOOSUtils.h" 

#include "BunnySockPacket.h"
#include "BunnySockNode.h"
#include "BunnySockListener.h"
#include "BunnySockListenerFIFO.h"
#include "BunnySockTcpNode.h"
#include "BunnySockUdpNode.h"


#endif	// END #ifndef _LIBBUNNYSOCK_H_
