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
/** @file BunnySockUdpNode.h
 *
 * @brief
 *	Definition of the BunnySockUdpNode class used to represent a single
 *	BunnySock TCP connection
 *
 * @author	Dave Billin
 */
//=============================================================================

#ifndef BUNNYSOCKUDPNODE_H_
#define BUNNYSOCKUDPNODE_H_

// Currently, the platform isn't getting defined in an OpenEmbedded CMake build
// To fix this, we'll add this hack...
#ifndef UNIX
	#ifndef _WIN32
		#define UNIX
	#endif
#endif

#include "MOOS/libMOOS/Utils/MOOSThread.h"
#include "MOOS/libMOOS/Comms/XPCUdpSocket.h"	// MOOS cross-platform UDP socket class
#include "YellowSubUtils.h"			// Needed for timed lock object
#include "BunnySockNode.h"

using YellowSubUtils::TimedLock;

namespace BunnySock
{

//=============================================================================
/** An object encapsulating a single BunnySock node capable of sending and
 *  receiving using connectionless UDP broadcast packets.
 *
 * @details
 *	BunnySock UDP nodes are provided as an adjunct to the normal (TCP) method
 *	of network communication.  They are <i>specifically intended</i> for the
 *	situation where a time-critical packet is being sent to <i>multiple</i>
 *	devices at once.  Unfortunately, due to the nature of the connection-less
 *	UDP protocol, there is no guarantee that packets sent from the node will
 *	actually be received by their respective destination.  As such, <b><i>TCP
 *	nodes should be favored over UDP nodes whenever possible</i></b>.
 */
class BunnySockUdpNode : public BunnySockNode
{
public:

	//=========================================================================
	/** Creates a (useless) BunnySock UDP node with a device ID and verbosity
	 *	of zero
	 */
	BunnySockUdpNode( void );


	//=========================================================================
	/** Creates a BunnySock Udp Node that communicates over a specified network
	 *  port
	 *
	 * @param Port
	 * 	Network port used for sending and receiving packets
	 *
	 * @param DeviceId
	 * 	Device ID to report in the SourceID field of packets
	 */
	BunnySockUdpNode(uint16_t Port, uint16_t DeviceId, uint16_t Verbosity = 0);


	/** Called when the node goes out of scope */
	virtual ~BunnySockUdpNode();


	//========================================
	// Inherited from BunnySockNode
	//========================================

	/** Inherited from the BunnySockNode class.  Does nothing, since UDP is
	 *  connectionless.
	 */
	void ResetConnection( void );


	/** Sends a BunnySock packet to the connected peer.
	 *
	 * @param [in] PacketToSend
	 * 	Reference to a BunnySockPacket to be sent
	 *
	 * @param [out] OUT_pPacketWasSent
	 *	Pointer to a variable that will be populated with the return value of
	 *	the send operation, or NULL if a return value is not required.
	 *		- true if the packet was sent successfully
	 *		- false if the packet could not be sent
	 */
	bool SendPacket( BunnySockPacket& PacketToSend );

	//========================================


	//=========================================================================
	/** If the node's connection mode is CLIENT, this returns the TCP port on the
	 *  remote host that the node will attempt to connect to.  Otherwise, if
	 *  the connection mode is SERVER, this returns the TCP port on the local
	 *  machine to listen on for incoming connections.
	 */
	int GetPort( void ) const;



	//=========================================================================
	/** Starts the node running.  This must be called before the node will
	 *  connect to a remote host and data can be sent or received */
	void Start( void );



	//=========================================================================
	/** Execution body of the internal worker thread used to send and receive
		packets.  Not intended to be called directly.  */
	void WorkerThreadBody( void );


private:
	long m_Port;			/**< TCP port to communicate on */

	TimedLock m_TxLock;		/**< Mutex used to prevent re-entrancy when
								 sending packets */

	XPCUdpSocket* m_pSocket;	/**< Udp socket used for communication */

	CMOOSThread m_WorkerThread;	/**< Worker thread used to receive packets */

	static const std::string sBunnySockUdpNode;


	//-------------------------------------------------------------------------
	// Prevent automatic generation of copy constructor and assignment operator
	BunnySockUdpNode(const BunnySockUdpNode&);
	const BunnySockUdpNode& operator= (const BunnySockUdpNode&);

};

}

#endif /* BUNNYSOCKUDPNODE_H_ */
