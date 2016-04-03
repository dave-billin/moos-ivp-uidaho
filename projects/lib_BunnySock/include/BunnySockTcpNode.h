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
/** @file BunnySockTcpNode.h
 *
 * @brief
 *	Definition of the BunnySockTcpNode class used to represent a single
 *	BunnySock TCP connection
 *
 * @author	Dave Billin
 */
//=============================================================================

#ifndef BUNNYSOCKTCPNODE_H_
#define BUNNYSOCKTCPNODE_H_

#include <string>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "MOOS/libMOOS/Utils/MOOSThread.h"

#include "XPTcpSocketEx.h"			// Extension of MOOS TCP socket class
#include "YellowSubUtils.h"
#include "BunnySockNode.h"



namespace BunnySock
{

//=============================================================================
/** An object encapsulating a single BunnySock node communicating over a TCP
 *  network connection */
class BunnySockTcpNode : public BunnySockNode
{
public:

	//=========================================================================
	/** Creates a (useless) BunnySock TCP Server node with remote host address
	 *  0.0.0.0, device ID 0, and verbosity 0 */
	BunnySockTcpNode( void );


	//=========================================================================
	/** Creates a BunnySock Tcp Node bound to a specified remote peer address
	 *  and port
	 *
	 * @param ConnectionMode
	 *	A member of e_ConnectionModeIds specifying how the node should connect
	 *	to a BunnySock peer.
	 *
	 * @param sTargetHostName
	 * 	Hostname or IP address of the peer the node should try to associate
	 *  with.  If ConnectionMode is SEEK, this parameter specifies the host
	 *  the node will try to connect to.  If ConnectionMode is SERVER, this
	 *  parameter is not used, as the connected peer's hostname is obtained
	 *  when an incoming connection is opened.
	 *
	 * @param Port
	 * 	Network port used to connect to the specified remote peer
	 *
	 * @param DeviceId
	 * 	Device ID to report in the SourceID field in keep-alive packets
	 *
	 * @param RetryPeriodSec
	 * 	If ConnectionMode is CLIENT, this parameter specifies a minimum number
	 * 	of seconds that must elapse between successive attempts by the node to
	 *	connect/reconnect to the target remote peer.  If ConnectionMode
	 *	is SERVER, this parameter is not used.
	 *
	 * @param ConnectionTimeoutMs
	 *	Timeout used for the TCP connection with the peer.  If no packets
	 *	(keep-alive or otherwise) are received from the peer within this
	 *	period, the connection will be reset and the node will resume
	 *	connecting in its assigned node (client/server).
	 *
	 * @param Verbosity
	 *	Sets the verbosity of debugging and status messages printed to stdio.
	 *	Larger values result in more messages being printed.  The default value
	 *	of zero disables all debugging and status message printing.
	 */
	BunnySockTcpNode( int ConnectionMode, std::string& sTargetHostName,
					  uint16_t Port, uint16_t DeviceId,
					  uint32_t RetryPeriodSec = 2,
					  uint32_t ConnectionTimeoutMs = BUNNYSOCK_DEFAULT_CONNECTION_TIMEOUT_MS,
					  uint16_t Verbosity = 0 );


	/** Called when the node goes out of scope */
	virtual ~BunnySockTcpNode();


	//========================================
	// Inherited from BunnySockNode
	//========================================

	/** Resets the remote peer connection.
	 * @details
	 * 	If a TCP connection with a remote peer exists, it is closed.  Following
	 *	this, the node will resume operation using its assigned connection
	 *	mode.
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
	/** Returns the node's connection mode as an element of e_ConnectionModeIds
	 */
	int GetConnectionMode( void ) const;


	//=========================================================================
	/** Returns the name of the remote host the node is connected to, or an
	 *  empty string if not connected
	 */
	std::string GetRemoteHostName( void ) const;


	//=========================================================================
	/** If the node's connection mode is CLIENT, this returns the TCP port on
	 *  the target remote host the node will try to connect to.  If the node's
	 *  connection mode is SERVER, this returns the port on the local machine
	 *  that the node will listen on for incoming connections.
	 */
	int GetPort( void ) const;


	//=========================================================================
	/** Returns the device ID of the connected peer, or 0 if no peer is
	 *  connected */
	uint16_t GetPeerDeviceId( void ) const;


	//=========================================================================
	/** Starts the node running.  This must be called before the node will
	 *  connect to a remote host and data can be sent or received */
	void Start( void );


	//=========================================================================
	/** Sets the connection timeout for the node
	 *
	 * @param TimeoutMs
	 * 	The maximum time in milliseconds that may elapse without receiving a
	 * 	packet from a connected peer.  If this timeout expires, the	connection
	 * 	with the peer is reset.  This value is also used to determine the rate
	 * 	at which the node sends keep-alive packets a connected peer.
	 * 	Keep-alive packets will be sent after approximately 3/4 of this time
	 * 	have elapsed without sending a packet.
	 */
	void SetConnectionTimeoutMs( uint32_t TimeoutMs );

	/** Returns the connection timeout setting for the node in milliseconds */
	uint32_t GetConnectionTimeoutMs( void ) const;

	//---------------------------------
	/** @enum e_ConnectionModeIds
	 * @brief
	 * 	ID's used to specify the behavior of a BunnySock TCP node
	 */
	enum e_ConnectionModeIds
	{
		SERVER = 1000,	/**< Listens for an incoming connection from the
						 	 registered remote peer */

		CLIENT,		/**< Actively tries to open a connection with the
						 registered remote peer */

		NUM_CONNECTIONMODE_IDS	/**< Internal use only */
	};


	//============================================================================
	/** Execution body of the worker thread used to send and receive data.  Not
		intended to be called directly.
	*/
	void WorkerThreadBody( void );


private:
	std::string m_sPeerHostname;	/**< Target remote peer host name as a string */

	uint16_t m_Port;		/**< TCP port to communicate on */

	int m_ConnectionMode;	/**< Connection mode of the node */

	uint32_t m_RetryPeriodSec;	/**< Minimum connection retry period
									 (milliseconds) used in CLIENT mode */

	uint32_t m_ConnectionTimeout_ms;/**< Maximum time that may elapse without
										 receiving a packet from a connected
										 peer before the connection is reset */

	BunnySockPacket m_KeepAlivePacket;	/**< Keep-alive packet sent
											 periodically to a connected
											 peer */

	unsigned int m_PeerDeviceId;/**< Device ID of a connected peer or 0 if not
									 connected */

	YellowSubUtils::PrecisionTime m_LastRxTime;	/**< Time last packet was
	                                                 sent */
	YellowSubUtils::PrecisionTime m_LastTxTime;	/**< Time last packet was
	                                                 received */

	YellowSubUtils::TimedLock m_TxLock;	/**< Used to synchronize access to the
									         node's TCP socket when sending
									         packets */

	XPCTcpSocket* m_pSocket;	/**< TCP socket used for communication */

	CMOOSThread m_WorkerThread;	/**< Worker thread used to receive packets */

	static const std::string sBunnySockTcpNode;

	/** Helper function to open a peer connection for nodes set to CLIENT mode */
	bool OpenClientConnection( void );

	/** Helper function to open a peer connection for nodes set to SERVER mode */
	bool OpenServerConnection( void );


	//-------------------------------------------------------------------------
	// Prevent automatic generation of copy constructor and assignment operator
	BunnySockTcpNode(const BunnySockTcpNode&);
	const BunnySockTcpNode& operator= (const BunnySockTcpNode&);

};


}	// END namespace BunnySock

#endif /* BUNNYSOCKTCPNODE_H_ */
