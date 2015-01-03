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
/** @file BunnySockNodeListener.h
 *
 * @brief
 *	Declaration of the BunnySockNodeListener interface class used to allow an
 *	object to receive notifications of received packets and connection events
 *
 * @author	Dave Billin
 */
//=============================================================================

#ifndef BUNNYSOCKNODELISTENER_H_
#define BUNNYSOCKNODELISTENER_H_

#include "BunnySockPacket.h"
//#include "BunnySockNode.h"



namespace BunnySock
{

class BunnySockNode;	// Pre-declare BunnySockNode base class


//=============================================================================
/** This interface class can be inherited by an object that needs to receive
 * packets or be notified when connection events occur on a BunnySock node.
*/
class BunnySockListener
{
public:

	//=========================================================================
	/** Function called when a packet is received on a BunnySock node.
	 * @note
	 *	This function is called on the receive thread of the BunnySock node,
	 *	which is not necessarily the same thread a listener object is executing
	 *	on.  Listener objects must take the necessary precautions to ensure
	 *	thread safety when handling received packets and events.
	 *
	 * @param RxPacket
	 *	A reference to the received BunnySock packet.  Use caution if modifying
	 *	the contents of the packet object for a node with multiple listeners!
	 *
	 * @param Node
	 *	Reference to the BunnySockNode object that received the packet
	 *
	 * @param TimeStamp_sec
	 *	LocalHost time when the packet was received.
	 */
	virtual void OnPacketReceived( BunnySockPacket& RxPacket,
								   BunnySockNode& Node,
								   double TimeStamp_sec ) = 0;


	//=========================================================================
	/** Called when a connection event (connect, disconnect, timeout, error)
	 * occurs on a BunnySock node.
	 *
	 * @details
	 *	Note that this function is called on the receive thread of the
	 *	BunnySockNode, which is NOT necessarily the same thread that
	 *	the listener object is executing on!  Listener objects must take
	 *	all necessary steps to ensure thread safety when handling
	 *	received packets.
	 *
	 * @param [in] EventId
	 * 	A member of e_ConnectionEventIds specifying the type of event that
	 *	occurred.
	 *
	 * @param [in] Node
	 *	Reference to the BunnySockNode object associated with the event
	 *
	 * @param [out] TimeStamp_sec
	 *	LocalHost time when the packet was received.
	*/
	virtual void OnConnectionEvent( int EventId, BunnySockNode& Node,
									double TimeStamp_sec ) = 0;


	//=============================================================================
	/** @enum e_NetworkEventIds
	@brief
		ID's of network events reported for a BunnySock node
	*/
	enum e_ConnectionEventIds
	{
		CONNECTED,			/**< A connection was established with a remote peer */
		DISCONNECTED,		/**< The node has disconnected from its remote peer */
		CONNECTION_ERROR,	/**< An error occurred with the connection to a
								 remote peer */
		CONNECTION_TIMEOUT 	/**< The connection with a remote peer timed out */
	};

};

};	// END namespace BunnySock


#endif /* BUNNYSOCKNODELISTENER_H_ */
