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
/** @file BunnySockListenerFIFO.h
 *
 * @brief
 *  Implementation of the BunnySockListenerFIFO class
 */
//=============================================================================

#ifndef BUNNYSOCKLISTENERFIFO_H_
#define BUNNYSOCKLISTENERFIFO_H_

#include <list>
#include "MOOS/libMOOS/Utils/MOOSLock.h"
#include "BunnySockPacket.h"
#include "BunnySockListener.h"

namespace BunnySock
{


class BunnySockListenerFIFO : BunnySockListener
{
public:
	BunnySockListenerFIFO( BunnySockNode& NodeToListenTo );
	virtual ~BunnySockListenerFIFO();

	//=========================================================================
	/** Returns the number of BunnySock packets currently in the FIFO */
	size_t NumPacketsInFIFO( void ) const { return m_PacketFIFO.size(); }

	//=========================================================================
	/** Returns the oldest BunnySock packet after removing it from the FIFO.
	 *
	 * @param TargetPacket
	 * 	Reference to a packet object to be populated with the oldest packet
	 * 	in the FIFO.
	 *
	 * @return
	 * 	true if a packet was returned in TargetPacket; else false if the FIFO
	 * 	was empty.
	 */
	bool GetOldestPacket( BunnySockPacket& TargetPacket );

	//=========================================================================
	/** Returns the newest BunnySock packet after removing it from the FIFO.
	 *
	 * @param TargetPacket
	 * 	Reference to a packet object to be populated with the newest packet
	 * 	in the FIFO.
	 *
	 * @return
	 * 	true if a packet was returned in TargetPacket; else false if the FIFO
	 * 	was empty.
	 */
	bool GetNewestPacket( BunnySockPacket& TargetPacket );


	//=========================================================================
	/** Returns the oldest BunnySock packet but does not remove it from the
	 *  FIFO.
	 *
	 * @param TargetPacket
	 * 	Reference to a packet object to be populated with the oldest packet
	 * 	in the FIFO.
	 *
	 * @return
	 * 	true if a packet was returned in TargetPacket; else false if the FIFO
	 * 	is empty.
	 */
	bool PeekOldestPacket( BunnySockPacket& TargetPacket );


	//=========================================================================
	/** Returns the newest BunnySock packet but does not remove it from the
	 *  FIFO.
	 *
	 * @param TargetPacket
	 * 	Reference to a packet object to be populated with the newest packet
	 * 	in the FIFO.
	 *
	 * @return
	 * 	true if a packet was returned in TargetPacket; else false if the FIFO
	 * 	is empty.
	 */
	bool PeekNewestPacket( BunnySockPacket& TargetPacket );


	//=========================================================================
	/** @internal */
	void OnPacketReceived( BunnySockPacket& RxPacket, BunnySockNode& Node,
						   double TimeStamp_sec );


	//=========================================================================
	/** @internal */
	void OnConnectionEvent( int EventId, BunnySockNode& Node,
							double TimeStamp_sec );

private:
	std::list<BunnySockPacket> m_PacketFIFO;	/**< Received packet FIFO */
	CMOOSLock m_FifoLock;	/**< Used to synchronize packet FIFO */

	// Disable copy constructor and assignment operator
	BunnySockListenerFIFO(const BunnySockListenerFIFO& src);
	const BunnySockListenerFIFO& operator=(const BunnySockListenerFIFO& src);
};

}; // END namespace BunnySock

#endif /* BUNNYSOCKLISTENERFIFO_H_ */
