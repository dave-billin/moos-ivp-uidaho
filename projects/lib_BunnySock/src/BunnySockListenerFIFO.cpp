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
/** @file BunnySockListenerFIFO.cpp
 *
 * @brief
 *  Implementation of the BunnySockListenerFIFO class
 */
//=============================================================================
#include "BunnySockNode.h"
#include "BunnySockListenerFIFO.h"

using namespace::std;
using namespace::BunnySock;

//=============================================================================
BunnySockListenerFIFO::BunnySockListenerFIFO( BunnySockNode& NodeToListenTo )
: m_FifoLock(false)
{
	NodeToListenTo.AddListener(this);	// Register this FIFO to receive packets
										// and events from the target node
}


//=============================================================================
BunnySockListenerFIFO::~BunnySockListenerFIFO()
{
}


//=============================================================================
bool BunnySockListenerFIFO::GetOldestPacket( BunnySockPacket& TargetPacket )
{
	bool Rc = false;

	m_FifoLock.Lock();		// Synchronize
	if (m_PacketFIFO.size() > 0)
	{
		TargetPacket = m_PacketFIFO.front();
		m_PacketFIFO.pop_front();
		Rc = true;
	}
	m_FifoLock.UnLock();

	return Rc;
}

//=============================================================================
bool BunnySockListenerFIFO::GetNewestPacket( BunnySockPacket& TargetPacket )
{
	bool Rc = false;

	m_FifoLock.Lock();		// Synchronize
	if (m_PacketFIFO.size() > 0)
	{
		TargetPacket = m_PacketFIFO.back();
		m_PacketFIFO.pop_back();
		Rc = true;
	}
	m_FifoLock.UnLock();

	return Rc;
}



//=============================================================================
bool BunnySockListenerFIFO::PeekOldestPacket( BunnySockPacket& TargetPacket )
{
	bool Rc = false;

	m_FifoLock.Lock();		// Synchronize
	if (m_PacketFIFO.size() > 0)
	{
		list<BunnySockPacket>::iterator iter = m_PacketFIFO.begin();
		TargetPacket = *iter;
		Rc = true;
	}
	m_FifoLock.UnLock();

	return Rc;
}



//=============================================================================
bool BunnySockListenerFIFO::PeekNewestPacket( BunnySockPacket& TargetPacket )
{
	bool Rc = false;

	m_FifoLock.Lock();		// Synchronize
	if (m_PacketFIFO.size() > 0)
	{
		list<BunnySockPacket>::reverse_iterator riter = m_PacketFIFO.rbegin();
		TargetPacket = *riter;
		Rc = true;
	}
	m_FifoLock.UnLock();

	return Rc;
}

//=============================================================================
void BunnySockListenerFIFO::OnPacketReceived( BunnySockPacket& RxPacket,
											  BunnySockNode& Node,
											  double TimeStamp_sec )
{
	// Do nothing.  We only care about packets...
	m_FifoLock.Lock();		// Synchronize
	m_PacketFIFO.push_back(RxPacket);
	m_FifoLock.UnLock();
}


//=============================================================================
void BunnySockListenerFIFO::OnConnectionEvent( int EventId,
											   BunnySockNode& Node,
											   double TimeStamp_sec )
{
}
