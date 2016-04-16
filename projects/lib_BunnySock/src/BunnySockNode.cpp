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
/** @file BunnySockNode.cpp
 *
 * @brief
 *  Implementation of the BunnySockNode base class
 *
 * @author Dave Billin
 */
//=============================================================================

#include <time.h>
#include <signal.h>
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "BunnySockNode.h"

using namespace std;


#ifdef UNIX
	#include <errno.h>
#endif

using namespace::BunnySock;


//=============================================================================
void BunnySockNode::IgnoreSigPipe( void )
{
	#ifdef UNIX
	sigset_t SignalMask;	// Signals to block
	int Rc;					// Return code

	sigemptyset (&SignalMask);
	sigaddset (&SignalMask, SIGPIPE);
	Rc = pthread_sigmask (SIG_BLOCK, &SignalMask, NULL);
	if (Rc != 0)
	{
		MOOSTrace("Failed to ignore SIGPIPE signal!\n");
	}

	// NOTE: any newly created threads will inherit the signal mask
	#endif
}



//=============================================================================
BunnySockNode::BunnySockNode( void )
: m_ConnectionTimeout_ms(2000),
  m_IsConnected(false),
  m_Verbosity(0),
  m_DeviceId(0)
{
}


//=============================================================================
BunnySockNode::BunnySockNode( BunnySockListener& listener, uint16_t DeviceId,
							  uint16_t Verbosity)
: m_IsConnected(false),
  m_Verbosity(Verbosity),
  m_DeviceId(DeviceId)
{
	AddListener(listener);
}


//=============================================================================
BunnySockNode::~BunnySockNode()
{
}



//=============================================================================
void BunnySockNode::AddListener( BunnySockListener& listener )
{
   m_ListenerListLock.Lock();
   m_ListenerList.push_back(&listener);
   m_ListenerListLock.UnLock();
}



//=============================================================================
void BunnySockNode::RemoveListener( BunnySockListener& listener )
{
   m_ListenerListLock.Lock();
   m_ListenerList.remove(&listener);
   m_ListenerListLock.UnLock();
}



//=============================================================================
void BunnySockNode::ReportConnectionEvent(
                                 BunnySockListener::ConnectionEventId EventId )
{
	BunnySockListener* pListener;

	// Report the disconnect to all listeners
	for (list<BunnySockListener*>::iterator iter = m_ListenerList.begin();
		 iter != m_ListenerList.end(); iter++)
	{
		pListener = *iter;
		pListener->OnConnectionEvent(EventId, *this, HPMOOSTime());
	}
}

