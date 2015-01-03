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
/** @file TimedLock.cpp
@brief
	Implementation of the TimedLock class

@author	Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the 
	University of Idaho, USA.
*/
//=============================================================================

#ifdef UNIX
	#include <unistd.h>
	#include <time.h>
#endif

#include "TimedLock.h"


namespace YellowSubUtils
{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//=============================================================================
TimedLock::TimedLock( bool ShouldInitiallyBeLocked )
{
	#ifdef _WIN32
    m_hLock = ::CreateEvent(NULL,false,bInitial,NULL);
	#else
    pthread_mutex_init(&m_hLock,NULL);
	#endif
}


//=============================================================================
TimedLock::~TimedLock()
{
#ifdef _WIN32
	if ( (m_hLock != INVALID_HANDLE_VALUE) && (m_hLock != 0) )
	{
		::CloseHandle(m_hLock);
	}
#else
	pthread_mutex_destroy(&m_hLock);
#endif
}


//=============================================================================
bool TimedLock::Lock( int TimeoutMs )
{
#ifdef _WIN32
	DWORD NumMs = (TimeoutMs == TIMEOUT_INFINITE) ? INFINITE : TimeoutMs;

	if ( (m_hLock == INVALID_HANDLE_VALUE) || (m_hLock == 0) )
	{
		return false;
	}
	else
	{
		return ( WaitForSingleObject(m_hLock, NumMs) == WAIT_OBJECT_0 );
	}

#else
	if (TimeoutMs == TIMEOUT_IMMEDIATE)		// No blocking
	{
		return (pthread_mutex_trylock(&m_hLock) == 0);
	}
	else if (TimeoutMs == TIMEOUT_INFINITE)	// Block forever
	{
		return (pthread_mutex_lock(&m_hLock) == 0);
	}
	else	// Block with a specified timeout
	{
		struct timespec AbsoluteTime;	// Absolute time when timeout expires

		// Try to get the current time from the hi-res system clock
		//MOOSAssert( (clock_gettime(CLOCK_REALTIME, &AbsoluteTime)), /* expr */
		//			"clock_gettime() failed to return CLOCK_REALTIME",
		//			"TimedLock.cpp",
		//			83 );
		clock_gettime(CLOCK_REALTIME, &AbsoluteTime);

		AbsoluteTime.tv_nsec += (TimeoutMs * 1000);

		return (pthread_mutex_timedlock(&m_hLock, &AbsoluteTime) == 0);
	}

#endif

}



//=============================================================================
void TimedLock::UnLock()
{
#ifdef _WIN32
    if ( (m_hLock != INVALID_HANDLE_VALUE) && (m_hLock != 0) )
    {
        ::SetEvent(m_hLock);
    }
#else
    pthread_mutex_unlock(&m_hLock);
#endif
}


};	// END namespace YellowSub
