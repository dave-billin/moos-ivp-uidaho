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
/** @file TimedLock.h
 * @brief
 *	A version of the CmoosLock class that adds an optional timeout to the
 *	Lock() method
 *
 * @author	Dave Billin
 *
 * @par Created for
 *	The Microcomputer Research and Communications Institute (MRCI) - at the
 *	University of Idaho, USA.
 */
//=============================================================================

#ifndef _TIMEDLOCK_H_
#define _TIMEDLOCK_H_

#include <stdint.h>

#ifdef _WIN32
    #include "windows.h"
    #include "winbase.h"
    #include "winnt.h"
#endif

#ifdef UNIX
    #include <pthread.h>
#endif

#ifdef PLATFORM_DARWIN
	#include "Semaphore.h"
#endif

namespace YellowSubUtils
{

/** A simple mutex class that offers an optional timeout period when locking.
*/
class TimedLock
{
public:

	//-----------------------------------------
	/** Special timeout value constants for use with the Lock() method */
	enum e_TimeoutConstants
	{
		TIMEOUT_INFINITE = -1,	/**< Specifies an infinitely long timeout (i.e.
									 forever) */
		TIMEOUT_IMMEDIATE = 0	/**< Specifies an immediate timeout.  This equates
									 to no blocking or waiting */
	};
	//-----------------------------------------


	//============================================================================
	/** Creates an instance of the the timed lock mutex
	@param ShouldInitiallyBeLocked
		true if the mutex should initially be locked (this is the default case); 
		else false
	*/
    TimedLock(bool ShouldInitiallyBeLocked = true);

	/** Destructor - called when the lock object goes out of scope */
    virtual ~TimedLock();


	//============================================================================
	/** Attempts to acquire the lock, and blocks up to a specified period of time
		before timing out

	@param TimeoutMs
		The maximum number of milliseconds that the caller should be blocked while
		waiting to acquire the lock, or one of the following special values:
		- TIMEOUT_IMMEDIATE : Specifies that the caller should not block.  
		  Instead, the caller will try once to acquire the lock and return 
		  immediately,
	    - TIMEOUT_INFINITE : Specifies that the caller should block indefinitely 
		  while waiting for the lock. To maintain consistency with 
		  the CMOOSLock class, this has been made the default behavior when 
		  locking.

	@return
		true if the lock was successfully acquired; else false
	*/
	bool Lock(int TimeoutMs = TIMEOUT_INFINITE);


	//============================================================================
	/** Releases the lock, marking the next thread blocking on the mutex as ready
		to run
	*/
    void UnLock();

	#ifdef PLATFORM_DARWIN
    // The MacOS implementation of a Semaphore needs direct access to
    // the mutex structure of a lock it uses...
    friend class Semaphore;
	#endif

protected:
	#ifdef _WIN32
    /// Win32 handle to locked object
    HANDLE            m_hLock;
	#else
    /// posix mutex
    pthread_mutex_t    m_hLock;
	#endif

private:
	// Prevent automatic generation of copy constructor and assignment operator 
	TimedLock (const TimedLock&);
    const TimedLock& operator= (const TimedLock&);
};

};	// END namespace YellowSubUtils

#endif // END #ifndef _TIMEDLOCK_H_
