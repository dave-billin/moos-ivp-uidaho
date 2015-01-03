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
/** @file Semaphore.h
 *
 * @brief
 * 	Declaration of a counting semaphore class for inter-process communication
 * 	
 * @author Dave Billin
 */
//=============================================================================

#ifndef _SEMAPHORE_H_
#define _SEMAPHORE_H_

#include <stdint.h>

#ifdef _WIN32
	#include <windows.h>
#endif

#ifdef PLATFORM_DARWIN
	#include <pthread.h>
	#include "TimedLock.h"
#endif

#ifdef UNIX
	#include <semaphore.h>
#endif


#ifndef PLATFORM_DARWIN	// Semaphores are disabled in OS X for now...


/** @def ENABLE_SEMDESTROY_ASSERTION
 * @brief
 * 	Defining this symbol enables an assertion to be triggered if a Semaphore
 * 	object with a non-zero count is destroyed
 */
#define ENABLE_SEMDESTROY_ASSERTION


namespace YellowSubUtils
{

//=============================================================================
/** A cross-platform semaphore class that can be used for inter-thread
 *	synchronization.  The implementation of this semaphore object is that of a 
 *  typical counting semaphore.
*/
class Semaphore
{
public:

	/** Creates an instance of the object
     * @param InitialCount
	 *  The initial value assigned to the semaphore (default is zero)
	 */
	Semaphore( uint32_t InitialCount = 0 );


	/** Called when the semaphore object goes out of scope */
	~Semaphore(void);


	//=========================================================================
	/** Blocking attempt to acquire the semaphore
     *
	 * @details
	 *  This function atomically decrements the semaphore's count.  If the 
	 *  resulting semaphore count is greater than zero, the function returns 
	 *  true immediately.  Otherwise, the caller's thread is blocked until the
	 *  semaphore's count becomes positive.
	 *  
	 * @param TimeoutMilliseconds
	 *  - NO_WAIT to return immediately (essentially the same effect as calling
	 *    TryWait() )
	 *  - A maximum number of milliseconds to wait for the semaphore before 
	 *    timing out and returning false
	 *  - WAIT_FOREVER to block indefinitely
	 *  
	 * @return
	 *  true if the semaphore was acquired; else false if the semaphore was
	 *  already locked
	 */
	bool Wait( int32_t TimeoutMilliseconds );


	//=========================================================================
	/** Non-blocking attempt to acquire the semaphore
     * 
	 * @details
	 *  This function makes a single attempt to acquire the semaphore.  If the 
	 *  semaphore's count is greater than zero, it returns true.  Otherwise, 
	 *  the count remains unaffected and the function immediately returns 
	 *  false.
	 *  
	 * @return
	 *  - true if the semaphore was acquired (i.e. decremented)
	 *  - false if the semaphore is already locked
	 */
	bool TryWait( void );


	//=========================================================================
	/** Atomically increments the semaphore's count.
	 * @details
	 *  This function increments the count of the semaphore.  If the resulting
	 *  count is positive, pending thread(s) will be allowed to run.
	 */
	void Post( void );


	//=========================================================================
	/** Resets the semaphore's count to a specified value (default is zero) */
	void Reset( int32_t Count = 0);


	//=========================================================================
	/** Returns the semaphore's current count */
	int GetCount( void );



	/** Constants for specifying exceptional timeout values */
	enum e_TimeoutConstants
	{
		WAIT_FOREVER = -1,	/**< Wait indefinitely */
		NO_WAIT = 0		    /**< Return immediately (no timeout) */
	};

private:

	// Prevent automatic generation of copy constructor and assignment operator 
	Semaphore (const Semaphore&);
    const Semaphore& operator= (const Semaphore&);

#ifdef _WIN32
	HANDLE m_Semaphore;
#endif

#ifdef DARWIN
	pthread_cond_t m_SemCondition;
	TimedLock m_SemLock;
	int m_SemCount;
	int m_NumPendingThreads;
#endif

#ifdef UNIX
	sem_t m_Semaphore;
#endif

};

};  // END namespace YellowSubUtils


#endif	// END #ifndef PLATFORM_DARWIN

#endif	// END #ifndef _SEMAPHORE_H_

