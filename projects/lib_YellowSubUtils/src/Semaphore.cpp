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
/** @file Semaphore.cpp
 *
 * @brief
 * 	Implementation of the Semaphore class declared in Semaphore.h
 *
 * @author Dave Billin
 *
 * @par Created for
 * 	The Microcomputer Research and Communications Institute (MRCI) - at the
 * 	University of Idaho, USA.
 */
//=============================================================================

#ifndef PLATFORM_DARWIN

#include <string>
#include <exception>
#include <assert.h>
#include "Semaphore.h"

using std::string;
using std::exception;
using YellowSubUtils::Semaphore;


/** Exception class thrown on a Semaphore error */
class SemaphoreException : public exception
{
public:
	SemaphoreException( const char* szDescription ) { m_What = szDescription; }
	virtual ~SemaphoreException() throw() {;}

	const char* What( void ) const throw() { return m_What.c_str(); }

private:
	string m_What;
};



#ifdef _WIN32
//===========================================
//===========================================
//	*** WIN32 SEMAPHORE IMPLEMENTATION ***
//===========================================
//===========================================


//=============================================================================
Semaphore::Semaphore(unsigned char InitialCount)
{
	m_Semaphore = CreateSemaphore( 0, /* Pointer to security attributes */
								   InitialCount, /* Initial count */
								   LONG_MAX, /* Maximum count */
								   0);	/* Pointer to semaphore name */
}


//=============================================================================
Semaphore::~Semaphore()
{
	CloseHandle(m_Semaphore);
}


//=============================================================================
bool Semaphore::Wait( int TimeoutMilliseconds )
{
	DWORD msWait = (TimeoutMilliseconds == TIMEOUT_FOREVER) 
						? INFINITE : TimeoutMilliseconds;
	
	return WaitForSingleObject( m_Semaphore, /* Semaphore handle */
								msWait)		/* Timeout (ms) */
								== WAIT_OBJECT_0;
}


//=============================================================================
bool Semaphore::TryWait( void )
{
	return (WaitForSingleObject( m_Semaphore, 0) == WAIT_OBJECT_0 );
}


//=============================================================================
void Semaphore::Post( void )
{
	ReleaseSemaphore( m_Semaphore /* Semaphore handle */, 
					  1, /* Value to increment semaphore by */
					  0); /* Pointer used to return previous value */
}


//=============================================================================
void Semaphore::Reset( int Count )
{
	CloseHandle(m_Semaphore);
	m_Semaphore = CreateSemaphore( 0, /* Pointer to security attributes */
								   Count,		  /* Initial count */
								   LONG_MAX, /* Maximum count */
								   0);	/* Pointer to semaphore name */
}


//=============================================================================
int Semaphore::GetCount( void )
{
	LONG Count = -1;
	ReleaseSemaphore( m_Semaphore /* Semaphore handle */, 
					  0, /* Value to increment semaphore by */
					  &Count); /* Pointer used to return previous value */
	return Count;
}

#endif


#ifndef DARWIN
#ifdef UNIX
//===========================================
//===========================================
//	*** UNIX/Linux IMPLEMENTATION ***
//===========================================
//===========================================

#include <sys/time.h>

//=============================================================================
Semaphore::Semaphore(uint32_t InitialCount)
{
	sem_init( &m_Semaphore, 0, InitialCount);
}


//=============================================================================
Semaphore::~Semaphore()
{

	#ifdef ENABLE_SEMDESTROY_ASSERTION
	assert( (GetCount() != 0) );

	// DANGER! A locked semaphore is being destroyed!
	#endif // END #ifdef ENABLE_SEMAPHORE_WARNINGS

	sem_destroy(&m_Semaphore);
}


//=============================================================================
bool Semaphore::Wait( int TimeoutMilliseconds )
{
	if (TimeoutMilliseconds == NO_WAIT)
	{
		return (sem_trywait(&m_Semaphore) == 0);
	}
	else if (TimeoutMilliseconds == WAIT_FOREVER)
	{
		return sem_wait(&m_Semaphore) == 0;
	}
	else
	{
		struct timespec AbsoluteTime;

		// Try to get the current time from the hi-res system clock
		//MOOSAssert( (clock_gettime(CLOCK_REALTIME, &AbsoluteTime)), /* expr */
		//			"clock_gettime() failed to return CLOCK_REALTIME",
		//			"Semaphore.cpp",
		//			135 );
		
		//clock_gettime(CLOCK_REALTIME, &AbsoluteTime);
		
		// Add the specified timeout to get the absolute time of the timeout
		//AbsoluteTime.tv_nsec += (TimeoutMilliseconds * 1000);
		return (sem_timedwait(&m_Semaphore, &AbsoluteTime) == 0);
	}
}


//=============================================================================
bool Semaphore::TryWait( void )
{
	return (sem_trywait(&m_Semaphore) == 0);
}


//=============================================================================
void Semaphore::Post( void )
{
	sem_post(&m_Semaphore);
}


//=============================================================================
void Semaphore::Reset( int InitialValue )
{
	/*
	#ifdef ENABLE_SEMDESTROY_WARNINGS
	MOOSAssert( (GetCount() == 0), 
				"WARNING: a locked semaphore is being destroyed!",
				"Semaphore.cpp",
				175 );
	#endif // END #ifdef ENABLE_SEMAPHORE_WARNINGS
	*/

	sem_destroy(&m_Semaphore);
	sem_init(&m_Semaphore, 0, InitialValue);
}


//=============================================================================
int Semaphore::GetCount( void )
{
	int Count = -1;
	sem_getvalue(&m_Semaphore, &Count);
	return Count;
}

#endif
#endif




#ifdef DARWIN
//===========================================
//===========================================
//	*** Mac OS X IMPLEMENTATION ***
//===========================================
//===========================================

#include <sys/time.h>


//=============================================================================
Semaphore::Semaphore(uint32_t InitialCount)
: m_SemCount(InitialCount),
  m_NumPendingThreads(0),
  m_SemCondition(PTHREAD_COND_INITIALIZER),
  m_SemLock(false)
{

}


//=============================================================================
Semaphore::~Semaphore()
{
	#ifdef ENABLE_SEMDESTROY_ASSERTION
	assert( m_SemCount != 0 );
	// DANGER! A locked semaphore is being destroyed!
	#endif // END #ifdef ENABLE_SEMAPHORE_WARNINGS

	pthread_cond_destroy(&m_SemCondition);
}


//=============================================================================
bool Semaphore::Wait( int TimeoutMilliseconds )
{
	if ( m_SemLock.Lock(TimeoutMilliseconds) )
	{
		// Make sure the semaphore count never goes negative
		m_SemCount--;

		if (m_SemCount < 0)
		{
			do
			{
				// Absolute time when timeout expires
				struct timespec AbsoluteTime;

				// Get the current time from the hi-res system clock
				clock_gettime(CLOCK_REALTIME, &AbsoluteTime);
				AbsoluteTime.tv_nsec += (TimeoutMilliseconds * 1000);

				if ( pthread_cond_timedwait(&m_SemCondition,
									   	    &m_SemLock.m_hLock,
									   	    &AbsoluteTime) == ETIMEDOUT )
				{
					// Return false if the specified timeout elapses
					return false;
				}

			} while ( m_NumPendingThreads < 1);
			m_NumPendingThreads--;
		}

		m_SemLock.UnLock();
		return true;
	}

	return false;
}


//=============================================================================
bool Semaphore::TryWait( void )
{
	bool Rc = false;

	// Lock the semaphore
	if ( m_SemLock.Lock(TimedLock::TIMEOUT_IMMEDIATE) )
	{
		Rc = (m_SemCount == 0);
		m_SemLock.UnLock();
	}

	return Rc;
}


//=============================================================================
void Semaphore::Post( void )
{
	sem_post(&m_Semaphore);
}


//=============================================================================
void Semaphore::Reset( int InitialValue )
{
	/*
	#ifdef ENABLE_SEMDESTROY_WARNINGS
	MOOSAssert( (GetCount() == 0),
				"WARNING: a locked semaphore is being destroyed!",
				"Semaphore.cpp",
				175 );
	#endif // END #ifdef ENABLE_SEMAPHORE_WARNINGS
	*/

	sem_destroy(&m_Semaphore);
	sem_init(&m_Semaphore, 0, InitialValue);
}


//=============================================================================
int Semaphore::GetCount( void )
{
	int Count = -1;
	sem_getvalue(&m_Semaphore, &Count);
	return Count;
}




#endif


#endif // END #ifndef PLATFORM_DARWIN
