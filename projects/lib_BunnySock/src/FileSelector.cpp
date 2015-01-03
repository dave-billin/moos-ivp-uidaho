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
/** @file FileSelector.cpp
 *
 * @brief
 *  Implementation of the FileSelector class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================

#include "FileSelector.h"

#ifdef _WIN32
	#include <Winsock2.h>
#else
	#include <sys/select.h>		/* According to POSIX.1-2001 */
	#include <sys/time.h>
	#include <sys/types.h>
#endif


//=============================================================================
FileSelector::FileSelector( int Target, int ReadWriteFlags )
{
	SetFile(Target, ReadWriteFlags );
	m_ReadWriteStatus = 0;
}


//=============================================================================
FileSelector::~FileSelector()
{
}



//=============================================================================
void FileSelector::SetFile(int Target, int ReadWriteFlags)
{
	m_TargetFd = Target;
	m_ReadWriteStatus = 0;
	m_ReadWriteEnable = ReadWriteFlags;
}



//=============================================================================
bool FileSelector::FileIsReadable(void) const
{
	return m_ReadWriteStatus & FileSelector::Readable;
}



//=============================================================================
bool FileSelector::FileIsWritable(void) const
{
	return (m_ReadWriteStatus & FileSelector::Writable) ? true : false;
}



//=============================================================================
void FileSelector::RemoveFile(int ReadWriteFlags)
{
	m_ReadWriteEnable &= ~ReadWriteFlags;	// Clear the specified flags
	m_ReadWriteStatus &= ~ReadWriteFlags;	// Clear associated status

	// If neither reading nor writing is enabled, clear the file descriptor
	m_TargetFd = (m_ReadWriteEnable == 0) ? 0 : m_TargetFd;
}



//=============================================================================
int FileSelector::WaitForReadiness(int ReadWriteFlags, long TimeOut_ms)
{
	struct timeval m_TimeOutVal;
	//{
	//    long    tv_sec;         /* seconds */
	//    long    tv_usec;        /* microseconds */
	//};


	fd_set ReadableFdSet;
	fd_set WritableFdSet;
	int Rc;

	if (m_TargetFd == 0)
	{
		return -1;		// Return error for an invalid file descriptor
	}

	// Zero out the file descriptor sets
	FD_ZERO(&ReadableFdSet);
	FD_ZERO(&WritableFdSet);

	// Set up fd_sets to monitor
	if (m_ReadWriteEnable & FileSelector::Readable)
	{
		FD_SET(m_TargetFd, &ReadableFdSet);
	}
	if (m_ReadWriteEnable & FileSelector::Writable)
	{
		FD_SET(m_TargetFd, &WritableFdSet);
	}

	// Set up timeout
	m_TimeOutVal.tv_sec = (TimeOut_ms > 1000) ? TimeOut_ms / 1000 : 0;
	m_TimeOutVal.tv_usec = (TimeOut_ms % 1000) * 1000;

	// Select on the file descriptor
	// NOTE: the first parameter should be the max value file descriptor PLUS ONE!
	Rc = select(m_TargetFd+1, &ReadableFdSet, &WritableFdSet, 0, &m_TimeOutVal);

	// Catch errors
	if (Rc == -1)
	{
		return -2;
	}

	m_ReadWriteStatus = 0;
	if ( FD_ISSET(m_TargetFd, &ReadableFdSet) )
	{
		m_ReadWriteStatus |= FileSelector::Readable;
	}
	if ( FD_ISSET(m_TargetFd, &WritableFdSet) )
	{
		m_ReadWriteStatus |= FileSelector::Writable;
	}

	m_ReadWriteStatus |= ( FD_ISSET(m_TargetFd, &ReadableFdSet) )
								? FileSelector::Readable : 0;

	m_ReadWriteStatus |= ( FD_ISSET(m_TargetFd, &WritableFdSet) )
								? FileSelector::Writable : 0;

	return m_ReadWriteStatus;
}






