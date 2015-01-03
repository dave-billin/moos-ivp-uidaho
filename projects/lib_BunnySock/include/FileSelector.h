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
/** @file FileSelector.h
 *
 * @brief
 *  Declaration of the FileSelector class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================

#ifndef FILESELECTOR_H_
#define FILESELECTOR_H_


/** An object to wrap the functionality of select() and to wait or provide
 *	notification when a FILE becomes readable or writable.
 *
 * @note
 * 	The implementation used here does NOT include any mechanism for
 *  synchronization, so be use caution when modifying the file associated with
 *  a FileSelector object.
 */
class FileSelector
{
public:

	/** @enum e_ReadWriteFlags
	 * @brief
	 * 	Flags used in conjunction with
	 */
	enum e_ReadWriteFlags
	{
		Readable = 1, /**< Add to or remove from the 'readable' set */
		Writable = 2  /**< Add to or remove from the 'writable' set */
	};


	//=========================================================================
	/** Creates a FileSelector object
	 * @param Target
	 * 	Descriptor of a file to associate with the object (default is no
	 *  file association)
	 *
	 * @param ReadWriteFlags
	 *	Flags from e_ReadWriteFlags specifying which type of monitoring to
	 *  perform on the file (Read-ability, Write-ability, both) (default is
	 *  no monitoring)
	 */
	FileSelector( int Target = 0, int ReadWriteFlags = 0 );

	/** Called when a FileSelector object goes out of scope */
	virtual	~FileSelector();


	//=========================================================================
	/** Assign the file to be monitored by the FileSelector object for read-
	 *  and write-ability
	 *
	 * @param Target
	 * 	Descriptor of the file to associate with
	 *
	 * @param ReadWriteFlags
	 * 	Flags from e_ReadWriteFlags specifying which type of monitoring to
	 *  perform on the file (Read-ability, Write-ability, both)
	 */
	void SetFile(int Target, int ReadWriteFlags );

	/** Returns the file descriptor being monitored or zero if no file is
	 *  associated with the object
	 */
	int GetFile( void ) const	{ return m_TargetFd; }


	//=========================================================================
	/** Ceases monitoring of the current file descriptor for read- and/or
	 *  write-ability
	 *
	 * @param ReadWriteFlags
	 * 	Flags from e_ReadWriteFlags specifying which type of monitoring to
	 *  cease: read-ability, write-ability, or both
	 */
	void RemoveFile(int ReadWriteFlags);


	/** Returns true if the file is writable */
	bool FileIsWritable( void ) const;

	/** Returns true if the file is readable */
	bool FileIsReadable( void ) const;



	//=========================================================================
	/** Blocks until the monitored set(s) are ready for reading/writing or a
	 *  timeout elapses
	 *
	 * @param ReadWriteFlags
	 * 	Flags from e_ReadWriteFlags specifying which monitored sets to wait on
	 *  (Readable, Writable, both)
	 *
	 * @param TimeOut_ms
	 * 	Maximum time (in milliseconds) to wait for the file to become readable
	 *  and/or writable
	 *
	 * @return
	 * 	- A positive value containing one or more flags from e_ReadWriteFlags
	 *    if the file is ready for reading/writing
	 *  - 0 if the specified timeout elapsed
	 *  - A negative value on error
	 */
	int WaitForReadiness( int ReadWriteFlags, long TimeOut_ms);

private:
	int m_TargetFd;			/**< file descriptor to monitor */
	int m_ReadWriteEnable;	/**< Flags indicating monitoring type */
	int m_ReadWriteStatus;	/**< Flags indicating if the file is read-able
								 and/or write-able */
};

#endif /* FILESELECTOR_H_ */
