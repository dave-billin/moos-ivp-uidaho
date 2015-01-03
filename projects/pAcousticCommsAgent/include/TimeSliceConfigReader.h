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
/** @file TimeSliceConfigReader.h
 *
 * @brief	Declaration of a class used to read time slice configuration files
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef _TIMESLICECONFIGREADER_H_
#define _TIMESLICECONFIGREADER_H_

#include <list>
#include <vector>
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "InfoBuffer.h"
#include "TimeSlice.h"


class TimeSliceConfigReader
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	TimeSliceConfigReader( void );

	/** Called when the object goes out of scope */
	~TimeSliceConfigReader();


	//=========================================================================
	/** Opens and parses a specified file
	 *
	 * @param sFilePath
	 *	Absolute path of the file to open and parse
	 *
	 * @param TargetInfoBuffer
	 * 	Reference to an InfoBuffer object to be associated with TimeSlice
	 * 	objects constructed from specifications in the file.
	 *
	 * @param CommClient
	 *	Reference to the MOOS Comms client object to associate with time slice
	 *	objects constructed from specifications in the file
	 *
	 * @return
	 * 	An sorted list of Timeslice objects created from specifications in
	 * 	the file.  TimeSlice objects are sorted by Order value in ascending
	 * 	order (lowest Order values occur first in the list; highest occur last
	 * 	in the list)
	 *
	 * @throw
	 * 	A CMOOSException object if the specified file could not be opened, or
	 * 	if a parsing error occurs.
	 */
	std::list<TimeSlice> ParseFile( const std::string& sFilePath,
							        InfoBuffer& TargetInfoBuffer,
							        CMOOSCommClient& CommClient );

private:


	//=========================================================================
	/** Extracts text in a file occuring between open- and close-brace
	 * characters into a vector of strings
	 *
	 * @param Fin
	 *	Input file stream to read from
	 *
	 * @param vBlockStrings
	 * 	Reference to a vector of strings to populate with configuration block
	 * 	contents.  Each element is populated with a string containing the
	 * 	entire contents of a single configuration block.
	 *
	 * @return
	 * 	true if one or more strings were returned in vBlockStrings; else
	 * 	false if Fin is no good or a parsing error occurs.
	 */
	bool ReadConfigFileBlocks( std::ifstream& Fin,
	                           std::vector<std::string>& vBlockStrings);


	//=========================================================================
	/** Parses text in a time slice configuration block
	 *
	 * @param sConfigBlock
	 * 	The text inside a time slice configuration block (not including the
	 * 	open- and close-brace characters used to delimit the block
	 *
	 * @param TargetInfoBuffer
	 * 	Reference to an InfoBuffer object to be associated with TimeSlice
	 * 	objects constructed from specifications in the file.
	 *
	 * @param CommClient
	 *	Reference to the MOOS Comms client object to associate with time slice
	 *	objects constructed from specifications in the file
	 *
	 * @return
	 * 	A TimeSlice object created using parameters in the configuration block
	 *
	 * @throw
	 * 	A CMOOSException object if a syntax error is detected in the block or
	 * 	if the specified file could not be read
	 */
	TimeSlice ParseConfigBlock( std::string& sConfigBlock,
								InfoBuffer& TargetInfoBuffer,
								CMOOSCommClient& CommClient );

};

#endif	// END #ifndef _TIMESLICECONFIGREADER_H_
