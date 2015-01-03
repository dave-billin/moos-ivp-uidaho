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
/** @file MonitorConfigReader.h
 *
 * @brief
 * 	Declaration of a class used to read MonitorTarget configuration
 *	files.
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
#include "MonitorTarget.h"


class MonitorConfigReader
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	MonitorConfigReader( void );

	/** Called when the object goes out of scope */
	~MonitorConfigReader();


	//=========================================================================
	/** Opens and parses a specified file
	 *
	 * @param sFilePath
	 *	Absolute path of the file to open and parse
	 *
	 * @return
	 * 	An sorted list of Timeslice objects created from specifications in
	 * 	the file.  MonitorTarget objects are sorted by Order value in ascending
	 * 	order (lowest Order values occur first in the list; highest occur last
	 * 	in the list)
	 *
	 * @throw
	 * 	A CMOOSException object if the specified file could not be opened, or
	 * 	if a parsing error occurs.
	 */
	std::vector<MonitorTarget> ParseFile( const std::string& sFilePath );

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
	 * @return
	 * 	A MonitorTarget object created using parameters in the configuration block
	 *
	 * @throw
	 * 	A CMOOSException object if a syntax error is detected in the block or
	 * 	if the specified file could not be read
	 */
	MonitorTarget ParseConfigBlock( std::string& sConfigBlock );

};

#endif	// END #ifndef _TIMESLICECONFIGREADER_H_
