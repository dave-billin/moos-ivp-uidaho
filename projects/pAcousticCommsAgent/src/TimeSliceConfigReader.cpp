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
/** @file TimeSliceConfigReader.cpp
 *
 * @brief	Declaration of a class used to read time slice configuration files
 *
 * @author Dave Billin
 */
//=============================================================================
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <MBUtils.h>
#include "TimeSliceConfigReader.h"


using namespace std;


//=============================================================================
TimeSliceConfigReader::TimeSliceConfigReader( void )
{
}



//=============================================================================
TimeSliceConfigReader::~TimeSliceConfigReader()
{
}




//=============================================================================
list<TimeSlice> TimeSliceConfigReader::ParseFile( const string& sFilePath,
							   	   	   	   	      InfoBuffer& TargetInfoBuffer,
							   	   	   	   	      CMOOSCommClient& CommClient )
{
	vector<string> vBlockStrings;
	list<TimeSlice> TimeSliceList;
	ifstream Fin(sFilePath.c_str(), ifstream::in);

	// Validate input file
	if (!Fin.good())
	{
		throw CMOOSException("Failed to open " + sFilePath);
	}

	if (ReadConfigFileBlocks(Fin, vBlockStrings))
	{
		int i = 1;
		// Parse all configuration blocks
		for (vector<string>::iterator iter = vBlockStrings.begin();
			 iter != vBlockStrings.end(); iter++)
		{
			try
			{
				TimeSliceList.push_back( ParseConfigBlock( *iter,
														   TargetInfoBuffer,
														   CommClient) );
			}
			catch ( CMOOSException& e )
			{
				CMOOSException ParseException(
					MOOSFormat("Syntax error in time slice block %d: ", i) +
					e.m_sReason);

				throw ParseException;
			}
			i++;
		}
	}
	else
	{
		CMOOSException BraceException( "Syntax error in time slice block: "
									   "one or more unmatched braces");
		throw BraceException;
	}

	// Sort the TimeSlice objects by Order
	if (!TimeSliceList.empty())
	{
		TimeSliceList.sort();
	}

	return TimeSliceList;
}




//=============================================================================
bool TimeSliceConfigReader::ReadConfigFileBlocks( ifstream& Fin,
												  vector<string>& vBlockStrings)
{
	char DelimChar = '{';
	string sBlock;

	vBlockStrings.clear();

	while (Fin.good())
	{
		// Read a block of text from the file
		std::getline(Fin, sBlock, DelimChar);

		// Read characters preceding an open-brace
		if (DelimChar == '{')
		{
			sBlock.clear();
			DelimChar = '}';
		}
		else	// Read characters preceding a close-brace
		{
			// Did the file end before a close-brace was found?
			if (Fin.fail() && Fin.eof())
			{
				// Return false to indicate brace mismatch
				return false;
			}
			else
			{
				vBlockStrings.push_back(sBlock);
			}
			DelimChar = '{';
		}
	}

	return true;
}



//=============================================================================
TimeSlice TimeSliceConfigReader::ParseConfigBlock(string& sConfigBlock,
												  InfoBuffer& TargetInfoBuffer,
												  CMOOSCommClient& CommClient )
{
	vector<string> vBlockLines;
	string sVal, sLine;
	double DVal;
	int IntVal;
	bool GotName = false;
	bool GotOrder = false;
	bool GotDuration = false;
	TimeSlice Slice(TargetInfoBuffer, CommClient);


	//-------------------------------------------------
	// Parse each line of the configuration block
	//-------------------------------------------------
	vBlockLines = parseString(sConfigBlock, '\n');
	for ( vector<string>::iterator iter = vBlockLines.begin();
		  iter != vBlockLines.end(); iter++ )
	{
		sLine = *iter;
		MOOSTrimWhiteSpace(sLine);

		// Remove commented portions of the line
		sLine = stripComment(sLine, string("//"));

		// Remove blanks
		sLine = stripBlankEnds(sLine);

		// Ignore empty lines
		if ( sLine.empty() )
		{
			continue;
		}

		//-----------------------------
		// Read time slice parameters
		//-----------------------------
		if ( MOOSValFromString(sVal, sLine, "NAME", true) )
		{
			GotName = true;
			MOOSTrimWhiteSpace(sVal);
			Slice.SetName(sVal);
		}
		else if ( MOOSValFromString(IntVal, sLine, "ORDER", true) )
		{
			if (IntVal < 0)
			{
				MOOSTrace("WARNING: ignored negative ORDER value in time "
						  "slice configuration\n");
			}
			else
			{
				GotOrder = true;
				Slice.SetOrder(IntVal);
			}
		}
		else if ( MOOSValFromString(DVal, sLine, "DURATION", true) )
		{
			if (DVal < 0.0)
			{
				MOOSTrace("WARNING: ignored negative DURATION value in time "
						  "slice configuration\n");
			}
			else
			{
				GotDuration = true;
				Slice.SetDuration(DVal);
			}
		}
		else if ( MOOSValFromString(sVal, sLine, "CONDITION", true) )
		{
			MOOSTrimWhiteSpace(sVal);
			if (!sVal.empty())
			{
				Slice.SetCondition(sVal);
			}
		}
		else if ( strBegins(sLine, "ALWAYSPUBLISH", true) )
		{
			MOOSChomp(sLine, "=", true);
			string sVarName = MOOSChomp(sLine, "=");
			MOOSTrimWhiteSpace(sVarName);
			MOOSTrimWhiteSpace(sLine);
			if ( !sVarName.empty() && !sLine.empty() )
			{
				Slice.AddToPublishItems( VarDataPair(sVarName, sLine, "auto"),
										 true );
			}
		}
		else if ( strBegins(sLine, "PUBLISH", true) )
		{
			MOOSChomp(sLine, "=", true);
			string sVarName = MOOSChomp(sLine, "=");
			MOOSTrimWhiteSpace(sVarName);
			MOOSTrimWhiteSpace(sLine);
			if ( !sVarName.empty() && !sLine.empty() )
			{
				Slice.AddToPublishItems( VarDataPair(sVarName, sLine, "auto"),
										 false );
			}
		}

	}	// END for ( vector<string>::iterator iter = vBlockLines.begin()...

	if (!GotName)
	{
		throw CMOOSException("missing NAME parameter");
	}
	if (!GotOrder)
	{
		throw CMOOSException("missing ORDER parameter");
	}
	if (!GotDuration)
	{
		throw CMOOSException("missing DURATION parameter");
	}
	if (!GotName)
	{
		throw CMOOSException("missing CONDITION parameter");
	}
	return Slice;
}
