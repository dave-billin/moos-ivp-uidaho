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
/** @file MonitorConfigReader.cpp
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
#include "MonitorConfigReader.h"

using namespace std;


//=============================================================================
MonitorConfigReader::MonitorConfigReader( void )
{
}



//=============================================================================
MonitorConfigReader::~MonitorConfigReader()
{
}




//=============================================================================
vector<MonitorTarget> MonitorConfigReader::ParseFile( const string& sFilePath)
{
	vector<string> vBlockStrings;
	vector<MonitorTarget> MonitorTargets;
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
				MonitorTargets.push_back( ParseConfigBlock(*iter) );
			}
			catch ( CMOOSException& e )
			{
				CMOOSException ParseException(
					MOOSFormat("Error in Monitor Target block %d: ", i) +
					e.m_sReason);

				throw ParseException;
			}
			i++;
		}
	}
	else
	{
		CMOOSException BraceException( "Syntax error in Monitor Target block: "
									   "one or more unmatched braces");
		throw BraceException;
	}

	return MonitorTargets;
}




//=============================================================================
bool MonitorConfigReader::ReadConfigFileBlocks( ifstream& Fin,
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
MonitorTarget MonitorConfigReader::ParseConfigBlock( string& sConfigBlock )
{

	vector<string> vBlockLines;
	string sVal, sLine;
	bool GotName = false;
	bool GotCondition = false;
	bool GotPublish = false;
	MonitorTarget MonTarget;


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
			MonTarget.SetName(sVal);
		}
		else if ( MOOSValFromString(sVal, sLine, "PRINT", true) )
		{
			MonTarget.SetPrintString(sVal);
		}
		else if ( MOOSValFromString(sVal, sLine, "CONDITION", true) )
		{
			MOOSTrimWhiteSpace(sVal);
			if (!sVal.empty())
			{
				if ( !MonTarget.AddCondition(sVal) )
				{
					MOOSTrace("Failed to add CONDITION string: " + sVal + "\n");
				}
				else
				{
					GotCondition = true;
				}
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
				VarDataPair Vdp(sVarName, sLine, "auto");
				MonTarget.AddToPublishItems( Vdp );
				GotPublish = true;
			}
		}

	}

	if (!GotName)
	{
		throw CMOOSException("Monitor config must contain a NAME section");
	}
	if (!GotCondition)
	{
		throw CMOOSException("Monitor config must contain at least one "
				 	 	 	 "CONDITION specification");
	}
	if (!GotPublish)
	{
		throw CMOOSException("Monitor config must contain at least one "
							 "PUBLISH specification");
	}

	MOOSTrace("CONDITION variables:\n");
	set<string> VarNames = MonTarget.GetMoosVariableNames();
	for (set<string>::iterator iter = VarNames.begin(); iter != VarNames.end();
		 iter++)
	{
		MOOSTrace(*iter + ", ");
	}

	MOOSTrace("\n\n");
	return MonTarget;
}
