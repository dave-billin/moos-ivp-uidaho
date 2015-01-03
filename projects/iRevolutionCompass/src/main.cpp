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
/** @file iRevolutionCompass/main.cpp
 *
 * @brief
 *  Execution entry point for the iRevolutionCompass application
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================
#include <iostream>
#include "MOOS/libMOOS/MOOSLib.h"
#include "iRevolutionCompass.h"
#include "AppVersion.h"

using std::cout;
using std::endl;

/** @defgroup iRevolutionCompass
 * @{
 */

//================================
// Function Prototypes
//================================
void PrintUsageInfo(void);
void PrintExampleConfig(void);


//=============================================================================
/** Creates an instance of the application and runs it */
int main(int argc, char* argv[])
{
	const char* szMissionFile = NULL;
	const char* szMOOSName = "iRevolutionCompass";

	// Print usage info and exit if no command line arguments were supplied
	if (argc == 1)
	{
		cout << "\nUSAGE:   iRevolutionCompass [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, use 'iRevolutionCompass --help' or 'man "
                "iRevolutionCompass'\n"
             << endl;

		return 0;
	}

	// Parse command line arguments for options or mission file
	if (argc >= 2)
	{
        //--------------------------------
		// Parse command-line switches
        //--------------------------------
		if ( MOOSStrCmp(argv[1], "-e") || MOOSStrCmp(argv[1], "--example") )
		{
            PrintExampleConfig();
			return 0;
		}
		else if ( MOOSStrCmp(argv[1], "-h") || MOOSStrCmp(argv[1], "--help") )
		{
            // Print usage info
			PrintUsageInfo();
			return 0;
		}
        else if ( MOOSStrCmp(argv[1], "-v") || MOOSStrCmp(argv[1], "--version") )
        {
            // Print application version
            cout << "iRevolutionCompass v" << APP_VERSION_STRING << "\n" 
                 << endl;
            return 0;
        }
		else	// No command line options, must be a mission file path
		{
			szMissionFile = argv[1];
		}
	}

	// Grab alternate name to use when registering if specified
	if (argc >= 3)
	{
		szMOOSName = argv[2];
	}

	// Create and launch the application
	iRevolutionCompass AppObject;
	AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

	return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
	cout <<
    "\niRevolutionCompass v" << APP_VERSION_STRING << "\n"
    "Written by Dave Billin (david.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   iRevolutionCompass [OPTIONS] MISSION_FILE [ALTNAME]\n"
    "\n"
    "OPTIONS\n"
    "   -e,--example  Prints an example mission file configuration block\n"
    "   -h,--help     Prints application usage information\n"
    "   -v,--version  Displays application version\n"
    "\n"
    "MISSION_FILE\n"
    "   MOOS mission file containing a configuration block\n"
    "\n"
    "ALTNAME\n"
    "   Optional name to use when registering with the MOOS database\n"
    "\n"
    "For additional help, type \"man iRevolutionCompass\"\n"
    "\n" 
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    // Print an example mission file configuration
    cout << "\n\nExample Mission File Configuration Block:\n" 
         << iRevolutionCompass::ExampleMissionFileConfiguration 
         << "\n" << endl;
}


/** @} */   // END @defgroup iRevolutionCompass

