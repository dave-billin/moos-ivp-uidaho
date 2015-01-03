//=============================================================================
/*  iGPSd - a MOOS-IvP application
    Copyright (C) 2012  Dave Billin

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
/** @file iGPSd/main.cpp
 *
 * @brief
 *  Execution entry point for the iGPSd application
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================
#include <iostream>

#include "MOOS/libMOOS/MOOSLib.h"
#include "iGPSd.h"
#include "AppVersion.h"

#ifdef USE_GOOGLE_PROTOCOL_BUFFERS
#include "iGPSd_FixData.pb.h"
#endif

using std::cout;
using std::endl;

/** @defgroup iGPSd
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
	const char* szMOOSName = "iGPSd";

	// Print usage info and exit if no command line arguments were supplied
	if (argc == 1)
	{
		cout << "\nUSAGE:   iGPSd [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, use 'iGPSd --help' or 'man "
                "iGPSd'\n"
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
            cout << "iGPSd v" << APP_VERSION_STRING << "\n" 
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

	// Verify the version of the Google Protocol buffers library
	// linked against is compatible with the version the app was
	// compiled against.  If not, the process will abort
    #ifdef USE_GOOGLE_PROTOCOL_BUFFERS
	GOOGLE_PROTOBUF_VERIFY_VERSION;
    #endif

	// Create and launch the application
	iGPSd AppObject;
	AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    #ifdef USE_GOOGLE_PROTOCOL_BUFFERS
	// Delete all global objects allocated by libprotobuf.
	google::protobuf::ShutdownProtobufLibrary();
    #endif

	return 0;
}





//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
	cout <<
    "\niGPSd v" << APP_VERSION_STRING << "\n"
    "Written by Dave Billin (david.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   iGPSd [OPTIONS] MISSION_FILE [ALTNAME]\n"
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
    "For additional help, type \"man iGPSd\"\n"
    "\n" 
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    // Print an example mission file configuration
    cout << "\n\nExample Mission File Configuration Block:\n" 
         << iGPSd::ExampleMissionFileConfiguration 
         << "\n" << endl;
}


/** @} */   // END @defgroup iGPSd

