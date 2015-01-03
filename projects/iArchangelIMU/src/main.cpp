//=============================================================================
/** @file iArchangelIMU/main.cpp
 *
 * @brief
 *  Execution entry point for the iArchangelIMU application
 *
 * @author Dave Billin
 *
 */
//=============================================================================
#include <iostream>
#include "MOOS/libMOOS/MOOSLib.h"
#include "iArchangelIMU.h"

using std::cout;
using std::endl;

/** @defgroup iArchangelIMU
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
	const char* szMOOSName = "iArchangelIMU";

	// Print usage info and exit if no command line arguments were supplied
	if (argc == 1)
	{
		cout << "\nUSAGE:   iArchangelIMU [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'iArchangelIMU --help' or 'man "
                "iArchangelIMU'\n"
             << endl;

		return 0;
	}

	// Parse command line arguments for options or mission file
	if (argc >= 2)
	{
		// Parse for option switches
		if ( MOOSStrCmp(argv[1], "-e") || MOOSStrCmp(argv[1], "--example") )
		{
			PrintExampleConfig();
			return 0;
		}
		else if ( MOOSStrCmp(argv[1], "-h") || MOOSStrCmp(argv[1], "--help") )
		{
			PrintUsageInfo();
			return 0;
		}
        else if ( MOOSStrCmp(argv[1], "-v") || MOOSStrCmp(argv[1], "--version") )
        {
            cout << "\niArchangelIMU version \n" << endl;
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
	iArchangelIMU AppObject;
	AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

	return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
	cout <<
    "\niArchangelIMU version \n"
    "Written by Dave Billin (dave.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   iArchangelIMU [OPTIONS] MISSION_FILE [APPNAME]\n"
    "\n"
    "OPTIONS\n"
    "   -e,--example  Prints an example mission file configuration block\n"
    "   -h,--help     Prints application usage information\n"
    "   -v,--version  Displays application version\n"
    "\n"
    "MISSION_FILE\n"
    "   MOOS mission file containing a configuration block\n"
    "\n"
    "APPNAME\n"
    "   Optional name to use when registering with the MOOS database\n"
    "\n"
    "For additional help, type \"man iArchangelIMU\"\n"
    "\n" 
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "ProcessConfig = iArchangelIMU\n"
    "{\n"
    "    AppTick   = 100    // Frequency at which Iterate() runs\n"
    "    CommsTick = 100    // Frequency of MOOSDB comms\n"
    "\n"
    "    Port      = /dev/ttyS0\n"
    "\n"
    "    VERBOSITY = 0      // Verbosity of debugging messages\n"
    "}\n"
    "\n" 
    << endl;
}


/** @} */   // END @defgroup iArchangelIMU

