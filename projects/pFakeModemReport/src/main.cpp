//=============================================================================
/** @file pFakeModemReport/src/main.cpp
 *
 * @brief
 *  Execution entry point for the pFakeModemReport application
 *
 * @author Brandt Pedrow
 *
 */
//=============================================================================
#include <iostream>
#include "MOOS/libMOOS/MOOSLib.h"
#include "pFakeModemReport.h"

using std::cout;
using std::endl;


/** @defgroup pFakeModemReport
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
    const char* szMOOSName = "pFakeModemReport";

    // Print usage info and exit if no command line arguments were supplied
    if (argc == 1)
    {
		cout << "\nUSAGE:   pFakeModemReport [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'pFakeModemReport --help' or 'man "
                "pFakeModemReport'\n"
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
            cout << "\npFakeModemReport version \n\n" << endl;
            return 0;
        }
        else    // No command line options, must be a mission file path
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
    pFakeModemReport AppObject;
    AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
    cout <<
    "\npFakeModemReport version \n"
    "Written by Brandt Pedrow (pedr5603@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   pFakeModemReport [OPTIONS] MISSION_FILE [APPNAME]\n"
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
    "For additional help, type \"man pFakeModemReport\"\n"
    "\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "//---------------------------------------\n"
    "// GLOBAL-SCOPE MISSION FILE VARIABLES\n"
    "//---------------------------------------\n"
    "\n"
    "// Local coordinate system origin\n"
    "LatOrigin = 46.7302\n"
    "LongOrigin = -117.00893\n"
    "\n"
    "// Navigation beacon locations in local coordinate system"
    "BeaconA_North = 0\n"
    "BeaconA_East = 0\n"
    "BeaconA_Depth = 0\n"
    "BeaconB_North = 100"
    "BeaconB_East = 0\n"
    "BeaconB_Depth = 0\n"
    "BeaconC_North = 50\n"
    "BeaconC_East = -50\n"
    "BeaconC_Depth = 0\n"
    "BeaconD_North = -50\n"
    "BeaconD_East = -50\n"
    "BeaconD_Depth = 0\n"
    "\n"
    "SSH20 = 1.470  // Sound velocity in water (meters/sec)\n"
    "\n"
    "ProcessConfig = pFakeModemReport\n"
    "{\n"
    "\n"
    "   AppTick= 10"
    "   CommsTick = 10\n"
    "}\n"
    "\n"
    << endl;
}

/** @} */   // END @defgroup pFakeModemReport
