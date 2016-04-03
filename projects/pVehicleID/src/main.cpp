//=============================================================================
/** @file pVehicleID/src/main.cpp
 *
 * @brief
 *  Execution entry point for the pVehicleID application
 *
 * @author Brandt Pedrow
 *
 */
//=============================================================================
#include "pVehicleID.h"
#include "config.h"
#include <MOOS/libMOOS/MOOSLib.h>
#include <iostream>

using std::cout;
using std::endl;


/** @defgroup pVehicleID
 * @{
 */

static std::string APPLICATION_VERSION(APP_VERSION_TUPLE);

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
    const char* szMOOSName = "pVehicleID";

    // Print usage info and exit if no command line arguments were supplied
    if (argc == 1)
    {
		cout << "\nUSAGE:   pVehicleID [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'pVehicleID --help' or 'man "
                "pVehicleID'\n"
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
            cout << "\npVehicleID v" << APPLICATION_VERSION << "\n" << endl;
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
    pVehicleID AppObject;
    AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
    cout <<
    "\npVehicleID v" << APPLICATION_VERSION << "\n"
    "Written by Brandt Pedrow (pedr5603@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   pVehicleID [OPTIONS] MISSION_FILE [APPNAME]\n"
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
    "For additional help, type \"man pVehicleID\"\n"
    "\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "// Absolute path of vehicle-specific configuration file\n"
    "VEHICLE_CONFIG_FILE_PATH = \AuvConfig.cfg\n"
    "\n"
    "ProcessConfig = pVehicleID\n"
    "{\n"
    "   AppTick = 1\n"
    "   CommsTick   = 1"
    "\n"
    "   //------------------------\n"
    "   // OPTIONAL PARAMETERS\n"
    "   //------------------------\n"
    "   // Alternate MOOSDB variable to publish the vehicle ID to\n"
    "   //PUBLISHTO_VEHICLEID = <MOOSDB variable name>\n"
	"}\n"
    "\n"
    << endl;
}

/** @} */   // END @defgroup pVehicleID
