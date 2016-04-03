//=============================================================================
/** @file pAcousticCommsAgent/src/main.cpp
 *
 * @brief
 *  Execution entry point for the pAcousticCommsAgent application
 *
 * @author Dave Billin
 *
 */
//=============================================================================
#include "pAcousticCommsAgent.h"
#include "config.h"
#include <MOOS/libMOOS/MOOSLib.h>
#include <iostream>

using std::cout;
using std::endl;

static std::string APPLICATION_VERSION(APP_VERSION_TUPLE);


/** @defgroup pAcousticCommsAgent
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
    const char* szMOOSName = "pAcousticCommsAgent";

    // Print usage info and exit if no command line arguments were supplied
    if (argc == 1)
    {
		cout << "\nUSAGE:   pAcousticCommsAgent [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'pAcousticCommsAgent --help' or 'man "
                "pAcousticCommsAgent'\n"
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
            cout << "\npAcousticCommsAgent v" << APPLICATION_VERSION << "\n" << endl;
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
    pAcousticCommsAgent AppObject;
    AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
    cout <<
    "\npAcousticCommsAgent v" << APPLICATION_VERSION << "\n"
    "Written by Dave Billin (dave.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   pAcousticCommsAgent [OPTIONS] MISSION_FILE [APPNAME]\n"
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
    "For additional help, type \"man pAcousticCommsAgent\"\n"
    "\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "ProcessConfig = pAcousticCommsAgent\n"
    "{\n"
    "\n"
    "   AppTick= 10"
    "   CommsTick = 10\n"
    "\n"
    "   TIMING_CYCLE_FILE = /home/bill5510/ui-moos-auv/pAcousticCommsAgent/"
    "FleetComms_3sub.cyc\n"
    "\n"
    "}\n"
    "\n"
    << endl;
}

/** @} */   // END @defgroup pAcousticCommsAgent
