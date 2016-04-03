//=============================================================================
/** @file iXStreamRadioModem/src/main.cpp
 *
 * @brief
 *  Execution entry point for the iXStreamRadioModem application
 *
 * @author Dave Billin
 *
 */
//=============================================================================
#include "iXStreamRadioModem.h"
#include "config.h"
#include <MOOS/libMOOS/MOOSLib.h>
#include <iostream>

using std::cout;
using std::endl;

static std::string APPLICATION_VERSION(APP_VERSION_TUPLE);

/** @defgroup iXStreamRadioModem
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
    const char* szMOOSName = "iXStreamRadioModem";

    // Print usage info and exit if no command line arguments were supplied
    if (argc == 1)
    {
		cout << "\nUSAGE:   iXStreamRadioModem [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'iXStreamRadioModem --help' or 'man "
                "iXStreamRadioModem'\n"
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
            cout << "\niXStreamRadioModem v" << APPLICATION_VERSION << "\n" << endl;
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
    iXStreamRadioModem AppObject;
    AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
    cout <<
    "\niXStreamRadioModem v" << APPLICATION_VERSION << "\n"
    "Written by Dave Billin (dave.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   iXStreamRadioModem [OPTIONS] MISSION_FILE [APPNAME]\n"
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
    "For additional help, type \"man iXStreamRadioModem\"\n"
    "\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "ProcessConfig = iSpock\n"
    "{\n"
    "    AppTick = 10\n"
    "    CommsTick = 8\n"
    "\n"
    "    // Hostname or IP address of the SPIOCK module\n"
    "    SPOCK_HOSTNAME = 192.168.2.103\n"
    "\n"
    "    // Network port to connect to on the SPOCK module\n"
    "    SPOCK_PORT = 20023\n"
    "\n"
    "   //------------------------------------\n"
    "   // OPTIONAL MISSION FILE PARAMETERS\n"
    "   //------------------------------------\n"
    "   //COMPASS_HEADING_PUBLISHTO  = <alternate variable to publish to>\n"
    "   //COMPASS_YAW_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //COMPASS_PITCH_PUBLISHTO    = <alternate variable to publish to>\n"
    "   //COMPASS_ROLL_PUBLISHTO     = <alternate variable to publish to>\n"
    "   //COMPASS_DIP_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //DEPTH_PUBLISHTO            = <alternate variable to publish to>\n"
    "   //ACCEL_PITCH_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //ACCEL_ROLL_PUBLISHTO       = <alternate variable to publish to>\n"
    "   //GPS_LONGITUDE_PUBLISHTO    = <alternate variable to publish to>\n"
    "   //GPS_LATITUDE_PUBLISHTO     = <alternate variable to publish to>\n"
    "   //GPS_VELOCITY_PUBLISHTO     = <alternate variable to publish to>\n"
    "   //GPS_HPE_PUBLISHTO          = <alternate variable to publish to>\n"
    "   //GPS_HEADING_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //GPS_YAW_PUBLISHTO          = <alternate variable to publish to>\n"
    "   //GPS_HOURS_PUBLISHTO        = <alternate variable to publish to>\n"
    "   //GPS_MINUTES_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //GPS_SECONDS_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //WATERLEAKISDETECTED_PUBLISHTO = <alternate variable to publish to>\n"
    "   //TEMPERATURE_PUBLISHTO      = <alternate variable to publish to>\n"
    "   //BATTERY_VOLTAGE_PUBLISHTO  = <alternate variable to publish to>\n"
    "\n"
    "\n"
    "   // Set this variable to \"TRUE\" if sensors should only be published\n"
    "   // when they change; otherwise, sensors are published at a rate\n"
    "   // equal to the value of AppTick\n"
    "   PUBLISH_ONLY_CHANGES = FALSE\n"
    "   \n"
    "   VERBOSITY = 0   // (optional) verbosity of debugging messages\n"
    "}\n"
    "\n"
    << endl;
}

/** @} */   // END @defgroup iXStreamRadioModem
