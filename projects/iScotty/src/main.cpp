//=============================================================================
/** @file iScotty/src/main.cpp
 *
 * @brief
 *  Execution entry point for the iScotty application
 *
 * @author Dave Billin
 *
 */
//=============================================================================
#include "iScotty.h"
#include "AppVersion.h"

#include <MOOS/libMOOS/MOOSLib.h>
#include <iostream>

using std::cout;
using std::endl;


static std::string APPLICATION_VERSION(APP_VERSION_STRING);

/** @defgroup iScotty
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
    const char* szMOOSName = "iScotty";

    // Print usage info and exit if no command line arguments were supplied
    if (argc == 1)
    {
		cout << "\nUSAGE:   iScotty [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, type 'iScotty --help' or 'man "
                "iScotty'\n"
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
            cout << "\niScotty version \n" << endl;
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
    iScotty AppObject;
    AppObject.Run((char*)szMOOSName, (char*)szMissionFile);

    return 0;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
    cout <<
    "\niScotty version " << APPLICATION_VERSION << "\n"
    "Written by Dave Billin (dave.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   iScotty [OPTIONS] MISSION_FILE [APPNAME]\n"
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
    "For additional help, type \"man iScotty\"\n"
    "\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    cout <<
    "\n"
    "ProcessConfig = iScotty\n"
    "{\n"
    "    AppTick = 10\n"
    "    CommsTick = 8\n"
    "\n"
    "    // Hostname or IP address of the SCOTTY module\n"
    "    SCOTTY_HOSTNAME = 192.168.2.104\n"
    "\n"
    "    // Network port to connect to on the SCOTTY module\n"
    "    SCOTTY_PORT = 20024\n"
    "\n"
    "\n"
    "\n"
    "    //---------------------------\n"
    "    // HEADING CONTROLLER GAINS\n"
    "    //---------------------------\n"
    "    YAW_PID_Kp = 0.8           // Proportional gain\n"
    "    YAW_PID_Ki = 0.0           // Integral gain\n"
    "    YAW_PID_Kd = 0.0           // Derivative gain\n"
    "    YAW_PID_IntSat = 0.0       // Integral saturation value\n"
    "\n"
    "    YAW_PID_OutSat = 1.483529839   // Output (rudder) saturation value\n"
    "\n"
    "\n"
    "    //---------------------------\n"
    "    // SPEED CONTROLLER GAINS\n"
    "    //---------------------------\n"
    "    SPEED_PID_Kp = 1.0         // Proportional gain\n"
    "    SPEED_PID_Ki = 0.0         // Integral gain\n"
    "    SPEED_PID_Kd = 0.0         // Derivative gain\n"
    "    SPEED_PID_IntSat = 0.0     // Integral saturation value\n"
    "\n"
    "    SPEED_PID_OutSat = 100.0   // Output (thrust) saturation value\n"
    "\n"
    "\n"
    "    //---------------------------\n"
    "    // DEPTH CONTROLLER GAINS\n"
    "    //---------------------------\n"
    "    DEPTH_PID_Kp = 1.0         // Proportional gain\n"
    "    DEPTH_PID_Ki = 0.0         // Integral gain\n"
    "    DEPTH_PID_Kd = 0.0         // Derivative gain\n"
    "    DEPTH_PID_IntSat = 100.0   // Integral saturation value\n"
    "\n"
    "\n"
    "    //---------------------------\n"
    "    // ROLL CONTROLLER GAINS\n"
    "    //---------------------------\n"
    "    ROLL_PID_Kp = 1.0          // Proportional gain\n"
    "    ROLL_PID_Ki = 0.0          // Integral gain\n"
    "    ROLL_PID_Kd = 0.0          // Derivative gain\n"
    "    ROLL_PID_IntSat = 0.0      // Integral saturation value\n"
    "    ROLL_PID_OutSat = 1.483529839   // Output (aileron) saturation value\n"
    "\n"
    "\n"
    "    //---------------------------\n"
    "    // PITCH CONTROLLER GAINS\n"
    "    //---------------------------\n"
    "    PITCH_PID_Kp = 2.0       // Proportional gain on pitch error\n"
    "    PITCH_PID_KpDepth = 0.0  // Proportional gain on depth controller output\n"
    "    PITCH_PID_Ki = 1.0       // Integral gain\n"
    "    PITCH_PID_Kd = 0.0       // Derivative gain\n"
    "    PITCH_PID_IntSat = 2.0         // Integral saturation value\n"
    "    PITCH_PID_OutSat = 1.483529839 // Output (elevator) saturation value\n"
    "\n"
    "\n"
    "    // Values used to estimate RPM-based velocity\n"
    "    VELOCITY_PER_RPM = 0.001167\n"
    "    VELOCITY_PER_RPM_OFFSET = -0.1988\n"
    "\n"
    "    VERBOSITY = 0   // (optional) verbosity of debugging messages\n"
    "}\n"
    "\n"
    << endl;
}

/** @} */   // END @defgroup iScotty
