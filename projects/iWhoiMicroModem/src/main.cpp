//=============================================================================
/** @file iWhoiMicroModem/src/main.cpp
 *
 * @brief
 *  Execution entry point for the iWhoiMicroModem application
 *
 * @author Dave Billin
 *
 */
//=============================================================================
#include "iWhoiMicroModem.h"
#include "config.h"

#include <MOOS/libMOOS/MOOSLib.h>
#include <iostream>

using std::cout;
using std::endl;

/** @defgroup iWhoiMicroModem
 * @{
 */

static std::string const APPLICATION_VERSION(APP_VERSION_TUPLE);

//================================
// Function Prototypes
//================================
void PrintUsageInfo( void );
void PrintExampleConfig( void );

//=============================================================================
/** Creates an instance of the application and runs it */
int main( int argc, char* argv[] )
{
   const char* szMissionFile = NULL;
   const char* szMOOSName = "iWhoiMicroModem";

   // Print usage info and exit if no command line arguments were supplied
   if ( argc == 1 )
   {
      cout << "\nUSAGE:   iWhoiMicroModem [OPTIONS] MISSION_FILE [APPNAME]\n"
              "For more info, type 'iWhoiMicroModem --help' or 'man "
              "iWhoiMicroModem'\n"
           << endl;

      return 0;
   }

   // Parse command line arguments for options or mission file
   if ( argc >= 2 )
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
         cout << "\n"
                 "iWhoiMicroModem version \n" << APPLICATION_VERSION << endl;
         return 0;
      }
      else    // No command line options, must be a mission file path
      {
         szMissionFile = argv[1];
      }
   }

   // Grab alternate name to use when registering if specified
   if ( argc >= 3 )
   {
      szMOOSName = argv[2];
   }

   // Create and launch the application
   iWhoiMicroModem AppObject;
   AppObject.Run((char*) szMOOSName, (char*) szMissionFile);

   return 0;
}

//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
   cout << "\niWhoiMicroModem v" << APPLICATION_VERSION
        << "\n"
           "Written by Dave Billin (dave.billin@vandals.uidaho.edu)\n"
           "\n"
           "USAGE:   iWhoiMicroModem [OPTIONS] MISSION_FILE [APPNAME]\n"
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
           "For additional help, type \"man iWhoiMicroModem\"\n"
           "\n"
        << endl;
}

//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig( void )
{
   cout << "\n"
           "ProcessConfig = iWhoiMicroModem\n"
           "{\n"
           "    AppTick = 4\n"
           "    CommsTick = 4\n"
           "\n"
           "   //----------------------------\n"
           "   // SERIAL PORT SETTINGS\n"
           "   //----------------------------\n"
           "   Port = COM1\n"
           "   BaudRate = 19200\n"
           "   Streaming = true\n"
           "\n"
           "\n"
           "   //------------------------------------\n"
           "   // WHOI NVRAM PARAMETERS\n"
           "   // 	These parameters describe settings applied to the WHOI NVRAM\n"
           "   //	configuration parameters.  See Table 5 of the Micro-Modem Software\n"
           "   //	Interface Guide for detailed information on parameter values\n"
           "   //------------------------------------\n"
           "   AGC = 0     // Enable automatic gain control (1=enabled; 0=disabled)\n"
           "   AGN = 255   // Analog Gain (0..255)\n"
           "   ASD = 0     // Enable test data transmission (1=enabled;0=disabled)\n"
           "   BND = 3     // Communications Band\n"
           "               //      1 = A (10 kHz)\n"
           "               //      2 = B (15 kHz)\n"
           "               //      3 = C (25 kHz)\n"
           "               //      4 = D (?)\n"
           "\n"
           "   BR2 = 3     // WHOI Aux Serial port (port 2) baud rate\n"
           "               //  0 = 2400 bps\n"
           "               //  1 = 4800 bps\n"
           "               //  2 = 9600 bps\n"
           "               //  3 = 19200 bps (default)\n"
           "               //  4 = 38400 bps\n"
           "               //  5 = 57600 bps\n"
           "               //  6 = 115200 bps\n"
           "               //  7 = 230400 bps\n"
           "\n"
           "   //BW0 = 4000    // Bandwidth for Band 0 PSK\n"
           "   CPR = 1         // Co-processor power (0=manual; 1=auto)\n"
           "   CRL = 50        // Cycle-init reverb lockout (milliseconds)\n"
           "   CST = 0         // Rx cycle statistics message (1=enable;0=disable)\n"
           "   CTO = 10        // Cycle-init timeout (sec)\n"
           "   DBG = 0         // Low-level debug messages (1=enable;0=disable)\n"
           "   DGM = 0         // Diagnostic messages (1=enable;0=disable)\n"
           "   DOP = 1         // CADOP message (1=enable;0=disable)\n"
           "   DQF = 1         // CADQF message (1=enable;0=disable)\n"
           "   DTH = 108       // Matched filter signal threshold, FSK\n"
           "   DTO = 2         // Data request timeout (sec)\n"
           "   //DTP = 90      // Matched filter signal threshold, PSK\n"
           "   ECD = 50        // End-of-cycle delay (milliseconds)\n"
           "   EFF = 20        // Feed-forward taps for the LMS equalizer\n"
           "   EFB = 10        // Feedback taps for the LMS equalizer\n"
           "   //FCO = 25120   // Carrier at Band 0 PSK only\n"
           "   //FML = 40      // PSK FM probe length, symbols\n"
           "   //FMD = 1       // PSK FM probe direction (0=up; 1=down)\n"
           "\n"
           "   GPS = 0     // Parse GPS on aux. serial port (1=enable;0=false)\n"
           "   //HFC = 0   // Hardware flow control on main serial port\n"
           "               // (1=enabled;0=disabled)\n"
           "\n"
           "   IRE = 0     // Print impulse response of FM sweep\n"
           "   //MOD = 0   // Mini-packet type (0=FSK; 1=PSK)\n"
           "   MFD = 1     // MFD messages (1=enable;0=disable)\n"
           "   MSE = 0     // Print MSE (dB) of LMS Eq\n"
           "   MCM = 1     // Current mode hydrophone PSU on Rev C. analog board\n"
           "               // (1=enable;0=disable)\n"
           "   MPR = 1     // Multi-channel analog board power (1=enable;0=disable)\n"
           "   //MVM = 1   // Analog board voltage mode hydrophone PSU\n"
           "               // (1=enable;0=disable)\n"
           "\n"
           "   NDT = 150   // Detect threshold for nav detector\n"
           "   NPT = 50    // Power threshold for nav detector\n"
           "   NRL = 25    // Navigation reverb lockout time (milliseconds)\n"
           "   NRV = 150   // Number of cycle timeout's before rebooting\n"
           "   PAD = 2     // Power amp delay (ms)\n"
           "   PCM = 0     // Passband channel mask\n"
           "   PRL = 50    // Packet reverb lockout (milliseconds)\n"
           "   PTH = 180   // Matched filter detector power threshold\n"
           "   POW = -100  // Detection power threshold (dB)\n"
           "   PTO = 6     // Packet timeout (sec)\n"
           "   REV = 1     // CAREV message (1=enable;0=disable)\n"
           "   RSP = -157  // Receiver gain response (dB)\n"
           "   RXA = 1     // CARXA message (1=enable;0=disable)\n"
           "   RXD = 1     // CARXD message (1=enable;0=disable)\n"
           "   RXP = 1     // CARXP message (1=enable;0=disable)\n"
           "   SCG = 0     // Set clock from GPS data\n"
           "   SGP = 0     // Show GPS messages on main serial port\n"
           "   SHF = 0     // CASHF message (1=enable;0=disable)\n"
           "   SNR = 0     // SNR stats for PSK communications (1=enable;0=disable)\n"
           "   SNV = 0     // Synchronous transmission of packets (1=enable;0=disable)\n"
           "   SRC = 2     // *** Default Acoustic ID ***\n"
           "   TAT = 50    // Navigation turn-around time (milliseconds)\n"
           "   TOA = 0     // Display of packet time of arrival (1=TRUE;0=FALSE)\n"
           "   TXD = 250   // Delay before transmit (milliseconds)\n"
           "   XST = 0     // CAXST stats message (1=enable;0=disable)\n"
           "\n"
           "\n"
           "   //------------------------------------\n"
           "   // *OPTIONAL* COMMAND PRIORITIES\n"
           "   //  Commands passed to the WHOI MicroModem at runtime are prioritized\n"
           "   //  based on these values.  Highest priority is 10; lowest is 0.\n"
           "   //  Commands with equal priority are handled in order of receipt from\n"
           "   //  the MOOS DB.\n"
           "   PRIORITY_SendUserMiniPacket = 10\n"
           "   PRIORITY_WriteASCIIData = 10\n"
           "   PRIORITY_ReadASCIIData = 10\n"
           "   PRIORITY_WriteBinaryData = 10\n"
           "   PRIORITY_ReadBinaryData = 10\n"
           "   PRIORITY_SendMiniPacketPing = 9\n"
           "   PRIORITY_SendFmSweepPing = 9\n"
           "   PRIORITY_SendRemusPing = 9\n"
           "   PRIORITY_SendNarrowBandPing = 9\n"
           "   PRIORITY_SetRealtimeClock = 8\n"
           "   PRIORITY_SetNvramParamValue = 7\n"
           "   PRIORITY_GetNvramParamValue = 7\n"
           "   PRIORITY_SetIOLineState = 6\n"
           "   PRIORITY_ModemSleep = 5\n"
           "   PRIORITY_AutoLevelAgc = 4\n"
           "   PRIORITY_MeasureNoiseLevel = 4\n"
           "\n"
           "\n"
           "   // Set this value to TRUE if ALL received data packets should be\n"
           "   // published to the MOOSDB, or FALSE (default) if only data packets\n"
           "   // addressed to the assigned acoustic ID should be published.\n"
           "   ENABLE_PROMISCUOUS_DATARX = true\n"
           "\n"
           "   // Set this value to TRUE if ALL received (13-bit) mini-packets should be\n"
           "   // published to the MOOSDB, or FALSE (default) if only packets\n"
           "   // addressed to the assigned acoustic ID should be published.\n"
           "   ENABLE_PROMISCUOUS_MINIPACKETRX = true\n"
           "\n"
           "}\n"
           "\n"
        << endl;
}

/** @} */   // END @defgroup iWhoiMicroModem
