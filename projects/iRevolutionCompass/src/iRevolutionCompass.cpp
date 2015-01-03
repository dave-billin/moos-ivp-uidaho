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
/** @file iRevolutionCompass.cpp
 *
 * @brief
 *  Implementation of the iRevolutionCompass application class
 *
 * @author Dave Billin 
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"

#include "iRevolutionCompass.h"
#include "AppVersion.h"

using namespace std;

// Character string containing an example mission file configuration
// block for the application
const char iRevolutionCompass::ExampleMissionFileConfiguration[]=
    "ProcessConfig = iRevolutionCompass\n"
    "{\n"
    "    // REQUIRED PARAMETERS\n"
    "    Port = /dev/ttyO1  // Serial port connected to the compass module\n"
    "    BaudRate = 19200   // Serial port baud rate\n"
    "\n"
    "    // OPTIONAL PARAMETERS\n"
    "    AppTick   = 100    // Frequency (Hz) at which Iterate() runs\n"
    "    CommsTick = 100    // Frequency (Hz) of MOOS database updates\n"
    "\n"
    "    VERBOSITY = 0      // Verbosity of debug messages (greater = more)\n"
    "}\n"
    "\n";





//=============================================================================
/* 
    This function is called when the application object is created in main().
    This is a good place to initialize application variables to default
    values, and set up dynamic memory resources.  

    NOTE: This function executes as the application object is being created, 
          so it's best to save activities that might require error handling
          (e.g. opening files or I/O devices) for the OnStartup() method.
*/
iRevolutionCompass::iRevolutionCompass( void )
:   m_Verbosity(0),
    m_OnStartupIsDone(false),
    m_HeadingOffset_deg(0.0f)
{

}



//=============================================================================
/* 
    This function is called when the application object is exiting.  Make sure
    you release any resources your program allocated!

    NOTE: There is no need to explicitly delete Dynamic Variables.  They get
          cleaned up automatically.
*/
iRevolutionCompass::~iRevolutionCompass()
{
}




//=============================================================================
/* 
    This is where your application will probably do most of its work.  Some
    handy methods you might want to use include:

    Get the current system time:
        MOOSTime()

    Check whether the MOOS database is connected:
        m_Comms.IsConnected()

    Publish a value directly to the MOOS database:
        m_Comms.Notify()

    Set or get the value of a Dynamic Variable:
        SetMOOSVar() and GetMOOSVar()

    Publish Dynamic Variable values that have changed to the MOOS database:
        PublishFreshMOOSVariables()

    Publish a debugging message to the MOOS database variable MOOS_DEBUG:
        MOOSDebugWrite()

    Set the error state and message in the iRevolutionCompass_STATUS variable:
        SetAppError()

    NOTE: returning false will cause the application to exit.
*/
bool iRevolutionCompass::Iterate( void )
{
    m_Compass.ProcessSerialComms(); // Process serial data

    double t = HPMOOSTime(false);   // Get report time


    //------------------------------
    // Handle connection timeout
    //------------------------------
    if ( m_Compass.ConnectionTimedOut() )
    {
        MOOSTrace("Connection to compass timed out after 5 seconds!\n");
        RequestQuit();
    }

    //------------------------------
    // Apply heading offset
    //------------------------------
    float AdjustedHeading = m_Compass.GetHeading();
    AdjustedHeading += m_HeadingOffset_deg;

    while (AdjustedHeading >= 360.0f)
    {
        AdjustedHeading -= 360.0f;
    }

    if ( !( m_PublishedVarNames[Heading].empty() ||
            m_Compass.PitchAlarmIsAsserted() ) )
    {
        m_Comms.Notify( m_PublishedVarNames[Heading],
                        AdjustedHeading, t );
    }


    if ( !m_PublishedVarNames[Dip].empty() )
    {
        m_Comms.Notify( m_PublishedVarNames[Dip],
                        m_Compass.GetDip(), t );
    }

    if ( !(m_PublishedVarNames[Pitch].empty() ||
           m_Compass.PitchAlarmIsAsserted() ) )
    {
        m_Comms.Notify( m_PublishedVarNames[Pitch],
                        m_Compass.GetPitch(), t );
    }

    if ( !(m_PublishedVarNames[Roll].empty() ||
           m_Compass.RollAlarmIsAsserted() ) )
    {
        m_Comms.Notify( m_PublishedVarNames[Roll],
                        m_Compass.GetRoll(), t );
    }

    if ( !m_PublishedVarNames[MagX].empty() )
    {
        m_Comms.Notify( m_PublishedVarNames[MagX],
                        m_Compass.GetMagX(), t );
    }

    if ( !m_PublishedVarNames[MagY].empty() )
    {
        m_Comms.Notify( m_PublishedVarNames[MagY],
                        m_Compass.GetMagY(), t );
    }

    if ( !m_PublishedVarNames[MagZ].empty() )
    {
        m_Comms.Notify( m_PublishedVarNames[MagZ],
                        m_Compass.GetMagZ(), t );
    }


    string sAlarms;

    if ( m_Compass.MagnetometerAlarmIsAsserted() )
    {
        sAlarms += "Magnetometer";
    }

    if ( m_Compass.PitchAlarmIsAsserted() )
    {
        if ( !sAlarms.empty() )
        {
            sAlarms += ", ";
        }
        sAlarms += "Pitch";
    }

    if ( m_Compass.RollAlarmIsAsserted() )
    {
        if ( !sAlarms.empty() )
        {
            sAlarms += ", ";
        }
        sAlarms += "Roll";
    }

    if ( !m_PublishedVarNames[Status].empty() )
    {
        m_Comms.Notify( m_PublishedVarNames[Status], sAlarms, t );
    }

	return true;
}






//=============================================================================
/*
    Here is where the application should handle mail received from the MOOS 
    database.  This mail takes the form of CMOOSMsg objects - each of which
    contains the name and contents of a MOOS variable the application has
    subscribed to.

    NOTE: returning false will cause the application to exit.
*/
bool iRevolutionCompass::OnNewMail(MOOSMSG_LIST & NewMail)
{
    // This application does not subscribe to any variables
	return true;
}









//=============================================================================
/*
    This function is called once when the application starts running.  The 
    first thing it does is call to LoadMissionParameters() to read parameters
    from the (.moos) mission file.

    This function is a good spot to open files, connect to I/O devices, or 
    perform other actions that might result in an error condition that must be
    handled gracefully.

    This is also a good place to create Dynamic Variables (which can simplify 
    the process of publishing and subscribing to the MOOS database) using 
    AddMOOSVariable().

    NOTE: returning false will cause the application to exit.
*/
bool iRevolutionCompass::OnStartUp( void )
{

    //---------------------------------
	// Print a startup banner
    //---------------------------------
	string sAppName = GetAppName();
	string sBar = string(40, '=') + "\n";
	MOOSTrace(sBar +
			  MOOSFormat("iRevolutionCompass version %s\n", 
                         APP_VERSION_STRING) +
			  "Written by Dave Billin\n" + sBar + "\n\n");


    //---------------------------------
	// Load mission file parameters.
    //---------------------------------
	if ( LoadMissionFileParameters() == false )
	{
		return false;
	}

    //------------------------------------------------------------
    // << Do your start-up work here: create Dynamic Variables, >>
    // << connect to I/O devices, etc.                          >>
    //------------------------------------------------------------


    // Register Dynamic Variables to receive 
    // updates from the MOOS database
    m_OnStartupIsDone = true;
    RegisterMOOSVariables();
    
    return true;
}







//=============================================================================
/*
    This function gets called from OnStartup() to load parameters from the
    (.moos) mission file.  Call: 

    m_MissionReader.GetConfigurationParam()
        to read the value of a mission file parameter occurring in the 
        application's "ProcessConfig = iRevolutionCompass" block

    m_MissionReader.GetValue()
        to load a global scope parameter (i.e. one occuring outside a
        ProcessConfig block)

    NOTE: There is no need to load the AppTick and CommsTick parameters;  
          they are automatically loaded.

    Returning false from this function causes the application to exit.
*/
bool iRevolutionCompass::LoadMissionFileParameters( void )
{
    
    //------------------------------------------
    // LOAD REQUIRED MISSION FILE PARAMETERS
    //------------------------------------------
    string s;
    const char* szParamFail = "Failed to read required mission file "
                              "parameter %s\n";

    //----------------------------------------
    // Load required mission file parameters
    //----------------------------------------

    // Get the hostname of the DAQ module
    string sSerialPortName;
    s = "PORT";
    if ( !m_MissionReader.GetConfigurationParam(s, sSerialPortName) )
    {
        return MOOSFail(szParamFail, s.c_str());
    }



    //------------------------------------------
    // LOAD OPTIONAL MISSION FILE PARAMETERS
    //------------------------------------------
    int BaudRate = 19200;
    s = "BAUDRATE";
    m_MissionReader.GetConfigurationParam(s, BaudRate);

    int Verbosity = 0;
    m_MissionReader.GetConfigurationParam("VERBOSITY", Verbosity);


    //--------------------------------
    // Initialize the compass module
    //--------------------------------
    MOOSTrace("Initializing compass module...\n");
    if ( !m_Compass.Initialize(sSerialPortName, BaudRate, Verbosity) )
    {
        return MOOSFail("FAILED!");
    }


    //--------------------------------
    // Load compass module parameters
    //--------------------------------
    MOOSTrace("Configuring compass module...\n");
    // Apply a new baud rate
    int NewBaudRate;
    if ( m_MissionReader.GetConfigurationParam("SET_BAUDRATE", NewBaudRate) )
    {
        int BaudRateTable[] = { 2400, 4800, 9600, 19200 };

        for (int i = 0; i < 4; i++)
        {
            // Find corresponding baud rate ID and apply it
            if ( NewBaudRate == BaudRateTable[i] )
            {
                if ( m_Compass.SetBaudRate(i+1) )
                {
                    MOOSTrace("Compass baud rate set to %d", NewBaudRate);
                }
                else
                {
                    return MOOSFail("Failed to set compass baud rate to %d\n",
                                    NewBaudRate );
                }
            }
        }
    }

    int i;
    if ( m_MissionReader.GetConfigurationParam("UPDATES_PER_SECOND", i) )
    {
        int SupportedRateTable[] = {1, 2, 3, 5, 7, 10, 20};
        int RateIDTable[] = { RevolutionCompassModule::Rate_1Hz,
                              RevolutionCompassModule::Rate_2Hz,
                              RevolutionCompassModule::Rate_3Hz,
                              RevolutionCompassModule::Rate_5Hz,
                              RevolutionCompassModule::Rate_6p88Hz,
                              RevolutionCompassModule::Rate_10Hz,
                              RevolutionCompassModule::Rate_20Hz };

        if ( (i <= 20) && (i >= 0))
        {
            // Get supported rate ID
            int UpdatesPerSecond = 0;
            for (int r = 0; r < 7; r++)
            {
                if (i == SupportedRateTable[r])
                {
                    UpdatesPerSecond = RateIDTable[r];
                    break;
                }
            }

            if (UpdatesPerSecond == 0)
            {
                MOOSTrace("Invalid value for UPDATES_PER_SECOND parameter: "
                          "%d\n", UpdatesPerSecond);
            }
            else
            {
                // Set output rate of HTM and XDR messages
                bool bSuccess = true;
                bSuccess &= m_Compass.SetMessageOutputRate(
                                      RevolutionCompassModule::NMEA_PTNTHTM, i);
                bSuccess &= m_Compass.SetMessageOutputRate(
                                      RevolutionCompassModule::NMEA_HCXDR, i);

                if (bSuccess)
                {
                    // Set AppTick to accommodate rate
                    SetAppFreq(i);
                    SetCommsFreq(i);
                }
                else
                {
                    return MOOSFail("Failed to set message output rate "
                                    "to %d Hz!\n", i);
                }
            }
        }
    }

    float f;

    if ( m_MissionReader.GetConfigurationParam("DEVIATION", f) )
    {
        if ( !m_Compass.SetDeviation(f) )
        {
            return MOOSFail("Failed to set compass Deviation to %f!\n", f);
        }
    }

    m_MissionReader.GetConfigurationParam("HEADING_OFFSET",
                                          m_HeadingOffset_deg);

    if ( m_MissionReader.GetConfigurationParam("DECLINATION", f) )
    {
        if ( !m_Compass.SetDeclination(f) )
        {
            return MOOSFail("Failed to set magnetic Declination to %f!\n", f);
        }
    }

    if ( m_MissionReader.GetConfigurationParam("TILT_FILTER_TIME_CONSTANT", f) )
    {
        if ( !m_Compass.SetFilterTimeConstant(
                                        RevolutionCompassModule::TiltFilter,f) )
        {
            return MOOSFail("Failed to set tilt filter time constant to %f!\n",
                            f);
        }
    }

    if ( m_MissionReader.GetConfigurationParam("MAG_FILTER_TIME_CONSTANT", f) )
    {
        if ( !m_Compass.SetFilterTimeConstant(
                                    RevolutionCompassModule::MagneticFilter,f) )
        {
            return MOOSFail("Failed to set magnetic filter time constant "
                            "to %f!\n", f);
        }
    }

    if ( m_MissionReader.GetConfigurationParam("ALARM_TIME_CONSTANT", f) )
    {
        if ( !m_Compass.SetFilterTimeConstant(
                                    RevolutionCompassModule::AlarmFilter, f) )
        {
            return MOOSFail("Failed to set alarm filter time constant "
                            "to %f!\n", f);
        }
    }


    if ( m_MissionReader.GetConfigurationParam("MAGNETOMETER_GAIN", i) )
    {
        uint8_t Gain = static_cast<uint8_t>(i);
        if ( !m_Compass.SetMagnetometerGain(Gain) )
        {
            return MOOSFail("Failed to set magnetometer gain to %d!\n",
                            Gain);
        }
    }


    if ( m_MissionReader.GetConfigurationParam("TILT_ALARM_THRESHOLD", f) )
    {
        if ( !m_Compass.SetTiltAlarmThreshold(f) )
        {
            return MOOSFail("Failed to set tilt alarm threshold to %f!\n",
                            f);
        }
    }


    if ( m_MissionReader.GetConfigurationParam("PITCH_OFFSET", f) )
    {
        if ( !m_Compass.SetPitchOffset(f) )
        {
            return MOOSFail("Failed to set pitch offset to %f!\n",
                            f);
        }
    }

    if ( m_MissionReader.GetConfigurationParam("ROLL_OFFSET", f) )
    {
        if ( !m_Compass.SetRollOffset(f) )
        {
            return MOOSFail("Failed to set roll offset to %f!\n",
                            f);
        }
    }

    MOOSTrace( m_Compass.GetDeviceParamString() );


    //------------------------------------------------
    // Load alternate MOOS variables to publish to
    //------------------------------------------------
    static const char* szSensorNames[] = { "HEADING", "DIP", "PITCH", "ROLL",
                                           "MAGX", "MAGY", "MAGZ", "ALARMS" };

    for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
    {
        string sTemp("REVCOMPASS_");
        sTemp += szSensorNames[i];

        // Assign default variable name
        m_PublishedVarNames[i] = sTemp;

        sTemp += "_PUBLISHTO";

        m_MissionReader.GetConfigurationParam(sTemp, m_PublishedVarNames[i] );
    };

    //---------------------------
    // List published variables
    //---------------------------
    MOOSTrace( "\n\n"
               "SENSOR    PUBLISHED AS\n"
               "--------------------------\n");
    for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
    {
        MOOSTrace("%-10s%s\n", szSensorNames[i],
                               m_PublishedVarNames[i].c_str() );
    }

	return true;
}






//=============================================================================
/*
    This function gets called when the application connects to the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iRevolutionCompass::OnConnectToServer( void )
{
    // If OnStartup has already run, (re-)register Dynamic Variables
    // to receive updates from the MOOS database
    if (m_OnStartupIsDone)
    {
        RegisterMOOSVariables();
    }

    // << Perform any tasks associated with MOOSDB connection >>

	return true;
}



//=============================================================================
/*
    This function gets called when the application disconnects from the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iRevolutionCompass::OnDisconnectFromServer( void )
{
    // << Perform any tasks associated with MOOSDB disconnection >>

	return true;
}

