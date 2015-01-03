//=============================================================================
/*    Copyright (C) 2013  Dave Billin

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
/** @file iBSDuC.cpp
 *
 * @brief
 *  Implementation of the iBSDuC application class
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================

#include <stdint.h>

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"

#include "iBSDuC.h"
#include "AppVersion.h"

using namespace std;
using namespace YellowSubUtils;



// Character string containing an example mission file configuration
// block for the application
const char iBSDuC::ExampleMissionFileConfiguration[]=
    "ProcessConfig = iBSDuC\n"
    "{\n"
    "    // REQUIRED PARAMETERS\n"
    "\n"
    "    // OPTIONAL PARAMETERS\n"
    "    AppTick   = 100    // Frequency (Hz) at which Iterate() runs\n"
    "    CommsTick = 100    // Frequency (Hz) of MOOS database updates\n"
    "\n"
    "    VERBOSITY = 0      // Verbosity of debugging messages\n"
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
iBSDuC::iBSDuC( void )
:   m_Verbosity( 0 ),
    m_StartupIsDone( false ),
    m_FirmwareResetCounter( 0 ),

    /* Configure proxies for published MOOS variables */
    m_BSD_IsOnline_Proxy( MOOSDBVariableProxy::PUBLISHES,
                          "BSD_IS_ONLINE", "FALSE" ),
    m_BSD_Depth_Proxy( MOOSDBVariableProxy::PUBLISHES, "BSD_DEPTH_M", 0 ),
    m_BSD_DepthFail_Proxy( MOOSDBVariableProxy::PUBLISHES,
                           "BSD_DEPTH_ALARM", "FALSE" ),
    m_Propeller_RPM_Proxy( MOOSDBVariableProxy::PUBLISHES,
                           "PROPELLER_RPM" ),
    m_WaterIsDetected_Proxy( MOOSDBVariableProxy::PUBLISHES,
                             "WATER_IS_DETECTED", "FALSE" ),
    m_BatteryMonitorSerialNum_Proxy( MOOSDBVariableProxy::PUBLISHES,
                                     "BSD_BATTERYMON_SERNUM", "", false ),
    m_FirmwareVersion_Proxy( MOOSDBVariableProxy::PUBLISHES,
                             "BSD_FIRMWARE_VERSION", "", false ),
    m_RPMVelocityEstimate_Proxy( MOOSDBVariableProxy::PUBLISHES,
                                 "RPM_VELOCITY_ESTIMATE", 0 ),
    m_BatteryVolts_Proxy( MOOSDBVariableProxy::PUBLISHES,
                          "BSD_BATTERY_VOLTS", 0, false ),
    m_BatteryAmps_Proxy( MOOSDBVariableProxy::PUBLISHES,
                         "BSD_BATTERY_AMPS", 0, false ),
    m_BatteryDischargedAmps_Proxy( MOOSDBVariableProxy::PUBLISHES,
                                   "BSD_BATTERY_DISCHARGED_AMPS", 0, false ),
    m_BatteryTemp_Proxy( MOOSDBVariableProxy::PUBLISHES,
                         "BSD_BATTERY_TEMP_C", 0, false ),

    /* Configure proxies for subscribed MOOS variables */
    m_DesiredRudderAngle_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                                "RUDDER_ANGLE_DEG", 0 ),
    m_DesiredElevatorAngle_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                                  "ELEVATOR_ANGLE_DEG", 0),
    m_DesiredAileronAngle_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                                 "AILERON_ANGLE_DEG", 0 ),
    m_DesiredThrust_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                           "THROTTLE", 0 ),
    m_LeftElevatorTrim_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                              "LEFT_ELEVATOR_TRIM_DEG", 0 ),
    m_RightElevatorTrim_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                               "RIGHT_ELEVATOR_TRIM_DEG", 0 ),
    m_RudderTrim_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                        "RUDDER_TRIM_DEG", 0 ),
    m_CouplingCoefficient_Proxy( MOOSDBVariableProxy::SUBSCRIBES,
                                 "ACTUATOR_COUPLING_COEFFICIENT",
                                 0 )
{
    //-------------------------------------------------------------
    // Add proxy variables to the app's proxy collection
    //-------------------------------------------------------------
    // Publisher proxies
    m_ProxyCollection.AddProxy( m_BSD_IsOnline_Proxy );
    m_ProxyCollection.AddProxy( m_BSD_Depth_Proxy );
    m_ProxyCollection.AddProxy( m_BSD_DepthFail_Proxy );
    m_ProxyCollection.AddProxy( m_Propeller_RPM_Proxy );
    m_ProxyCollection.AddProxy( m_WaterIsDetected_Proxy );
    m_ProxyCollection.AddProxy( m_BatteryMonitorSerialNum_Proxy );
    m_ProxyCollection.AddProxy( m_FirmwareVersion_Proxy );
    m_ProxyCollection.AddProxy( m_RPMVelocityEstimate_Proxy );
    m_ProxyCollection.AddProxy( m_BatteryVolts_Proxy );
    m_ProxyCollection.AddProxy( m_BatteryAmps_Proxy );
    m_ProxyCollection.AddProxy( m_BatteryDischargedAmps_Proxy );
    m_ProxyCollection.AddProxy( m_BatteryTemp_Proxy );

    // Subscriber proxies
    m_ProxyCollection.AddProxy( m_DesiredRudderAngle_Proxy );
    m_ProxyCollection.AddProxy( m_DesiredElevatorAngle_Proxy );
    m_ProxyCollection.AddProxy( m_DesiredAileronAngle_Proxy );
    m_ProxyCollection.AddProxy( m_DesiredThrust_Proxy );
    m_ProxyCollection.AddProxy( m_LeftElevatorTrim_Proxy );
    m_ProxyCollection.AddProxy( m_RightElevatorTrim_Proxy );
    m_ProxyCollection.AddProxy( m_RudderTrim_Proxy );
    m_ProxyCollection.AddProxy( m_CouplingCoefficient_Proxy );
}



//=============================================================================
iBSDuC::~iBSDuC()
{
    // Un-register proxies
    m_ProxyCollection.UnregisterFromMOOSDB( m_Comms );

    m_BSDMicro.Close();     // Close the BSD serial port connection
}




//=============================================================================
bool iBSDuC::Iterate( void )
{
    // Implement connection and disconnection logic based on connectivity
    // to the MOOSDB
    Service_BSDuC_Connection();



    if ( m_BSDMicro.IsOpen() )
    {
        // Send and receive data, process session startup FSM
        m_BSDMicro.Process();

        // Publish BSD sensors that have changed
        if ( m_BSDMicro.IsOnline() )
        {
            double t = HPMOOSTime();
            bitset<BSDuCModule::NUM_CHANGE_FLAGS> const& SensorChangeFlags =
                                                    m_BSDMicro.ChangeFlags();

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_BATTERY_VOLTS ] )
            {
                m_BatteryVolts_Proxy.Set( m_BSDMicro.BatteryVolts(), t);
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_BATTERY_AMPS ] )
            {
                m_BatteryAmps_Proxy.Set( m_BSDMicro.BatteryAmps(), t );
            }

            if ( SensorChangeFlags[
                            BSDuCModule::CHANGEFLAG_BATTERY_DISCHARGED_AMPS ] )
            {
                m_BatteryDischargedAmps_Proxy.Set(
                                       m_BSDMicro.BatteryAmpsDischarged(), t );
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_BATTERY_TEMP ] )
            {
                m_BatteryTemp_Proxy.Set( m_BSDMicro.BatteryTempC(), t );
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_DEPTH ] )
            {
                m_BSD_Depth_Proxy.Set( m_BSDMicro.Depth_meters(), t );
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_DEPTH_ALARM ] )
            {
                m_BSD_DepthFail_Proxy.Set( m_BSDMicro.DepthAlarm(), t );
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_WATER_DETECTED ] )
            {
                m_WaterIsDetected_Proxy.Set( m_BSDMicro.WaterIsDetected(), t );
            }

            if ( SensorChangeFlags[ BSDuCModule::CHANGEFLAG_PROPELLER_RPM ] )
            {
                m_Propeller_RPM_Proxy.Set( m_BSDMicro.PropellerRpm(), t );
            }

            if ( SensorChangeFlags[BSDuCModule::CHANGEFLAG_BATTERYMON_SERIAL] )
            {
                m_BatteryMonitorSerialNum_Proxy.Set(
                                  m_BSDMicro.BatteryMonitorSerialNumber(), t );
            }

            if ( SensorChangeFlags[BSDuCModule::CHANGEFLAG_FIRMWARE_VERSION] )
            {
                m_FirmwareVersion_Proxy.Set( m_BSDMicro.FirmwareVersion(), t );
            }


            // Apply actuator commands
            if ( m_DesiredRudderAngle_Proxy.IsFresh() )
            {
                float angle = static_cast<float>(
                            m_DesiredRudderAngle_Proxy.AsDouble() );
                m_BSDMicro.SetRudderAngleDeg( angle );
            }

            if ( m_DesiredElevatorAngle_Proxy.IsFresh() )
            {
                float angle = static_cast<float>(
                            m_DesiredElevatorAngle_Proxy.AsDouble() );
                m_BSDMicro.SetElevatorAngleDeg( angle );
            }

            if ( m_DesiredAileronAngle_Proxy.IsFresh() )
            {
                float angle = static_cast<float>(
                            m_DesiredAileronAngle_Proxy.AsDouble() );
                m_BSDMicro.SetAileronAngleDeg( angle );
            }

            if ( m_DesiredThrust_Proxy.IsFresh() )
            {
                float thrust =
                    static_cast<float>( m_DesiredThrust_Proxy.IsFresh() );
                m_BSDMicro.SetThrust( thrust );
            }
        }
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
bool iBSDuC::OnNewMail(MOOSMSG_LIST & NewMail)
{
    //-----------------------------------------------------
    // Update subscriber proxy variables from the MOOSDB
    //-----------------------------------------------------
    m_ProxyCollection.UpdateFromMOOSMail( NewMail, m_Comms );

	return true;
}



//=============================================================================
bool iBSDuC::OnCommandMsg( CMOOSMsg Msg )
{
    // Ignore commands that are skewed (i.e. old)
    if ( Msg.IsSkewed( MOOSTime() ) == false )
    {
        string sVal = Msg.GetString();

        MOOSTrace( "Received iBSDuC_CMD Command: " + sVal + "\n" );


        if ( MOOSStrCmp( sVal, "ResetBatteryMonitor") )
        {
            // Process a request to reset battery monitoring
            if ( m_BSDMicro.IsOnline() )
            {
                m_BSDMicro.RequestBatteryMonitorReset();
            }
        }
        else if ( MOOSStrCmp( sVal, "DisableProp") )
        {
            // Process a request to disable propeller speed commands
            m_BSDMicro.SetPropellerEnabled( false );
        }
        else if ( MOOSStrCmp( sVal, "EnableProp") )
        {
            // Process a request to enable propeller speed commmands
            m_BSDMicro.SetPropellerEnabled( true );
        }
        else if ( MOOSStrCmp( sVal, "SoftReset" ) )
        {
            // Process a request to reset microcontroller firmware
            m_FirmwareResetCounter++;
            if ( m_FirmwareResetCounter < 2 )
            {
                m_FirmwareResetTimer = PrecisionTime::Now();
            }
            else
            {
                m_BSDMicro.RequestFirmwareReset();
            }
        }
    }

    return true;
}




//=============================================================================
bool iBSDuC::OnStartUp( void )
{

    //---------------------------------
	// Print a startup banner
    //---------------------------------
	string sAppName = GetAppName();
	string sBar = string(40, '=') + "\n";
	MOOSTrace(sBar +
			  MOOSFormat("iBSDuC version %s\n", 
                         APP_VERSION_STRING) +
			  "Written by Dave Billin\n" + sBar + "\n\n");

    //---------------------------------
	// Load mission file parameters.
    //---------------------------------
	if ( LoadMissionFileParameters() == true )
	{
	    if ( m_Verbosity > 0 )
	    {
	        MOOSTrace( sBar +
	                   "MOOSDB VARIABLES:\n" +
	                   sBar );

	        MOOSTrace( m_ProxyCollection.Print() +
	                   sBar );
	    }
	}

    return m_StartupIsDone;
}







//=============================================================================
bool iBSDuC::LoadMissionFileParameters( void )
{
    bool rc = true;
    
    //------------------------------------------
    // LOAD REQUIRED MISSION FILE PARAMETERS
    //------------------------------------------

    // Get serial port device name
    bool GotSerialPort = m_MissionReader.GetConfigurationParam( "Port",
                                                        m_SerialPortDevice );
    if ( GotSerialPort == false )
    {
        MOOSTrace( "Missing required mission file parameter: Port\n" );
        rc = true;
    }
    else
    {
        //------------------------------------------
        // LOAD OPTIONAL MISSION FILE PARAMETERS
        //------------------------------------------

        // Apply mission file re-mapping
        m_ProxyCollection.DoMissionFileRemapping( m_MissionReader );

        // Get the verbosity level used for debugging messages
        m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);
    }

    return rc;
}






//=============================================================================
/*
    This function gets called when the application connects to the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iBSDuC::OnConnectToServer( void )
{
    // If OnStartup has already run, (re-)register Dynamic Variables
    // to receive updates from the MOOS database
    if (m_StartupIsDone)
    {
        m_ProxyCollection.RegisterWithMOOSDB( m_Comms );
    }

	return true;
}



//=============================================================================
/*
    This function gets called when the application disconnects from the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iBSDuC::OnDisconnectFromServer( void )
{
	return true;
}


//=============================================================================
void iBSDuC::Service_BSDuC_Connection( void )
{
    bool MOOSDB_is_connected = m_Comms.IsConnected();
    bool BSDuC_is_open = m_BSDMicro.IsOpen();

    if ( (MOOSDB_is_connected == true) && (BSDuC_is_open == false) )
    {
        //----------------------------------------
        // Open the specified serial port
        // connected to the BSD microcontroller
        //----------------------------------------
        if ( m_BSDMicro.Open( m_SerialPortDevice ) )
        {
            // Register proxy variables to receive updates from the MOOSDB
            m_ProxyCollection.RegisterWithMOOSDB( m_Comms );
        }
        else
        {
            MOOSTrace( "Failed to open serial port device "
                        + m_SerialPortDevice + "\n" );
            RequestQuit();
        }
    }
    else if ( (MOOSDB_is_connected == false) && (BSDuC_is_open == true) )
    {
        m_BSDMicro.Close();
    }
}
