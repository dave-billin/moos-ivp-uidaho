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
/** @file iSpock.cpp

@brief
	Implementation of the iSpock application object

@author Dave Billin

*/
//=============================================================================

#include "MOOS/libMOOS/App/MOOSApp.h"
#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "iSpock.h"

using namespace std;

// Verbosity macros
#define VERBOSE1(_expr_)	if (m_Verbosity >= 1) { _expr_; }
#define VERBOSE2(_expr_)	if (m_Verbosity >= 2) { _expr_; }
#define VERBOSE3(_expr_)	if (m_Verbosity >= 3) { _expr_; }



/* 	Default variable names for SPOCK sensors.  These strings get prepended by
	"GetAppName() + "_" during runtime.  Order of the members of this array
	must be consistent with items in e_SpockSensorIds
*/
const char* iSpock::sm_DefaultPublishedVariables[NUM_SPOCK_SENSOR_IDS] =
											{	"COMPASS_HEADING",
												"COMPASS_YAW",
												"COMPASS_PITCH",
												"COMPASS_ROLL",
												"COMPASS_DIP",
												"DEPTH",
												"ACCEL_PITCH",
												"ACCEL_ROLL",
												"GPS_LONGITUDE",
												"GPS_LATITUDE",
												"GPS_VELOCITY",
												"GPS_HPE",
												"GPS_HEADING",
												"GPS_YAW",
												"GPS_HOURS",
												"GPS_MINUTES",
												"GPS_SECONDS",
												"WATERLEAKISDETECTED",
												"TEMPERATURE",
												"BATTERY_VOLTAGE"
											};




//=============================================================================
iSpock::iSpock( void )
:	m_AppIsOnline(false),
 	m_pSpock(NULL),
 	m_OnlyPublishChanges(false),
 	m_Verbosity(0)
{
	// Initialize default published variable names and values
	for (int i = 0; i < NUM_SPOCK_SENSOR_IDS; i++)
	{
		m_PublishedVarNames[i] = sm_DefaultPublishedVariables[i];
		m_PublishedVarValues[i] = 0.0;
	}

	EnableCommandMessageFiltering(true);	// Enable command messages
}





//=============================================================================
iSpock::~iSpock()
{
	// Release BunnySock node resources
	if (m_pSpock != NULL)	delete m_pSpock;
}





//=============================================================================
bool iSpock::OnNewMail(MOOSMSG_LIST & NewMail)
{
	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		// Walk through the list of received messages
		MOOSMSG_LIST::iterator iter;
		for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
		{
			// Identify skewed messages
			CMOOSMsg& RxMsg = *iter;
			if( RxMsg.IsSkewed( MOOSTime() ) )
			{
				MOOSTrace("\nDiscarding skewed mail: " +
						  RxMsg.GetKey() + " = " + RxMsg.GetString() + "\n");
				continue;
			}

			// Ignore received _CMD messages if command message filtering is
			// enabled to prevent us from processing the same message twice.
			/*if ( m_bCommandMessageFiltering == true )
			{
				if ( MOOSStrCmp(RxMsg.GetKey(), GetCommandKey()) )
				{
					continue;
				}
			}
			*/

			/*--------------------------------------------------------------
				<<< Parse incoming mail from the MOOS Database here >>>
			--------------------------------------------------------------*/
		}
	}
	else
	{
			/*--------------------------------------------------------------
				<<< Add behavior specific to simulation mode here >>>
			--------------------------------------------------------------*/
	}

	return true;
}



//=============================================================================
bool iSpock::OnCommandMsg(CMOOSMsg Msg)
{
	string sVal = Msg.GetString();

	// Ignore skewed commands!
	if ( !Msg.IsSkewed( MOOSTime()) )
	{

		if ( MOOSStrCmp(sVal, "AllSensorZero"))
		{
			m_pSpock->ZeroSensor(SpockModule::SensorId_All);
		}
		else if ( MOOSStrCmp(sVal, "DepthSensorZero"))
		{
			m_pSpock->ZeroSensor(SpockModule::SensorId_Depth);
		}
	}

	return true;
}



//=============================================================================
bool iSpock::Iterate( void )
{
	static bool SpockConnectedLastTime = false;
	static bool LeakDetectedLastTime = true;	// Set true to trigger update
	static bool IsFirstIterate = true;	// Always publish on first Iterate()
	bool SpockConnectedThisTime = m_pSpock->IsConnected();
	double d;
	string s;
	double t = HPMOOSTime();

	//--------------------------------------------
	// Publish changes in SPOCK conection status
	//--------------------------------------------
	if (SpockConnectedLastTime != SpockConnectedThisTime )
	{
		s = (SpockConnectedThisTime) ? "Connected" : "Disconnected";
		SetAppError( !SpockConnectedThisTime, ("SPOCK " + s) );
	}


	//-----------------------------------
	// Request sensors from SPOCK
	//-----------------------------------
	m_pSpock->RequestMultiSensors();


	//----------------------------------------------
	// If new sensor values have arrived, publish
	// any sensor values that have changed to the
	// MOOS database
	//----------------------------------------------
	if ( m_pSpock->SensorValuesAreFresh())
	{
		IsFirstIterate = false;

		for (int i = 0; i < NUM_SPOCK_SENSOR_IDS; i++)
		{
			// Don't publish to empty name, and skip leak detector reporting
			if ( !m_PublishedVarNames[i].empty() &&
				 (i != SENSOR_WATERLEAKISDETECTED) )
			{
				d = GetSpockSensorAsDouble(i);

				// Only publish values when they change
				if ( !m_OnlyPublishChanges ||
					 (m_PublishedVarValues[i] != d) ||
					 IsFirstIterate )
				{
					m_Comms.Notify(m_PublishedVarNames[i], d, t);
					m_PublishedVarValues[i] = d;
				}
			}
		}

		// Report leak detector state as a string
		bool LeakDetectedThisTime = m_pSpock->WaterLeakIsDetected();
		if (LeakDetectedThisTime != LeakDetectedLastTime)
		{
			s = (LeakDetectedThisTime) ? "TRUE" : "FALSE";
			m_Comms.Notify( m_PublishedVarNames[SENSOR_WATERLEAKISDETECTED],
							s, t);

			LeakDetectedLastTime = LeakDetectedThisTime;	// update state
		}

	}

	return true;
}



//=============================================================================
bool iSpock::OnStartUp( void )
{
	string s;
	string sBar40 = string(40, '=') + "\n";
	string sAppName = GetAppName();

	// Print a banner
	s = string(40, '=') + "\n";
	MOOSTrace(sBar40 +
			  MOOSFormat("iSpock v%3.2f\n", ISPOCK_VERSION) +
			  "Written by Dave Billin\n\n"
			  "  Live long... and prosper.\n" +
			  sBar40 + "\n\n");

	// Load mission file parameters.
	if ( !LoadMissionFileParameters() )
	{
		return false;
	}

	//-------------------------------------------------
	// Display variables sensors will be published to
	//-------------------------------------------------
	const char* szSensor[] = { "Compass Heading", "Compass Yaw",
			"Compass Pitch", "Compass Roll", "Compass Dip", "Depth (pressure)",
			"Pitch Accelerometer", "Roll Accelerometer", "GPS Longitude",
			"GPS Latitude", "GPS Velocity", "GPS HPE", "GPS Heading", "GPS Yaw",
			"GPS Hours", "GPS Minutes", "GPS Seconds", "Leak Status",
			"Temperature", "Battery Voltage" };
	MOOSTrace("Publishing sensors to the following MOOS variables:\n");
	for (int i = 0; i < NUM_SPOCK_SENSOR_IDS; i++)
	{
		s = string("  ") + szSensor[i] + " : " + m_PublishedVarNames[i] + "\n";
		MOOSTrace(s);
	}
	MOOSTrace("\n\n");

	SetAppError(true, "SPOCK Disconnected");

	//----------------------------------------
	// Create the object used to communicate
	// with the SPOCK module
	//----------------------------------------
	try
	{
	   MOOSTrace( "Connecting to SPOCK module " + m_SpockHostname
	              + MOOSFormat(":%d\n", m_SpockPort) );
		m_pSpock = new SpockModule( m_SpockHostname, m_SpockPort, m_Verbosity );
	}
	catch (CMOOSException& e)
	{
		return MOOSFail(e.c_str());		// Handle failure to create the object
	}

	m_AppIsOnline = true;
	OnConnectToServer();	// Register for MOOS variables

	return true;
}



//=============================================================================
bool iSpock::OnConnectToServer( void )
{
	if (m_AppIsOnline)
	{
		// Register to receive application commands
		m_Comms.Register( GetCommandKey(), 0);
	}

	return true;
}



//=============================================================================
bool iSpock::OnDisconnectFromServer( void )
{
	return true;
}



//=============================================================================
bool iSpock::LoadMissionFileParameters( void )
{
	string s;
	int IntVal;
	const char* szParamFail = "Failed to read required mission file "
							  "parameter %s\n";


	//----------------------------------------
	// Load required mission file parameters
	//----------------------------------------

	// Get the hostname of the SPOCK module
	if ( !m_MissionReader.GetConfigurationParam("SPOCK_HOSTNAME",
												m_SpockHostname) )
	{
		return MOOSFail(szParamFail, s.c_str());
	}

	// Get the network port to connect to on the SPOCK module
	if ( !m_MissionReader.GetConfigurationParam("SPOCK_PORT", IntVal) )
	{
		return MOOSFail(szParamFail, s.c_str());
	}
	m_SpockPort = static_cast<uint16_t>(IntVal);



	//----------------------------------------
	// Load optional mission file parameters
	//----------------------------------------

	// Loop through the entries of sm_DefaultPublishedVariables.  Append each
	// with "_PUBLISHTO" and attempt to load the corresponding parameter value
	// from the mission file.  If the parameter is given in the mission file,
	// the resulting value will override the default variable that the
	// corresponding sensor will be published to.
	for (int i = 0; i < NUM_SPOCK_SENSOR_IDS; i++)
	{
		s = sm_DefaultPublishedVariables[i] + string("_PUBLISHTO");
		m_MissionReader.GetConfigurationParam(s, m_PublishedVarNames[i]);
	}

	// Optional parameter to publish only sensors that have changed
	s = "PUBLISH_ONLY_CHANGES";
	m_MissionReader.GetConfigurationParam(s, m_OnlyPublishChanges);

	// Get verbosity level for debugging messages
	m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);

	return true;
}




//=============================================================================
double iSpock::GetSpockSensorAsDouble( int SensorId )
{
	double d;

	switch (SensorId)
	{
		case SENSOR_COMPASS_HEADING:
			d = m_pSpock->CompassHeading();
			break;

		case SENSOR_COMPASS_PITCH:
			d = m_pSpock->CompassPitch();
			break;

		case SENSOR_COMPASS_ROLL:
			d = m_pSpock->CompassRoll();
			break;

		case SENSOR_COMPASS_YAW:
			d = MOOS_ANGLE_WRAP( MOOSDeg2Rad( m_pSpock->CompassHeading() ));
			break;

		case SENSOR_COMPASS_DIP:
			d = m_pSpock->CompassDip();
			break;

		case SENSOR_DEPTH:
			d = m_pSpock->Depth();
			break;

		case SENSOR_ACCEL_PITCH:
			d = m_pSpock->AccelPitch();
			break;

		case SENSOR_ACCEL_ROLL:
			d = m_pSpock->AccelRoll();
			break;

		case SENSOR_GPS_LONGITUDE:
			d = m_pSpock->GpsLongitude();
			break;

		case SENSOR_GPS_LATITUDE:
			d = m_pSpock->GpsLatitude();
			break;

		case SENSOR_GPS_VELOCITY:
			d = m_pSpock->GpsVelocity();
			break;

		case SENSOR_GPS_HPE:
			d = m_pSpock->GpsHPE();
			break;

		case SENSOR_GPS_HEADING:
			d = m_pSpock->GpsHeading();
			break;

		case SENSOR_GPS_YAW:
			d = MOOS_ANGLE_WRAP( MOOSDeg2Rad( m_pSpock->GpsHeading()) );
			break;

		case SENSOR_GPS_HOURS:
			d = m_pSpock->GpsHours();
			break;

		case SENSOR_GPS_MINUTES:
			d = m_pSpock->GpsMinutes();
			break;

		case SENSOR_GPS_SECONDS:
			d = m_pSpock->GpsSeconds();
			break;

		case SENSOR_WATERLEAKISDETECTED:
			d = m_pSpock->WaterLeakIsDetected();
			break;

		case SENSOR_TEMPERATURE:
			d = m_pSpock->Temperature();
			break;

		case SENSOR_BATTERYVOLTS:
			d = m_pSpock->BatteryVoltage();
			break;
	}

	return d;
}

