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
/** @file pVehicleEKF.cpp

@brief
	Implementation of the pVehicleEKF object

@author Dave Billin

*/
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "pVehicleEKF.h"

using namespace std;
using namespace NEWMAT;
using namespace YellowSubNav;


/** Modes used for publishing from pVehicleEKF */
enum e_PublishModeIds
{
	PUBLISH_ESTIMATE = 0,	/**< Publishing estimated values from EKF */
	PUBLISH_SENSORS = 1		/**< Publishing sensor values at water surface */
};

/** Flag masks used to signal sensors that will trigger a heading update */
enum e_HeadingUpdateFlagMasks
{
	HEADINGUPDATE_COMPASS = 1,
	HEADINGUPDATE_VELOCITY = 2
};

/** Flag masks used to signal sensors that will trigger an IMU update */
enum e_ImuFlagMasks
{
	IMUUPDATE_WZ = 1,
	IMUUPDATE_WY = 2,
	IMUUPDATE_PITCH = 4,
	IMUUPDATE_ROLL = 8
};



// NOTE: elements in this array *must* directly correspond to the elements of
// e_SubscribedVariables
const char* pVehicleEKF::sm_DefaultSubscribedVarNames[] = {
										"GPS_LATITUDE",
										"GPS_LONGITUDE",
										"GPS_HEADING",
										"GPS_VELOCITY",
										"GPS_HPE",
										"DEPTH_SENSOR",
                                        "COMPASS_HEADING",
										"PITCH_SENSOR",
										"ROLL_SENSOR",
										"IMU_WZ",
										"IMU_WY",
										"LBL_2WAYTRAVELTIME",
										"LBL_RANGEPERIOD",
										"RPM_VELOCITY_ESTIMATE"
										};




// NOTE: this array of string constants *must* correspond in order and number
// of elements to e_PublishedVariables
const char* pVehicleEKF::sm_DefaultPublishedVarNames[] = {
										"NAV_X",
										"NAV_Y",
										"NAV_DEPTH",
										"NAV_HEADING",
										"NAV_YAW",
										"NAV_SPEED",
										"EST_YAW_BIAS",
										"NAV_PITCH",
										"NAV_ROLL"
										};




//=============================================================================
/* Creates an instance of the object */
pVehicleEKF::pVehicleEKF( void )
: m_AppIsInitialized(false),
  m_EKF(m_ReferenceGeodesy),
  m_DepthThreshold(0.0),
  m_GpsHPEThreshold(0.0)
{
	string s;

	// Populate default subscribed variable names
	for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
	{
		m_SubscribedVarNames[i] = sm_DefaultSubscribedVarNames[i];
	}

	// Populate default published variable names
	for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
	{
		m_PublishedVarNames[i] = sm_DefaultPublishedVarNames[i];
	}

}




//=============================================================================
/* Called when the object goes out of scope */
pVehicleEKF::~pVehicleEKF()
{

}






//=============================================================================
/* This function is where the application can do most of its work.
@details
	The rate at which Iterate() is called is determined by the value of the
	AppTick parameter specified in the (.moos) mission file.  The value of
	AppTick is loaded automatically when the application starts up.
*/
bool pVehicleEKF::Iterate( void )
{
	double t, N, E;
	static int LastPublishMode = -1;
	static double Start_Time = MOOSTime(false);
	double Current_Time;

	// Get current sensor values
	double GpsHPE = m_SubscribedVarTable[GPS_HPE]->GetDoubleVal();
	double Depth = m_SubscribedVarTable[DEPTH]->GetDoubleVal();
	double GpsLat = m_SubscribedVarTable[GPS_LATITUDE]->GetDoubleVal();
	double GpsLon = m_SubscribedVarTable[GPS_LONGITUDE]->GetDoubleVal();
	double Velocity = m_SubscribedVarTable[GPS_VELOCITY]->GetDoubleVal();
	t = HPMOOSTime();

	m_ReferenceGeodesy.LatLong2LocalGrid(GpsLat, GpsLon, N, E);
	if ((Velocity < 0) || (Velocity > 1.5))
	{
		Velocity = m_SubscribedVarTable[RPM_VELOCITY_ESTIMATE]->GetDoubleVal();
	}

	if ( (Depth <= m_DepthThreshold) &&
		 (GpsHPE <= m_GpsHPEThreshold) )
	{
		//------------------------------------------------
		// CASE: the vehicle is at the water's surface
		// and GPS HPE is below the configured threshold.
		// Publish values based on GPS and sensor readings
		//------------------------------------------------

		//-----------------------------
		// Handle switch to sensor mode
		//-----------------------------
		if (LastPublishMode != PUBLISH_SENSORS)	// Publish mode change
		{
			string sMode = GetAppName() + "_VehicleEkfMode";
			m_Comms.Notify(sMode, string("SENSORS"), t);
			LastPublishMode = PUBLISH_SENSORS;
		}

		//-----------------------------
		// Publish values from sensors
		//-----------------------------
		double YawBias = 0;	//m_PublishedVarTable[YAWBIAS]->GetDoubleVal();
		double Heading = m_SubscribedVarTable[COMPASS_HEADING]->GetDoubleVal();
		double Yaw = MOOS_ANGLE_WRAP(MOOSDeg2Rad(Heading));
		UpdatePublishedVariables( N, /* North coordinate */
								  E, /* East coordinate */
								  Velocity, /* Speed */
								  Yaw, /* Yaw */
								  YawBias,	/* Bias */
								  Depth, /* Depth */
								  t ); /* MOOSTime */

		m_PublishedVarTable[NAV_PITCH]->Set(
				m_SubscribedVarTable[PITCH]->GetDoubleVal(), t);
		m_PublishedVarTable[NAV_ROLL]->Set(
						m_SubscribedVarTable[ROLL]->GetDoubleVal(), t);
	}
	else
	{

		//------------------------------------------------
		// CASE: the vehicle is either below the water's
		// surface or GPS HPE is greater than the
		// configured threshold
		//------------------------------------------------

		// If dropping back into estimation mode, initialize the VehicleEKF
		if (LastPublishMode != PUBLISH_ESTIMATE)	// Publish mode change
		{											// and init the EKF
			string sMode = GetAppName() + "_VehicleEkfMode";
			m_Comms.Notify( sMode, string("ESTIMATED"), t);
			InitEKFObject(t);
			LastPublishMode = PUBLISH_ESTIMATE;

		}
		else
		{
			// If already in estimation mode, propagate the VehicleEKF
			m_EKF.Propagate(t);
		}

		//-----------------------------
		// Publish estimated values
		//-----------------------------
		const ColumnVector& EkfStates = m_EKF.GetStates();
		UpdatePublishedVariables( EkfStates(VehicleEKF::EST_COORD_N),
								  EkfStates(VehicleEKF::EST_COORD_E),
								  EkfStates(VehicleEKF::EST_SPEED),
								  EkfStates(VehicleEKF::EST_YAW),
								  EkfStates(VehicleEKF::EST_YAW_BIAS),
								  Depth,
								  t );

		m_PublishedVarTable[NAV_PITCH]->Set(
				m_SubscribedVarTable[PITCH]->GetDoubleVal(), t);
		m_PublishedVarTable[NAV_ROLL]->Set(
						m_SubscribedVarTable[ROLL]->GetDoubleVal(), t);
	}
	//8 hz
	Current_Time = MOOSTime(false);
	double Elapsed_Time = Current_Time - Start_Time;
	if (Elapsed_Time > .125)
	{
	PublishFreshMOOSVariables();	// Publish any updated MOOS variables
	Start_Time = MOOSTime(false);
	}
	return true;
}




//=============================================================================
/* Called when new mail has arrived from the MOOS Database.
@details
    This method will be called at a rate of approximately 1/CommsTick Hz.
	In this function you'll most likely interate over the collection of
	mail messages received or call a m_Comms::PeekMail() to look for a
	specific named message.

@param NewMail
	A reference to the list of received mail from the MOOS Database

@return
	true on success; false on error
*/
bool pVehicleEKF::OnNewMail(MOOSMSG_LIST & NewMail)
{
	string sMsgKey, sVal;

	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		// Update subscribed dynamic MOOS variables
		// (and apply skew-filtering)
		if ( !UpdateMOOSVariables(NewMail) )
		{
			return MOOSFail("Failed to update MOOS variables!\n");
		}


		//=======================================
		// If any IMU values were received,
		// apply an IMU measurement to the EKF
		//=======================================
		if ( m_SubscribedVarTable[WZ]->IsFresh() ||
			 m_SubscribedVarTable[WY]->IsFresh() ||
			 m_SubscribedVarTable[PITCH]->IsFresh() ||
			 m_SubscribedVarTable[ROLL]->IsFresh())
		{
			double wz = m_SubscribedVarTable[WZ]->GetDoubleVal();
			double wy = m_SubscribedVarTable[WY]->GetDoubleVal();
			double Pitch = m_SubscribedVarTable[PITCH]->GetDoubleVal();
			double Roll = m_SubscribedVarTable[ROLL]->GetDoubleVal();
			ImuMeasurement m(wz, wy, Pitch, Roll);
			m_EKF.UpdateMeasurement(m);
		}


		//=======================================
		// If heading or speed changed, apply a
		// heading/speed update to the EKF
		//=======================================
		if ( m_SubscribedVarTable[COMPASS_HEADING]->IsFresh() ||
			 m_SubscribedVarTable[GPS_VELOCITY]->IsFresh() ||
			 m_SubscribedVarTable[RPM_VELOCITY_ESTIMATE]->IsFresh() )
		{
			double Heading =
					m_SubscribedVarTable[COMPASS_HEADING]->GetDoubleVal();
			double Velocity =
					m_SubscribedVarTable[GPS_VELOCITY]->GetDoubleVal();

			// Use GPS velocity by default
			// If GPS velocity is out-of-bounds, use RPM-based velocity
			if ((Velocity < 0.0) || (Velocity > 1.5))
			{
				Velocity =
				   m_SubscribedVarTable[RPM_VELOCITY_ESTIMATE]->GetDoubleVal();
			}

			// Update heading and speed measurements
			HeadingMeasurement mHeading(Heading);
			m_EKF.UpdateMeasurement(mHeading);

			SpeedMeasurement mSpeed(Velocity);
			m_EKF.UpdateMeasurement(mSpeed);

		}

		//========================================
		// If LBL PING travel times were received,
		// apply a corresponding measurement to
		// the EKF
		//========================================
		if ( m_SubscribedVarTable[LBL_2WAYTRAVELTIME]->IsFresh() )
		{
			// Extract beacon ranges into a measurement object
			Real BeaconRanges[4] = {0.0, 0.0, 0.0, 0.0};
			string sRanges =
					m_SubscribedVarTable[LBL_2WAYTRAVELTIME]->GetStringVal();

			MOOSValFromString(BeaconRanges[0], sRanges, "TimeA", true);
			MOOSValFromString(BeaconRanges[1], sRanges, "TimeB", true);
			MOOSValFromString(BeaconRanges[2], sRanges, "TimeC", true);
			MOOSValFromString(BeaconRanges[3], sRanges, "TimeD", true);

			// Apply the range measurement
			double Depth = m_SubscribedVarTable[DEPTH]->GetDoubleVal();
			LblBeaconMeasurement m(BeaconRanges, Depth);
			m_EKF.UpdateMeasurement(m);

			// Report bad ranges
			if (m_EKF.BadRangesFlagIsSet())
			{
				string sName = GetAppName() + "_EKFError";
				m_Comms.Notify( sName, "bad ranges flag set" );
			}
		}

	}
	else
	{
		// Simulate received positions here...
	}

	//-------------------------------------
	// Report VehicleEKF error conditions
	//-------------------------------------
	if (m_EKF.InvertFailed())		// Failed to invert a matrix
	{
		string sName = GetAppName() + "_EKFError";
		m_Comms.Notify( sName, "Matrix invert failed" );
		m_EKF.ResetInvertFlag();
	}
	if (m_EKF.BadJumpFlagIsSet())	// Estimated position jumped more than allowed
	{
		string sName = GetAppName() + "_EKFError";
		m_Comms.Notify( sName, "Bad position jump" );
		m_EKF.ResetBadJumpFlag();
	}
	return true;
}




//=============================================================================
void pVehicleEKF::InitEKFObject(double t)
{
	double N, E;
	double GpsLat = m_SubscribedVarTable[GPS_LATITUDE]->GetDoubleVal();
	double GpsLon = m_SubscribedVarTable[GPS_LONGITUDE]->GetDoubleVal();
	double Velocity = m_SubscribedVarTable[GPS_VELOCITY]->GetDoubleVal();
	double CompassHeading = m_SubscribedVarTable[COMPASS_HEADING]->GetDoubleVal();
	double LblPingPeriod = m_SubscribedVarTable[LBL_RANGEPERIOD]->GetDoubleVal();

	m_ReferenceGeodesy.LatLong2LocalGrid(GpsLat, GpsLon, N, E);

	if ((Velocity < 0) || (Velocity > 1.5))
	{
		Velocity = m_SubscribedVarTable[RPM_VELOCITY_ESTIMATE]->GetDoubleVal();
	}

	m_EKF.Initialize( E, N, Velocity, CompassHeading,
	                  static_cast<uint32_t>(LblPingPeriod), t);
}




//=============================================================================
void pVehicleEKF::UpdatePublishedVariables( double N, double E, double Speed,
											double Yaw, double Bias,
											double Depth, double t )
{
	CMOOSVariable* pNavY = m_PublishedVarTable[NAV_Y];
	CMOOSVariable* pNavX = m_PublishedVarTable[NAV_X];
	CMOOSVariable* pNavSpeed = m_PublishedVarTable[NAV_SPEED];
	CMOOSVariable* pNavYaw = m_PublishedVarTable[NAV_YAW];
	CMOOSVariable* pYawBias = m_PublishedVarTable[YAWBIAS];
	CMOOSVariable* pNavDepth = m_PublishedVarTable[NAV_DEPTH];

	// Update North coordinate
	if ( (pNavY->GetDoubleVal() != N) || (pNavY->GetAge(t) > 1.0) )
	{
		pNavY->Set(N, t);
	}

	// Update East coordinate
	if ( (pNavX->GetDoubleVal() != E) || (pNavX->GetAge(t) > 1.0) )
	{
		pNavX->Set(E, t);
	}

	// Update speed
	if ( (pNavSpeed->GetDoubleVal() != Speed) ||
		 (pNavSpeed->GetAge(t) > 1.0) )
	{
		pNavSpeed->Set(Speed, t);
	}

	// Update yaw and heading
	if ( (pNavYaw->GetDoubleVal() != Yaw) ||
		 (pNavYaw->GetAge(t) > 1.0) )
	{
		pNavYaw->Set(Yaw, t);
        double d = MOOSRad2Deg(Yaw);
        d = (d < 0.0) ? (d + 360.0) : d;
		m_PublishedVarTable[NAV_HEADING]->Set(d,t);
	}

	// Update yaw bias
	if ( (pYawBias->GetDoubleVal() != Bias) ||
		 (pYawBias->GetAge(t) > 1.0) )
	{
		pYawBias->Set(Bias, t);
	}

	// Update depth
	if ( (pNavDepth->GetDoubleVal() != Depth) ||
		 (pNavDepth->GetAge(t) > 1.0) )
	{
		pNavDepth->Set(Depth, t);
	}

}





//=============================================================================
/* If command message filtering is enabled for this application (via the
	EnableCommandMessageFiltering() function), then this function is called
	when a command message having the identifier
	PPOSITIONAGENT_CMD is recieved from the MOOS Database.

@param Msg
	A copy of the received command message

@return
	true on success; false on error

@see CMOOSApp::EnableCommandMessageFiltering
*/
bool pVehicleEKF::OnCommandMsg(CMOOSMsg Msg)
{
	string sMsg = Msg.GetAsString();
	string sCommand, sVal;

	if ( Msg.IsSkewed(MOOSTime()) )
	{
		return true;
	}

	// Get the command ID
	if ( MOOSValFromString( sCommand, sMsg, "Cmd", true ) )
	{

	}

	return true;
}










//=============================================================================
/* Called when the application first starts up
@details
	This function is called as the application first starts up before any
	calls to Iterate() begin.
*/
bool pVehicleEKF::OnStartUp()
{
	string sBar80 = string(79, '-') + "\n";
	string sBar50 = string(50,'-') + "\n";
	double Lat, Long, Depth;

	m_MOOSVars.clear();

	//----------------------------------
	// Print greeting banner
	//----------------------------------
	MOOSTrace( "\n<<< pVehicleEKF v%3.2f >>>\n\n",
			   APPLICATION_SOFTWARE_VERSION );


	//----------------------------------
	// Load mission file parameters
	//----------------------------------
	MOOSTrace("Loading mission file parameters...\n\n\n");

	if ( !LoadMissionParameters() )
	{
		return false;	// Pass along failure
	}

	//---------------------------------------------
	// Add Dynamic MOOS variables for subscribing
	//---------------------------------------------
	for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
	{
		bool VarAdded =  AddMOOSVariable( m_SubscribedVarNames[i],
								   m_SubscribedVarNames[i],
								   "", 0 );

		m_SubscribedVarTable[i] = GetMOOSVar(m_SubscribedVarNames[i]);

		// Verify that we created the variable
		if ( !VarAdded || (m_SubscribedVarTable[i] == NULL) )
		{
			return MOOSFail("Failed to create MOOS variable for subscribing "
							"to %s!\n", m_SubscribedVarNames[i].c_str() );
		}
		else
		{
			// Give numeric subscribed variables some default values
			if (i != LBL_2WAYTRAVELTIME)
			{
				m_SubscribedVarTable[i]->Set(0.0, 0.0);
			}
		}
	}
	// Init LBL time as string value
	m_SubscribedVarTable[LBL_2WAYTRAVELTIME]->Set("", 0.0);


	//---------------------------------------------
	// Add Dynamic MOOS variables for publishing
	//---------------------------------------------
	for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
	{
		bool VarAdded =  AddMOOSVariable( m_PublishedVarNames[i],
										  "", m_PublishedVarNames[i], 0 );

		m_PublishedVarTable[i] = GetMOOSVar(m_PublishedVarNames[i]);

		// Verify that we created the variable
		if ( !VarAdded || (m_PublishedVarTable[i] == NULL) )
		{
			return MOOSFail("Failed to create MOOS variable for publishing "
							"to %s!\n", m_PublishedVarNames[i].c_str() );
		}
	}


	//---------------------------------------------
	// Print LBL Beacon info
	//---------------------------------------------
	MOOSTrace(sBar50);
	MOOSTrace(" LBL Transponder Beacon Locations:\n");
	MOOSTrace(sBar50);
	MOOSTrace("| Beacon |   Lat      |   Long     |  Depth     |\n"
			  "--------------------------------------------------\n" );
	for (char BeaconId = 'A'; BeaconId <= 'D'; BeaconId++)
	{
		m_EKF.NavBeacon(BeaconId).GetLocation(Lat, Long, Depth);
		MOOSTrace( "| %-6c | %-10f | %-10f | %-10f |\n",
				   BeaconId, Lat, Long, Depth );
	}
	MOOSTrace(sBar50 + "\n\n");

	MOOSTrace( "Speed of sound in water: %f meters/sec\n\n",
			   m_EKF.GetH20SoundVelocity() );
	MOOSTrace("\n" + sBar80 + "\n");


	MOOSTrace("Local coordinate system origin is located at:\n"
			  "\tLatitude  = %f\n"
			  "\tLongitude = %f\n",
			  m_ReferenceGeodesy.GetOriginLatitude(),
			  m_ReferenceGeodesy.GetOriginLongitude() );

	MOOSTrace(sBar80 + "\n\n");

	MOOSTrace("---------------------------------\n"
			  "*** Subscribed MOOS Variables ***\n"
			  "---------------------------------\n");
	for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
	{
		MOOSTrace( m_SubscribedVarNames[i] + "\n");
	}

	MOOSTrace("\n"
			  "--------------------------------\n"
			  "*** Published MOOS Variables ***\n"
			  "--------------------------------\n");

	for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
	{
		MOOSTrace( m_PublishedVarNames[i] + "\n");
	}

	MOOSTrace("\n\n<<< VehicleEKF is online >>>\n\n");

	m_AppIsInitialized = true;
	OnConnectToServer();
	return true;
}




//=============================================================================
/* Called when the application connects to the MOOS Database. */
bool pVehicleEKF::OnConnectToServer( void )
{
	if (m_AppIsInitialized)
	{
		RegisterMOOSVariables();	// Subscribe to our MOOS variables
	}
	return true;
}




//=============================================================================
/* Called when the application is disconnected from the MOOS Database. */
bool pVehicleEKF::OnDisconnectFromServer( void )
{
	/*-----------------------------------------------------
		The application is now disconnected from the
		MOOS Database.
	-----------------------------------------------------*/
	return true;
}




//=============================================================================
bool pVehicleEKF::LoadMissionParameters( void )
{
	string s;
	double d, Lat, Lon, Depth;
	//const char* szBeaconParam[] = { "Latitude", "Longitude", "Depth" };
	//const char* szAltBeaconParam[] = { "North", "East", "Depth" };
	const char* szRequiredParamError = "ERROR: Failed to load required mission "
									   "file parameter %s!";
	const char szGlobalParamError[] = "ERROR: Failed to load required global "
									  "mission file parameter %s!";

	//---------------------------------------------------------------
	// Load REQUIRED global parameter - coordinate system origin
	//---------------------------------------------------------------
	s = "LatOrigin";

	if (!m_MissionReader.GetValue(s, Lat))
	{
		return MOOSFail(szGlobalParamError, s.c_str());
	}

	s = "LongOrigin";
	if (!m_MissionReader.GetValue(s, Lon))
	{
		return MOOSFail(szGlobalParamError, s.c_str());
	}

	// Initialize coordinate system origin
	m_ReferenceGeodesy.Initialise(Lat, Lon);




	//---------------------------------------------------------------
	// Load REQUIRED global parameters - LBL beacon locations
	//---------------------------------------------------------------
	for (char BeaconId = 'A'; BeaconId <= 'D'; BeaconId++)
	{
		s = string("Beacon") + BeaconId;
		LblBeacon& NavBeacon = m_EKF.NavBeacon(BeaconId);

		// Try to read beacon location using longitude/latitude parameters
		if ( m_MissionReader.GetValue(s + "_Longitude", Lon) &&
			 m_MissionReader.GetValue(s + "_Latitude", Lat) &&
			 m_MissionReader.GetValue(s + "_Depth", Depth) )
		{
			NavBeacon.SetLocation(Lat, Lon, Depth);
		}
		else if ( m_MissionReader.GetValue(s + "_East", Lon) &&
				 m_MissionReader.GetValue(s + "_North", Lat) &&
				 m_MissionReader.GetValue(s + "_Depth", Depth) )
		{
			NavBeacon.SetLocation_LocalGrid(Lon, Lat, Depth);
		}
		else
		{
			return MOOSFail("Mission file is missing one or more global "
							"parameters for LBL Beacon location %c!\n",
							BeaconId);
		}
	}


	//------------------------------------------------------
	// Load REQUIRED global parameter - speed of sound in water
	//------------------------------------------------------
	s = "SSH20";
	if (!m_MissionReader.GetValue(s, d))
	{
		return MOOSFail(szRequiredParamError, s.c_str());
	}
	m_EKF.SetH20SoundVelocity( static_cast<float>(d) );


	//------------------------------------------------------
	// Load EKF threshold values
	//------------------------------------------------------
	s = "EKFDepthThreshold";
	if (!m_MissionReader.GetConfigurationParam(s, d))
	{
		return MOOSFail(szRequiredParamError, s.c_str());
	}
	m_DepthThreshold = d;

	s = "GPS_HPE_Threshold";
	if (!m_MissionReader.GetConfigurationParam(s, d))
	{
		return MOOSFail(szRequiredParamError, s.c_str());
	}
	m_GpsHPEThreshold = d;




	//------------------------------------------------------
	// Load optional nnn_SUBSCRIBETO parameters
	//------------------------------------------------------
	for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
	{
		s = string(sm_DefaultSubscribedVarNames[i]) + "_SUBSCRIBETO";
		m_MissionReader.GetConfigurationParam(s, m_SubscribedVarNames[i]);
	}


	//------------------------------------------------------
	// Load optional nnn_PUBLISHTO parameters
	//------------------------------------------------------
	for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
	{
		s = string(sm_DefaultPublishedVarNames[i]) + "_PUBLISHTO";
		m_MissionReader.GetConfigurationParam(s, m_PublishedVarNames[i]);
	}

	return true;
}

