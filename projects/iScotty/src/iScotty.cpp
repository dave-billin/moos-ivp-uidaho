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
/** @file iScotty.cpp

@brief
	Implementation of the iScotty application object

@author Dave Billin

*/
//=============================================================================

#include <cmath>
#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "AppVersion.h"
#include "iScotty.h"

using namespace std;


//===============================
// Implement verbosity macros
//===============================
#define VERBOSE1( _expr_ )	if (m_Verbosity > 1) {_expr_;}
#define VERBOSE2( _expr_ )	if (m_Verbosity > 2) {_expr_;}
#define VERBOSE3( _expr_ )	if (m_Verbosity > 3) {_expr_;}

const char* iScotty::sm_SubscribedVariableNames[NUM_SUBSCRIBED_VARIABLES] =
				{
					"NAV_YAW",			/* Indexed by Idx_NavYaw */
					"NAV_SPEED", 		/* Indexed by Idx_NavSpeed */
					"NAV_DEPTH", 		/* Indexed by Idx_NavDepth */
					"NAV_PITCH",		/* Indexed by Idx_NavPitch */
					"NAV_ROLL",			/* Indexed by Idx_NavRoll */

					"DESIRED_HEADING",  /* Indexed by Idx_DesiredHeading */
					/*"DESIRED_YAW",*/ 		/* Indexed by Idx_DesiredYaw */
					"DESIRED_SPEED", 	/* Indexed by Idx_DesiredSpeed */
					"DESIRED_DEPTH",	/* Indexed by Idx_DesiredDepth */
					"DESIRED_PITCH",	/* Indexed by Idx_DesiredPitch */
					"DESIRED_ROLL", 	/* Indexed by Idx_DesiredRoll */

					"DESIRED_RUDDER", 	/* Indexed by Idx_DesiredRudder */
					"DESIRED_ELEVATOR",	/* Indexed by Idx_DesiredElevator */
					"DESIRED_AILERON", 	/* Indexed by Idx_DesiredAileron */
					"DESIRED_THRUST",	/* Indexed by Idx_DesiredThrust */

					"MAX_ALLOWED_DEPTH",	/* Indexed by Idx_MaxDepth */

					"YAW_PID_Kp",	/* Indexed by Idx_YawPid_Kp */
					"YAW_PID_Ki",	/* Indexed by Idx_YawPid_Ki */
					"YAW_PID_Kd",	/* Indexed by Idx_YawPid_Kd */
					"YAW_PID_IntSat",/* Indexed by Idx_YawPid_IntSat */
					"YAW_PID_OutSat",/* Indexed by Idx_YawPid_OutSat */

					"SPEED_PID_Kp",	/* Indexed by Idx_SpeedPid_Kp */
					"SPEED_PID_Ki",	/* Indexed by Idx_SpeedPid_Ki */
					"SPEED_PID_Kd",	/* Indexed by Idx_SpeedPid_Kd */
					"SPEED_PID_IntSat",/* Indexed by Idx_SpeedPid_IntSat */
					"SPEED_PID_OutSat",/* Indexed by Idx_SpeedPid_OutSat */

					"DEPTH_PID_Kp",	/* Indexed by Idx_DepthPid_Kp */
					"DEPTH_PID_Ki",	/* Indexed by Idx_DepthPid_Ki */
					"DEPTH_PID_Kd",	/* Indexed by Idx_DepthPid_Kd */
					"DEPTH_PID_IntSat",/* Indexed by Idx_DepthPid_IntSat */
					"DEPTH_PID_OutSat",/* Indexed by Idx_DepthPid_OutSat */

					"ROLL_PID_Kp",	/* Indexed by Idx_RollPid_Kp */
					"ROLL_PID_Ki",	/* Indexed by Idx_RollPid_Ki */
					"ROLL_PID_Kd",	/* Indexed by Idx_RollPid_Kd */
					"ROLL_PID_IntSat",/* Indexed by Idx_RollPid_IntSat */
					"ROLL_PID_OutSat",/* Indexed by Idx_RollPid_OutSat */

					"PITCH_PID_Kp",	/* Indexed by Idx_PitchPid_Kp */
					"PITCH_PID_Ki",	/* Indexed by Idx_PitchPid_Ki */
					"PITCH_PID_Kd",	/* Indexed by Idx_PitchPid_Kd */
					"PITCH_PID_IntSat",/* Indexed by Idx_PitchPid_IntSat */
					"PITCH_PID_OutSat",/* Indexed by Idx_PitchPid_OutSat */
					"PITCH_PID_KpDepth"/* Indexed by Idx_PitchPid_KpDepth */
				};

const char* iScotty::sm_szPidParamNames[NUM_PID_PARAMETERS] =
				{
					"Kp", "Ki", "Kd", "IntSat", "OutSat", "KpDepth"
				};

const char* iScotty::sm_szControllerNames[5] =
				{ "YAW", "SPEED", "DEPTH", "ROLL", "PITCH" };



static double GetMoosTimeAtMidnight( void )
{
	time_t t;
	struct tm MidnightTime;
	struct tm* pLocalTime;

	time( &t );	// Get the raw time
	pLocalTime = localtime( &t );	// Translate to local time struct

	// Get a local copy of the time
	memcpy( &MidnightTime, pLocalTime, sizeof(struct tm) );

	// Set up the time at midnight
	MidnightTime.tm_hour = 0;
	MidnightTime.tm_min = 0;
	MidnightTime.tm_sec = 0;

	// Return MOOSTime (epochal time) of the current day at midnight
	return static_cast<double>( mktime( &MidnightTime ) );
}



//=============================================================================
/** Creates an instance of the object */
iScotty::iScotty( void )
:	m_AppIsInitialized(false),
	m_pScotty(NULL),
 	m_ScottyPort(0),
 	m_YawControlIsEnabled(true),
 	m_SpeedControlIsEnabled(true),
 	m_DepthControlIsEnabled(true),
 	m_RollControlIsEnabled(true),
 	m_PitchControlIsEnabled(true),
 	m_PropIsEnabled(true),
	m_HelmIsActive(false),
	m_DesiredActuatorValuesChanged(false),
	m_ServoCenterRudder(128),
	m_ServoCenterElevL(128),
	m_ServoCenterElevR(128),
	m_CouplingCoeff(0.5),
	m_Verbosity(0),
	m_VelocityPerRPM(0.0),
	m_VelocityPerRPMOffset(0.0)
{
	// Initialize subscribed variable values
	for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
	{
		m_SubscribedVarValueTable[i] = 0.0;
		m_SubscribedVarChanged[i] = false;
	}

	// Initialize controller table
	m_ControllerTable[0] = &m_YawPID;
	m_ControllerTable[1] = &m_SpeedPID;
	m_ControllerTable[2] = &m_DepthPID;
	m_ControllerTable[3] = &m_RollPID;
	m_ControllerTable[4] = &m_PitchController;

	// Disable PID logging by default
	m_YawPID.SetLog(false);
	m_SpeedPID.SetLog(false);
	m_DepthPID.SetLog(false);
	m_RollPID.SetLog(false);

	EnableCommandMessageFiltering(true);	// Enable command messages
}



//=============================================================================
/** Called when the object goes out of scope */
iScotty::~iScotty()
{
	// Release BunnySock node resources
	if (m_pScotty != NULL)	delete m_pScotty;
}



//=============================================================================
bool iScotty::OnNewMail(MOOSMSG_LIST & NewMail)
{
	double dMoosTime = MOOSTime();

	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		//--------------------------------------------------------
		// Update dynamic MOOS variables.
		// NOTE: variables are only updated from non-skewed mail!
		//--------------------------------------------------------
		UpdateMOOSVariables(NewMail);

		//---------------------------------------------
		// Parse mail from the MOOSDB
		//---------------------------------------------
		MOOSMSG_LIST::iterator iter;
		for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
		{
			CMOOSMsg& RxMsg = *iter;
			string sMsgKey = RxMsg.GetKey();
			string sVal;

			//-----------------------------
			// *** HELM MANUAL OVERRIDE ***
			//-----------------------------
			// We always read manual override value regardless of skew
			if ( MOOSStrCmp(sMsgKey, "MOOS_MANUAL_OVERIDE") ||
				 MOOSStrCmp(sMsgKey, "MOOS_MANUAL_OVERRIDE") )
			{
				sVal = RxMsg.GetString();
				MOOSToUpper(sVal);
				m_HelmIsActive = !MOOSStrCmp(sVal, "TRUE");
				continue;
			}

			// Receive actuator trims
			else if ( MOOSStrCmp(sMsgKey, "SCOTTY_SERVO_TRIMS") )
			{
				sVal = RxMsg.GetString();
				SetTrimsFromMOOSMail(sVal);
			}

			// Receive (skew-filtered) indexed variables
			else
			{
				// Don't update desired values with mail that is skewed
				// or mail that we sent ourselves
				if ( !iter->IsSkewed(dMoosTime) &&
					 !MOOSStrCmp(iter->GetSource(), "iScotty") )
				{
					for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
					{
						if (MOOSStrCmp(sMsgKey, sm_SubscribedVariableNames[i]))
						{
							m_SubscribedVarValueTable[i] = iter->GetDouble();
							m_SubscribedVarChanged[i] = true;
						}
					}
				}
			}
		}

		//& DEBUG
		/*if (m_SubscribedVarChanged[Idx_DesiredThrust])
		{
			MOOSTrace("*** Gozintas: %f ***\n",
					  m_SubscribedVarValueTable[Idx_DesiredThrust]);
		}
		*/


		//--------------------------------------------
		// Handle subscribed variable value changes
		//--------------------------------------------
		UpdateControllerParameters();
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
bool iScotty::OnCommandMsg(CMOOSMsg Msg)
{
	const char* szBool[2] = {"Enabled", "Disabled"};
	string s;

	// Ignore skewed application commands!
	if ( !Msg.IsSkewed(MOOSTime()) )
	{
		string sVal = Msg.GetString();
		bool b;

		VERBOSE1( MOOSTrace("Received Command message: " + sVal + "\n") );

		if ( MOOSValFromString(b, sVal, "YawControlIsEnabled") )
		{
			m_YawControlIsEnabled = b;
			s = MOOSFormat("Yaw controller is %s\n",
							((b) ? szBool[0] : szBool[1]) );
		}
		else if ( MOOSValFromString(b, sVal, "SpeedControlIsEnabled") )
		{
			m_SpeedControlIsEnabled = b;
			s = MOOSFormat("Speed controller is %s\n", 
							((b) ? szBool[0] : szBool[1]) );
		}
		else if ( MOOSValFromString(b, sVal, "DepthControlIsEnabled") )
		{
			m_DepthControlIsEnabled = b;
			s = MOOSFormat("Depth controller is %s\n", 
							((b) ? szBool[0] : szBool[1]) );
		}
		else if ( MOOSValFromString(b, sVal, "RollControlIsEnabled") )
		{
			m_RollControlIsEnabled = b;
			s = MOOSFormat("Roll controller is %s\n", 
							((b) ? szBool[0] : szBool[1]) );
		}
		else if ( MOOSValFromString(b, sVal, "PitchControlIsEnabled") )
		{
			m_PitchControlIsEnabled = b;
			s = MOOSFormat("Pitch controller is %s\n", 
							((b) ? szBool[0] : szBool[1]) );
		}
		else if ( MOOSStrCmp(sVal, "ResetPIDs") )
		{
			s = "Reset all controllers\n";
            ResetControllers();
		}
		else if ( MOOSValFromString(b, sVal, "PropIsEnabled") )
		{
			m_PropIsEnabled = b;
			s = MOOSFormat("Propeller is %s\n",
							((b) ? szBool[0] : szBool[1]) );
		}

	}

	if (m_Verbosity >= 1)
	{
		MOOSTrace(s);
	}
	return true;
}



//=============================================================================
bool iScotty::Iterate( void )
{
	static bool ScottyConnectedLastTime = false;
	static bool HelmActiveLastTime = false;
	bool ScottyConnectedThisTime = m_pScotty->IsConnected();
	double t = MOOSTime();
	CMOOSVariable* pVar;


	//--------------------------------------
	// Handle changes in SCOTTY connection
	//--------------------------------------
	if ( ScottyConnectedLastTime != ScottyConnectedThisTime )
	{
		// CASE: Connection with SCOTTY opened
		if (ScottyConnectedThisTime == true)
		{
			// Send servo center settings to SCOTTY
			m_pScotty->SetServoCenters( m_ServoCenterRudder,
										m_ServoCenterElevL,
										m_ServoCenterElevR,
										m_CouplingCoeff );
		}

		// CASE: Connection with SCOTTY closed
		else
		{
			ResetControllers();	// Reset all controllers
		}

		string s = (ScottyConnectedThisTime) ? "Connected" : "Disconnected";
		SetAppError(!ScottyConnectedThisTime, ("SCOTTY " + s) );
		ScottyConnectedLastTime = ScottyConnectedThisTime;
	}




	//--------------------------------------
	// Handle changes in Helm engagement
	//--------------------------------------
	if ( HelmActiveLastTime ^ m_HelmIsActive )
	{
		if (m_HelmIsActive == true)
		{
			// Helm has ben engaged - reset all controllers
			ResetControllers();
		}
		else
		{
			// STATUS:
			// Helm has been disengaged - zero actuator values
			m_SubscribedVarValueTable[Idx_DesiredRudder] = 0.0;
			m_SubscribedVarValueTable[Idx_DesiredElevator] = 0.0;
			m_SubscribedVarValueTable[Idx_DesiredAileron] = 0.0;
			m_SubscribedVarValueTable[Idx_DesiredThrust] = 0.0;
		}
	}
	HelmActiveLastTime = m_HelmIsActive;


	/*if (m_SubscribedVarChanged[Idx_DesiredThrust])
	{
		MOOSTrace("*** Gozintas (Iterate): %f ***\n",
				  m_SubscribedVarValueTable[Idx_DesiredThrust]);
	}
	*/

	//---------------------------------------------------------------------
	// Run PID controllers if Helm is active (i.e. if MOOS_MANUAL_OVERRIDE
	// is FALSE) and SCOTTY is connected.
	//
	// NOTE: the controllers store their output to members of
	// 		 m_SubscribedVarValueTable corresponding to DESIRED_RUDDER,
	//		 DESIRED_ELEVATOR, etc., effectively overriding any direct
	//		 actuator commands received from the MOOSDB
	//---------------------------------------------------------------------
	if ( m_HelmIsActive && m_pScotty->IsConnected() )
	{
		RunPIDControllers();

		// Set the flag to send actuator settings to SCOTTY
		m_DesiredActuatorValuesChanged = true;
	}


	/*if (m_SubscribedVarChanged[Idx_DesiredThrust])
	{
		MOOSTrace("*** Gozouttas: %f ***\n",
				  m_SubscribedVarValueTable[Idx_DesiredThrust]);
	}
	*/

	//----------------------------
	// Update actuator settings
	//----------------------------
	// Handle changes to desired actuator settings.
	if ( m_SubscribedVarChanged[Idx_DesiredRudder] ||
		 m_SubscribedVarChanged[Idx_DesiredElevator] ||
		 m_SubscribedVarChanged[Idx_DesiredAileron] ||
		 m_SubscribedVarChanged[Idx_DesiredThrust] )
	{
		float Thrust, Rudder, Elevator, Aileron;


		if (m_PropIsEnabled)
		{
			Thrust = (m_SubscribedVarChanged[Idx_DesiredThrust]) ?
					m_SubscribedVarValueTable[Idx_DesiredThrust] : 
					m_pScotty->Thrust();
		}
		else
		{
			Thrust = 0.0;
		}

		Rudder = (m_SubscribedVarChanged[Idx_DesiredRudder]) ?
						   	m_SubscribedVarValueTable[Idx_DesiredRudder] : 
							m_pScotty->RudderAngle();

		Elevator = (m_SubscribedVarChanged[Idx_DesiredElevator]) ?
					   	m_SubscribedVarValueTable[Idx_DesiredElevator] : 
						m_pScotty->ElevatorAngle();

		Aileron = (m_SubscribedVarChanged[Idx_DesiredAileron]) ?
					   	m_SubscribedVarValueTable[Idx_DesiredAileron] : 
						m_pScotty->AileronAngle();

		// Reset changes
		m_SubscribedVarChanged[Idx_DesiredThrust] = false;
		m_SubscribedVarChanged[Idx_DesiredElevator] = false;
		m_SubscribedVarChanged[Idx_DesiredAileron] = false;
		m_SubscribedVarChanged[Idx_DesiredRudder] = false;
		m_pScotty->SetActuators( Rudder, Elevator, Aileron, Thrust );
	}
	m_DesiredActuatorValuesChanged = false;


	double PropRpm = m_pScotty->PropellerRpm();
	pVar = GetMOOSVar("PropRPM");
	pVar->Set( static_cast<double>(PropRpm), t );

	double VelEst = (PropRpm * m_VelocityPerRPM + m_VelocityPerRPMOffset);
	if (VelEst < 0.0)
	{
		VelEst = 0.0;
	}
	pVar = GetMOOSVar("RPM_VELOCITY_ESTIMATE");
	pVar->Set(VelEst, t );


	PublishFreshMOOSVariables();
	return true;
}




//=============================================================================
bool iScotty::OnStartUp( void )
{
	string sBar = string(40, '=') + "\n";
	string sAppName = GetAppName();

	//---------------------------------
	// Print a banner
	//---------------------------------
	MOOSTrace(sBar +
			  MOOSFormat("iScotty version %s\n", APP_VERSION_STRING) +
			  "Written by Dave Billin\n" +
			  sBar + "\n");


	//---------------------------------
	// Load mission file parameters
	//---------------------------------
	if ( !LoadMissionFileParameters() )
	{
		return false;
	}
	PrintControllerParameters();

	//-----------------------------------------------
	// Create dynamic MOOSVars for PUBLISHING
	//-----------------------------------------------
	AddMOOSVariable("PropRPM", "", "PROPELLER_RPM", 0);
	AddMOOSVariable("RPM_VELOCITY_ESTIMATE", "", "RPM_VELOCITY_ESTIMATE", 0);


	//-----------------------------------------------
	// Create dynamic MOOSVars for SUBSCRIBING
	//-----------------------------------------------
	AddMOOSVariable("iScotty_MAX_DEPTH", "iScotty_MAX_DEPTH", "", 0);

	//----------------------------------------
	// Create the object used to communicate
	// with the SCOTTY module
	//----------------------------------------
	try
	{
		m_pScotty = new ScottyModule( m_sScottyHostname, m_ScottyPort,
									  m_Verbosity );
	}
	catch (CMOOSException& e)
	{
		return MOOSFail(e.c_str());		// Handle failure to create the object
	}

	m_AppIsInitialized = true;

	if (m_Comms.IsConnected())
	{
		OnConnectToServer();	// Register for MOOS variables
	}
	return true;
}



//=============================================================================
bool iScotty::OnConnectToServer( void )
{
	// Register dynamic MOOS variables
	RegisterMOOSVariables();

	if (m_AppIsInitialized)
	{
		// Register for table-accelerated subscribed variables
		string s;
		for (int i = 0; i < NUM_SUBSCRIBED_VARIABLES; i++)
		{
			s = sm_SubscribedVariableNames[i];
			if ( !m_Comms.Register(s, 0) )
			{
				MOOSTrace("Failed to register for %s",
							sm_SubscribedVariableNames[i] );
			}
		}

		// MOOS_MANUAL_OVERRIDE is treated differently so that we can ensure it
		// gets received from the MOOSDB regardless of skew
		m_Comms.Register("MOOS_MANUAL_OVERIDE", 0);
		m_Comms.Register("MOOS_MANUAL_OVERRIDE", 0);

		// Register to receive application commands
		m_Comms.Register(GetCommandKey(), 0);

		// If the ScottyModule object has been created and a connection exists,
		// send the current servo values.
		if (m_pScotty != NULL)
		{
			if (m_pScotty->IsConnected())
			{
				uint8_t Rudder, ElevL, ElevR;
				float Coupling;
				m_pScotty->GetServoCenters(Rudder, ElevL, ElevR);
				m_pScotty->GetServoCouplingCoefficient(Coupling);
				string s = MOOSFormat("Rudder=[INT], ElevL=[INT], ElevR=[INT], "
									  "CouplingCoeff=[DOUBLE]",
									  Rudder, ElevL, ElevR, Coupling);
				m_Comms.Notify("SCOTTY_SERVO_CENTERS", s, MOOSTime() );
			}
		}

		// Publish current controller parameters
		PublishControllerParameters();
	}
	return true;
}



//=============================================================================
bool iScotty::OnDisconnectFromServer( void )
{
	// On disconnection from the MOOSDB, reset all PID's, center SCOTTY
	// servos and disengage the prop

	// Center actuators and stop
	m_pScotty->SetActuators(0.0f, 0.0f, 0.0f, 0.0f);
	m_HelmIsActive = false;

	return true;
}



//=============================================================================
bool iScotty::LoadMissionFileParameters( void )
{
	// Indices of PID parameters in vectors used by this function
	enum e_PIDParamIDs
	{
		PID_Kp = 0,		/* Proportional gain */
		PID_Ki,			/* Integral gain */
		PID_Kd,			/* Derivative gain */
		PID_iSat,		/* Integral limit (saturation value) */
		PID_ySat,		/* Output limit (saturation value) */
		NUM_PID_PARAMS
	};

	string s, sConfigFilePath;
	const char* szMissingParam = "Failed to load required mission file "
								 "parameter %s\n";

	//-----------------------------------------
	// Load required mission file parameters
	//-----------------------------------------
	// Load the hostname of the SCOTTY module (required)
	s = "SCOTTY_HOSTNAME";
	if ( !m_MissionReader.GetConfigurationParam(s, m_sScottyHostname) )
	{
		return MOOSFail( szMissingParam, s.c_str());
	}

	// Load network port used to connect to the SCOTTY module (required)
	s = "SCOTTY_PORT";
	uint32_t UintVal;
	if ( !m_MissionReader.GetConfigurationParam(s, UintVal) )
	{
		return MOOSFail(szMissingParam, s.c_str());
	}
	m_ScottyPort = static_cast<uint16_t>(UintVal);


	// Read path of vehicle-specific data from the mission file
	// Note: this is a global parameter; i.e. it lives outside any particular
	// module's configuration block
	s = "VEHICLE_CONFIG_FILE_PATH";
	if (!m_MissionReader.GetValue(s, sConfigFilePath))
	{
		return MOOSFail(szMissingParam, s.c_str());
	}


	//---------------------------------------------------
	// Load controller gains
	//---------------------------------------------------
	if ( LoadControllerParameters() == false )
	{
		return false;
	}


	//--------------------------------------------------
	// Load RPM-based velocity estimate values
	//--------------------------------------------------
	s = "VELOCITY_PER_RPM";
	if (!m_MissionReader.GetConfigurationParam(s, m_VelocityPerRPM))
	{
		return MOOSFail( szMissingParam, s.c_str() );
	}

	s = "VELOCITY_PER_RPM_OFFSET";
	if (!m_MissionReader.GetConfigurationParam(s, m_VelocityPerRPMOffset))
	{
		return MOOSFail( szMissingParam, s.c_str() );
	}



	//---------------------------------------------------
	// LOAD OPTIONAL MISSION FILE PARAMETERS
	//---------------------------------------------------
	if ( m_MissionReader.GetConfigurationParam("YAW_PID_LOGFILE", s) )
	{
		m_YawPID.SetLogPath(s);
		m_YawPID.SetLog(true);
		VERBOSE1("Logging Yaw controller to " + s);
	}
	else
	{
		m_YawPID.SetLog(false);
	}

	if ( m_MissionReader.GetConfigurationParam("SPEED_PID_LOGFILE", s) )
	{
		m_SpeedPID.SetLogPath(s);
		m_SpeedPID.SetLog(true);
		VERBOSE1("Logging Speed controller to " + s);
	}
	else
	{
		m_SpeedPID.SetLog(false);
	}

	if ( m_MissionReader.GetConfigurationParam("DEPTH_PID_LOGFILE", s) )
	{
		m_DepthPID.SetLogPath(s);
		m_DepthPID.SetLog(true);
		VERBOSE1("Logging Depth controller to " + s);
	}
	else
	{
		m_DepthPID.SetLog(false);
	}

	if ( m_MissionReader.GetConfigurationParam("ROLL_PID_LOGFILE", s) )
	{
		m_RollPID.SetLogPath(s);
		m_RollPID.SetLog(true);
		VERBOSE1("Logging Roll controller to " + s);
	}
	else
	{
		m_RollPID.SetLog(false);
	}

	if ( m_MissionReader.GetConfigurationParam("PITCH_PID_LOGFILE", s) )
	{
		m_PitchController.SetLogPath(s);
		m_PitchController.SetLog(true);
		VERBOSE1("Logging Pitch controller to " + s);
	}
	else
	{
		m_PitchController.SetLog(false);
	}

	int Verbosity;
	if ( m_MissionReader.GetConfigurationParam("VERBOSITY", Verbosity) )
	{
		m_Verbosity = Verbosity;
	}

	// DB: don't require actuator trims for now...
	LoadVehicleConfigParameters(sConfigFilePath);

	return true;
}




//=============================================================================
bool iScotty::LoadControllerParameters( void )
{
	const char* szMissingParam = "Failed to load required mission file "
								 "parameter %s\n";

	bool* const pParamChangedBase[5] = {
									 &m_SubscribedVarChanged[Idx_YawPid_Kp],
									 &m_SubscribedVarChanged[Idx_SpeedPid_Kp],
									 &m_SubscribedVarChanged[Idx_DepthPid_Kp],
									 &m_SubscribedVarChanged[Idx_RollPid_Kp],
									 &m_SubscribedVarChanged[Idx_PitchPid_Kp]};

	double* const pControllerGainBase[5] = {
								&m_SubscribedVarValueTable[Idx_YawPid_Kp],
								&m_SubscribedVarValueTable[Idx_SpeedPid_Kp],
								&m_SubscribedVarValueTable[Idx_DepthPid_Kp],
								&m_SubscribedVarValueTable[Idx_RollPid_Kp],
								&m_SubscribedVarValueTable[Idx_PitchPid_Kp] };

	// Load the Kp, Ki, Kd, IntSat, and OutSat parameters of all controllers
	string sC;
	for (int i = 0; i < 5; i++)
	{
		sC = sm_szControllerNames[i] + string("_PID_");
		double* const pControllerGain = pControllerGainBase[i];
		bool* const pParamChanged = pParamChangedBase[i];

		for (int j = 0; j < (NUM_PID_PARAMETERS-1); j++)
		{
			string s = sC + sm_szPidParamNames[j];

			// Read the controller values into the dParam array
			pParamChanged[j] = m_MissionReader.GetConfigurationParam(s,
														   pControllerGain[j]);
			if (pParamChanged[j] == false)
			{
				return MOOSFail( szMissingParam, s.c_str() );
			}
		}
	}

	// Load the KpDepth parameter for the pitch controller
	string s = "PITCH_PID_KpDepth";

	m_SubscribedVarChanged[Idx_PitchPid_KpDepth] =
			m_MissionReader.GetConfigurationParam(s,
							m_SubscribedVarValueTable[Idx_PitchPid_KpDepth]);
	if (m_SubscribedVarChanged[Idx_PitchPid_KpDepth] == false)
	{
		return MOOSFail( szMissingParam, s.c_str() );
	}

	return true;
}





//=============================================================================
void iScotty::PublishControllerParameters( void )
{
	const double* pParamValueBase[5] = {
								&m_SubscribedVarValueTable[Idx_YawPid_Kp],
								&m_SubscribedVarValueTable[Idx_SpeedPid_Kp],
								&m_SubscribedVarValueTable[Idx_DepthPid_Kp],
								&m_SubscribedVarValueTable[Idx_RollPid_Kp],
								&m_SubscribedVarValueTable[Idx_PitchPid_Kp]};
	double t = MOOSTime();

	for (int i = 0; i < 5; i++)
	{
		const double* pParamValue = pParamValueBase[i];
		string sController = sm_szControllerNames[i] + string("_PID_");

		// Publish common controller gains
		for (int j = 0; j < (NUM_PID_PARAMETERS-1); j++)
		{
			string sVarName = sController + sm_szPidParamNames[j];
			m_Comms.Notify(sVarName, pParamValue[j]);
		}
	}

	// Publish KpDepth parameter of Pitch controller as special case
	m_Comms.Notify("PITCH_PID_KpDepth",
				   m_SubscribedVarValueTable[Idx_PitchPid_KpDepth]);
}





//=============================================================================
bool iScotty::LoadVehicleConfigParameters( string& sConfigFilePath )
{
	bool Rc = true;

	if (!sConfigFilePath.empty())
	{
		CMOOSFileReader Fr;
		string s;
		uint32_t UintVal;
		const char* szParam[3] = {"RUDDER", "ELEV_L", "ELEV_R"};
		uint8_t* pServoTrim[3] = { &m_ServoCenterRudder, &m_ServoCenterElevL,
								   &m_ServoCenterElevR };
		const char* szMissingParam = "  Missing parameter: %s  Using default "
									 "value of ";

		MOOSTrace( "Loading vehicle configuration file parameters from:\n  " +
				   sConfigFilePath + "...\n" );

		// Open the vehicle-specific config file and read in the vehicle ID
		if ( Fr.SetFile(sConfigFilePath) )
		{
			for (int i = 0; i < 3; i++)
			{
				s = string("SERVO_CENTER_") + szParam[i];
				if ( Fr.GetValue(s, UintVal) )
				{
					*pServoTrim[i] = static_cast<uint8_t>(UintVal);
				}
				else
				{
					MOOSTrace( MOOSFormat(szMissingParam, s.c_str()) +
							   MOOSFormat("%d\n", *pServoTrim[i]) );
				}
			}

			s = "SERVO_COUPLINGCOEFF";
			if ( !Fr.GetValue(s, m_CouplingCoeff) )
			{
				MOOSTrace( MOOSFormat(szMissingParam, s.c_str()) +
									  MOOSFormat("%f\n", m_CouplingCoeff));
				//Rc = false;
			}

			// Notify user of trim set
			MOOSTrace("\n"
					  "Actuator trim values:\n"
					  "  Rudder         = %d\n"
					  "  Left Elevator  = %d\n"
					  "  Right Elevator = %d\n"
					  "  Coupling Coeff = %f\n\n",
					  m_ServoCenterRudder, m_ServoCenterElevL,
					  m_ServoCenterElevR, m_CouplingCoeff );

			return true;
		}
		else
		{
			MOOSTrace("The configuration file could not be opened!\n");
		}
	}
	else
	{
		MOOSTrace("No configuration file specified!\n");
	}

	return false;
}



//=============================================================================
void iScotty::UpdateControllerParameters( void )
{
	static bool* const pParamChangedBase[5] = {
								&m_SubscribedVarChanged[Idx_YawPid_Kp],
								&m_SubscribedVarChanged[Idx_SpeedPid_Kp],
								&m_SubscribedVarChanged[Idx_DepthPid_Kp],
								&m_SubscribedVarChanged[Idx_RollPid_Kp],
								&m_SubscribedVarChanged[Idx_PitchPid_Kp]};
	static double* const pParamValueBase[5] = {
								&m_SubscribedVarValueTable[Idx_YawPid_Kp],
								&m_SubscribedVarValueTable[Idx_SpeedPid_Kp],
								&m_SubscribedVarValueTable[Idx_DepthPid_Kp],
								&m_SubscribedVarValueTable[Idx_RollPid_Kp],
								&m_SubscribedVarValueTable[Idx_PitchPid_Kp]};

	for (int i = 0; i < 5; i++)
	{
		double* const pParamValue = pParamValueBase[i];
		bool* const pParamChanged = pParamChangedBase[i];

		bool GainsChanged = pParamChanged[PIDParam_Kp] ||
							pParamChanged[PIDParam_Ki] ||
							pParamChanged[PIDParam_Kd];

		// Update controller gains
		if (GainsChanged)
		{
			m_ControllerTable[i]->SetGains( pParamValue[PIDParam_Kp],
											pParamValue[PIDParam_Kd],
											pParamValue[PIDParam_Ki]);

			if (m_Verbosity >= 1)
			{
				MOOSTrace("Setting %s controller gains:\n"
						  "  Kp=%5.3f  Ki=%5.3f  Kd=%5.3f\n\n",
						  sm_szControllerNames[i],
						  pParamValue[PIDParam_Kp],
						  pParamValue[PIDParam_Ki],
						  pParamValue[PIDParam_Kd]);
			}
		}

		// Update depth gain as a special case for pitch controller
		if (i == 4)
		{
			if (pParamChanged[PIDParam_KpDepth] == true)
			{
				m_PitchController.SetDepthGain(pParamValue[PIDParam_KpDepth]);

				if (m_Verbosity >= 1)
				{
					MOOSTrace("Setting PITCH controller depth gain: "
							  "KpDepth=%5.3f\n\n",
							  pParamValue[PIDParam_KpDepth] );
				}
			}
		}

		// Update integral and output saturation values
		if ( pParamChanged[PIDParam_IntSat] ||
			 pParamChanged[PIDParam_OutSat] )
		{
			m_ControllerTable[i]->SetLimits(pParamValue[PIDParam_IntSat],
											pParamValue[PIDParam_OutSat]);
			if (m_Verbosity >= 1)
			{
				MOOSTrace("Setting %s controller limits: "
						  "IntSat=%5.3f  OutSat=%5.3f\n\n",
						  sm_szControllerNames[i],
						  pParamValue[PIDParam_IntSat],
						  pParamValue[PIDParam_OutSat]);
			}
		}

		// Clear the change flags to register parameter changes
		int NumParams = (i == 4) ? NUM_PID_PARAMETERS : (NUM_PID_PARAMETERS-1);
		for (int j = 0; j < NumParams; j++)
		{
			pParamChanged[j] = false;
		}
	}

}




//=============================================================================
void iScotty::PrintControllerParameters( void )
{
	const double* pParamValueBase[5] = {
								&m_SubscribedVarValueTable[Idx_YawPid_Kp],
								&m_SubscribedVarValueTable[Idx_SpeedPid_Kp],
								&m_SubscribedVarValueTable[Idx_DepthPid_Kp],
								&m_SubscribedVarValueTable[Idx_RollPid_Kp],
								&m_SubscribedVarValueTable[Idx_PitchPid_Kp]};

	for (int i = 0; i < 5; i++)
	{
		const double* pParamValue = pParamValueBase[i];

		// Print YAW, SPEED, DEPTH, and ROLL controller parameters
		if (i != 4)
		{
			MOOSTrace("\n%s CONTROLLER PARAMETERS:\n"
					  "Kp=%5.3f  Ki=%5.3f  Kd=%5.3f\n"
					  "IntSat=%4.2f  OutSat=%4.2f\n\n",
					  sm_szControllerNames[i],
					  pParamValue[PIDParam_Kp],
					  pParamValue[PIDParam_Ki],
					  pParamValue[PIDParam_Kd],
					  pParamValue[PIDParam_IntSat],
					  pParamValue[PIDParam_OutSat] );
		}
		else
		{
			MOOSTrace("\n%s CONTROLLER PARAMETERS:\n"
					  "Kp=%5.3f  KpDepth=%5.3f  Ki=%5.3f  Kd=%5.3f\n"
					  "IntSat=%4.2f  OutSat=%4.2f\n\n",
					  sm_szControllerNames[i],
					  pParamValue[PIDParam_Kp],
					  pParamValue[PIDParam_KpDepth],
					  pParamValue[PIDParam_Ki],
					  pParamValue[PIDParam_Kd],
					  pParamValue[PIDParam_IntSat],
					  pParamValue[PIDParam_OutSat] );
		}
	}

}





//=============================================================================
void iScotty::ResetControllers( void )
{
	m_YawPID.Reset();
	m_SpeedPID.Reset();
	m_DepthPID.Reset();
	m_RollPID.Reset();
	m_PitchController.Reset();
}





//=============================================================================
void iScotty::SetTrimsFromMOOSMail( string& sTrimSettings )
{
	int ServoValue;
	string sVal;
	uint8_t Rudder = 0;
	uint8_t ElevL = 0;
	uint8_t ElevR = 0;
	float CouplingCoeff = -1.0;

	// Get current servo values
	m_pScotty->GetServoCenters(Rudder, ElevL, ElevR);
	m_pScotty->GetServoCouplingCoefficient(CouplingCoeff);

	while ( !sTrimSettings.empty() )
	{
		int ServoValue;
		sVal = MOOSChomp(sTrimSettings, ",");
		if ( MOOSValFromString(ServoValue, sVal, "EL") )
		{
			ElevL = static_cast<uint8_t>(ServoValue);
		}
		else if ( MOOSValFromString(ServoValue, sVal, "ER") )
		{
			ElevR = static_cast<uint8_t>(ServoValue);
		}
		else if ( MOOSValFromString(ServoValue, sVal, "RR") )
		{
			Rudder = static_cast<uint8_t>(ServoValue);
		}
		else
		{
			MOOSValFromString(CouplingCoeff, sVal, "CC");
		}
	}

	// Don't update servo values if not all servos have been
	// specified!
	if ( (Rudder == 0) || (ElevL == 0) || (ElevR == 0) ||
		 (CouplingCoeff < 0.0) )
	{
		MOOSTrace("Failed to update servo trims: not all servos "
				  "were specified!\n");
		return;
	}
	else
	{
		if (m_Verbosity > 1)
		{
			MOOSTrace("Sent servo center values:\n"
					  "  L Elevator: %d\n"
					  "  R Elevator: %d\n"
					  "  Rudder:     %d\n"
					  "  Coupling:   %5.4f\n",
					  ElevL, ElevR, Rudder, CouplingCoeff);
		}
		m_pScotty->SetServoCenters( Rudder, ElevL, ElevR,
									CouplingCoeff);
	}
}





//=============================================================================
void iScotty::RunPIDControllers( void )
{
	double Error, y;
	double t = MOOSTime() - GetMoosTimeAtMidnight();
	double DepthControlLaw = 0.0;

	//----------------------
	// Yaw PID controller
	//----------------------
	//m_SubscribedVarValueTable[Idx_DesiredRudder] = 0.0;	// Default to null
	if (m_YawControlIsEnabled)
	{
		double DesiredYaw;
		DesiredYaw= MOOSDeg2Rad(m_SubscribedVarValueTable[Idx_DesiredHeading]);
		DesiredYaw = MOOS_ANGLE_WRAP(DesiredYaw);

		Error = DesiredYaw - m_SubscribedVarValueTable[Idx_NavYaw];
		if ( m_YawPID.Run(Error, t, y) )
		{
			m_SubscribedVarValueTable[Idx_DesiredRudder] = y;
			m_SubscribedVarChanged[Idx_DesiredRudder] = true;
			m_Comms.Notify("DESIRED_RUDDER", y);
		}
	}


	//----------------------
	// Speed PID controller
	//----------------------
	//m_SubscribedVarValueTable[Idx_DesiredThrust] = 0.0;	// Default to null
	if (m_SpeedControlIsEnabled)
	{
		Error = m_SubscribedVarValueTable[Idx_DesiredSpeed] -
				m_SubscribedVarValueTable[Idx_NavSpeed];

		// We can't really go any closer than 0.1 m/sec, so bound the error
		Error = (Error < 0.1) ? 0.0 : Error;

		if ( m_SpeedPID.Run(Error, t, y) )
		{
			m_SubscribedVarValueTable[Idx_DesiredThrust] = y;
			m_SubscribedVarChanged[Idx_DesiredThrust] = true;
			m_Comms.Notify("DESIRED_THRUST", y);
		}
	}

	//----------------------
	// Roll PID controller
	//----------------------
	//m_SubscribedVarValueTable[Idx_DesiredAileron] = 0.0;	// Default to null
	if (m_RollControlIsEnabled)
	{
		Error = m_SubscribedVarValueTable[Idx_DesiredRoll] -
				m_SubscribedVarValueTable[Idx_NavRoll];
		if ( m_RollPID.Run(Error, t, y) )
		{
			m_SubscribedVarValueTable[Idx_DesiredAileron] = y;
			m_SubscribedVarChanged[Idx_DesiredAileron] = true;
			m_Comms.Notify("DESIRED_AILERON", y);
		}
	}


	//----------------------
	// Depth PID controller
	//----------------------
	DepthControlLaw = 0.0;	// Default to null
	if (m_DepthControlIsEnabled)
	{
		// Saturate the desired depth by the maximum depth allowed by iScotty
		if (m_SubscribedVarValueTable[Idx_DesiredDepth] >
			  m_SubscribedVarValueTable[Idx_MaxDepth] )
		{
			m_SubscribedVarValueTable[Idx_DesiredDepth] =
					m_SubscribedVarValueTable[Idx_MaxDepth];
		}
		// Calculate depth control law
		Error = m_SubscribedVarValueTable[Idx_DesiredDepth] -
				m_SubscribedVarValueTable[Idx_NavDepth];

		if ( m_DepthPID.Run(Error, t, y) )
		{
			DepthControlLaw = y;
		}
	}


	//----------------------
	// Pitch controller
	// NOTE: If we disable pitch control, but the depth controller is enabled,
	// we still run the pitch controller - only with DESIRED_PITCH set to zero.
	// This allows us to disable the DESIRED_PITCH input from the controller,
	// while still getting an input from the depth control law.  If both pitch
	// and depth controllers are disabled, we don't run the pitch controller
	//----------------------
	if (m_PitchControlIsEnabled || m_DepthControlIsEnabled)
	{
		if (m_PitchControlIsEnabled)
		{
			Error = m_SubscribedVarValueTable[Idx_DesiredPitch] -
					m_SubscribedVarValueTable[Idx_NavPitch];
		}
		else
		{
			// If pitch control is disabled, zero the error term to nullify the
			// input from DESIRED_PITCH}
			Error = 0.0;
		}

		//MOOSTrace("Pitch error = %f\n", Error);

		if ( m_PitchController.Run(Error, DepthControlLaw, t, y) )
		{
			y = -1.0 * y;
			m_SubscribedVarValueTable[Idx_DesiredElevator] = y;
			m_SubscribedVarChanged[Idx_DesiredElevator] = true;
			m_Comms.Notify("DESIRED_ELEVATOR", y);
		}
	}

}

