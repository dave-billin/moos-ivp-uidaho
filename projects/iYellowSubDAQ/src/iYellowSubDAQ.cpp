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
/** @file iYellowSubDAQ.cpp
 *
 * @brief
 *  Implementation of the iYellowSubDAQ MOOS application object
 *  
 * @author Dave Billin
 */
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "AppVersion.h"
#include "iYellowSubDAQ.h"

using namespace std;


// NOTE: elements in this array *must* directly correspond to the elements of
// e_SubscribedVariables
const char* iYellowSubDAQ::
			sm_DefaultSubscribedVarNames[NUM_SUBSCRIBED_VARIABLES] = {
										"NAV_X",
										"NAV_Y",
										"NAV_HEADING",
										"NAV_DEPTH",
										"NAV_PITCH",
										"NAV_ROLL",
										"VEHICLE_ID",
										"MISSION_ID",
										"RUN_ID"
									};



// NOTE: this array of string constants *must* correspond in order and number
// of elements to e_PublishedVariables
const char* iYellowSubDAQ::
			sm_DefaultPublishedVarNames[NUM_PUBLISHED_VARIABLES] = {
										"DAQ_STATUS"
										};




//=============================================================================
iYellowSubDAQ::iYellowSubDAQ( void )
 : m_Verbosity(0),
   m_BunnySockVerbosity(0),
   m_AppIsOnline(false),
   m_pDAQ(NULL),
   m_DAQPort(0),
   m_ConnectionTimeoutMs(BUNNYSOCK_DEFAULT_CONNECTION_TIMEOUT_MS)
{
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

	// Process command messages in OnCommandMsg()
	EnableCommandMessageFiltering(true);
}


//=============================================================================
iYellowSubDAQ::~iYellowSubDAQ()
{
	if (m_pDAQ != NULL)
	{
		delete m_pDAQ;
	}
}



//=============================================================================
bool iYellowSubDAQ::OnNewMail(MOOSMSG_LIST & NewMail)
{
	// Update subscribed variables
	UpdateMOOSVariables(NewMail);
	return true;
}



//=============================================================================
bool iYellowSubDAQ::OnCommandMsg(CMOOSMsg Msg)
{
	string sVal = Msg.GetString();

	// Reject skewed commands
	if ( Msg.IsSkewed(MOOSTime(true)) )
	{
		MOOSTrace("Rejecting skewed DAQ command: " + sVal + "\n");
		return true;
	}

	// Parse record commands here
	if ( MOOSStrCmp(sVal, "RECORD") )
	{
		m_pDAQ->StartRecording(
				(int16_t)(m_SubscribedVarTable[VEHICLE_ID]->GetDoubleVal()),
				(int16_t)(m_SubscribedVarTable[MISSION_ID]->GetDoubleVal()),
				(int16_t)(m_SubscribedVarTable[RUN_ID]->GetDoubleVal()),
				0
				);
	}
	else if ( MOOSStrCmp(sVal, "STOP") )
	{
		m_pDAQ->StopRecording();
	}

	return true;
}



//=============================================================================
bool iYellowSubDAQ::Iterate( void )
{
	static bool LastDAQConnected = false;
	static int LastDAQStatus = YellowSubDAQModule::NOT_CONNECTED;
	static int IterateCounter = 0;		// Used to time DAQ status queries
	static const char* szDreStates[] =
								{ "Not Connected", "Stalled", "No Disk",
								  "Idle", "Preparing", "Recording",
								  "Stopping" };

	bool DAQIsConnected = m_pDAQ->IsConnected();
	bool DAQConnectionChanged = (LastDAQConnected != DAQIsConnected);

	// Handle changes in DAQ connection status
	if ( DAQConnectionChanged )
	{
		// Update DAQ connection status
		if ( DAQIsConnected )
		{
			SetAppError(true, "DAQ connected");
			m_pDAQ->QueryEngineStatus();	// Query the engine status
			IterateCounter = 0;		// Reset query timer
		}
		else
		{
			SetAppError(false, "DAQ not connected");
			LastDAQStatus = YellowSubDAQModule::NOT_CONNECTED;
			m_PublishedVarTable[DAQ_STATUS]->Set(szDreStates[0], MOOSTime());
		}

		LastDAQConnected = DAQIsConnected;
	}


	// Publish DAQ engine status when it changes or when the DAQ
	// connection status changes
	if ( DAQIsConnected )
	{
		int DAQStatus = m_pDAQ->GetEngineStatus();

		if ( DAQConnectionChanged || (DAQStatus != LastDAQStatus) )
		{
			//---------------------------------------
			// Publish the DAQ engine status
			//---------------------------------------
			string s;

			if ( (DAQStatus >= YellowSubDAQModule::NOT_CONNECTED) &&
				 (DAQStatus <= YellowSubDAQModule::STOPPING) )
			{
				s = szDreStates[DAQStatus + 3];
			}
			else
			{
				s = "Unknown";
			}

			m_PublishedVarTable[DAQ_STATUS]->Set(s, MOOSTime());

			LastDAQStatus = DAQStatus;
		}
	}

	// Query the status of the DAQ module once every second
	if (IterateCounter++ == m_dfFreq)
	{
		//m_pDAQ->QueryEngineStatus();
		IterateCounter = 0;
	}

	PublishFreshMOOSVariables();
	return true;
}



//=============================================================================
bool iYellowSubDAQ::OnStartUp( void )
{
	string s;
	string sBar40 = string(40, '=') + "\n";
	string sAppName = GetAppName();

	SetAppError(true, "DAQ not connected");

	// Print a banner
	s = string(40, '=') + "\n";
	MOOSTrace(sBar40 +
			  MOOSFormat("iYellowSubDAQ v%s\n", APP_VERSION_STRING) +
			  "Written by Dave Billin\n\n"
			  "  Live long... and prosper.\n" +
			  sBar40 + "\n\n");

	// Load mission file parameters.
	if ( !LoadMissionFileParameters() )
	{
		return false;
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
			m_SubscribedVarTable[i]->Set(0.0, 0.0);
		}
	}


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


	//----------------------------------------
	// Create the object used to communicate
	// with the SPOCK module
	//----------------------------------------
	try
	{
		m_pDAQ = new YellowSubDAQModule( m_DAQHostname, m_DAQPort,
										 m_ConnectionTimeoutMs, m_Verbosity );
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
bool iYellowSubDAQ::OnConnectToServer( void )
{
	if (m_AppIsOnline)
	{
		// Register to receive application commands
		m_Comms.Register( GetCommandKey(), 0);
		RegisterMOOSVariables();
	}

	return true;
}



//=============================================================================
bool iYellowSubDAQ::OnDisconnectFromServer( void )
{
	return true;
}



//=============================================================================
bool iYellowSubDAQ::LoadMissionFileParameters( void )
{
	string s;
	int IntVal;
	const char* szParamFail = "Failed to read required mission file "
							  "parameter %s\n";

	//----------------------------------------
	// Load required mission file parameters
	//----------------------------------------

	// Get the hostname of the DAQ module
	s = "DAQ_HOSTNAME";
	if ( !m_MissionReader.GetConfigurationParam(s,
												m_DAQHostname) )
	{
		return MOOSFail(szParamFail, s.c_str());
	}

	// Get the network port to connect to on the DAQ module
	s = "DAQ_PORT";
	if ( !m_MissionReader.GetConfigurationParam(s, IntVal) )
	{
		return MOOSFail(szParamFail, s.c_str());
	}
	m_DAQPort = static_cast<uint16_t>(IntVal);

    // Get the DAQ module connection timeout in milliseconds
	s = "CONNECTION_TIMEOUT";
    if ( !m_MissionReader.GetConfigurationParam(s, m_ConnectionTimeoutMs) )
    {
        return MOOSFail(szParamFail, s.c_str());
    }

	//----------------------------------------
	// Load optional mission file parameters
	//----------------------------------------

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

	// Get verbosity level for debugging messages
	m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);
	m_MissionReader.GetConfigurationParam("BUNNYSOCK_VERBOSITY",
										  m_BunnySockVerbosity );
	return true;
}

