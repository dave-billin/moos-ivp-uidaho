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
/** @file pMissionMonitor.cpp

@brief
	Implementation of the pMissionMonitor object

@author Dave Billin

*/
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "MonitorConfigReader.h"
#include "pMissionMonitor.h"


using namespace std;


//=============================================================================
/* Creates an instance of the object */
pMissionMonitor::pMissionMonitor( void )
: m_AppIsInitialized(false)
{
	// <<< Construct the MOOS Application here >>>
}




//=============================================================================
/* Called when the object goes out of scope */
pMissionMonitor::~pMissionMonitor()
{
	// <<< Construct the MOOS Application here >>>
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
bool pMissionMonitor::OnNewMail(MOOSMSG_LIST& NewMail)
{
	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		string sKey, sVal;
		double dVal;
		set<string>::iterator VarName;

		// Walk through the list of received messages
		MOOSMSG_LIST::iterator iter;
		for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
		{
			sKey = iter->GetKey();
			sVal = iter->GetString();
			dVal = iter->GetDouble();

			// If the received mail message is for a CONDITION variable,
			// update it in the InfoBuffer
			VarName = m_ConditionVarNames.find(sKey);
			if ( VarName != m_ConditionVarNames.end() )
			{
				if (iter->IsString())
				{
					m_InfoBuffer.setValue(sKey, sVal);
				}
				else
				{
					m_InfoBuffer.setValue(sKey, dVal);
				}
			}
		}

		//-----------------------------------------------
		// UPDATE AND EVALUATE ALL MONITOR TARGETS
		//-----------------------------------------------
		for ( vector<MonitorTarget>::iterator Target = m_MonitorTargets.begin();
				Target != m_MonitorTargets.end(); Target++ )
		{
			Target->UpdateConditionVariables(m_InfoBuffer);

			// If the monitor's CONDITIONS evaluate to false,
			// publish the monitor's items and print its string
			if ( Target->EvaluateConditions() == false )
			{
				Target->PublishItems(m_Comms);
				MOOSTrace(Target->GetPrintString() + "\n");
			}
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
/* If command message filtering is enabled for this application (via the
	EnableCommandMessageFiltering() function), then this function is called
	when a command message having the identifier
	pMissionMonitor_CMD is recieved from the MOOS Database.

@param Msg
	A copy of the received command message

@return
	true on success; false on error

@see CMOOSApp::EnableCommandMessageFiltering
*/
bool pMissionMonitor::OnCommandMsg(CMOOSMsg Msg)
{
	/*--------------------------------------------------------------
		<<< After enabling command message filtering, you can >>>
		<<< parse received application commands here		  >>>
	--------------------------------------------------------------*/
	return true;
}





//=============================================================================
/* This function is where the application can do most of its work.
@details
	The rate at which Iterate() is called is determined by the value of the
	AppTick parameter specified in the (.moos) mission file.  The value of
	AppTick is loaded automatically when the application starts up.
*/
bool pMissionMonitor::Iterate( void )
{
	//-----------------------------------------------
	// EVALUATE AND ENFORCE ALL MONITOR TARGETS
	// NOTE: the rate at which this will be done is
	// set by the value of the APP_TICK mission
	// file parameter.
	//-----------------------------------------------
	for ( vector<MonitorTarget>::iterator Target = m_MonitorTargets.begin();
		  Target != m_MonitorTargets.end(); Target++ )
	{
		// If the monitor's CONDITIONS evaluate to false,
		// publish the monitor's items and print its string
		if ( Target->EvaluateConditions() == false )
		{
			Target->PublishItems(m_Comms);
			MOOSTrace(Target->GetPrintString() + "\n");
		}
	}

	return true;
}




//=============================================================================
/* Called when the application first starts up
@details
	This function is called as the application first starts up before any
	calls to Iterate() begin.
*/
bool pMissionMonitor::OnStartUp()
{
	bool Rc = false;
	string sParamError("Failed to read required mission file parameter: ");
	string sParam, sVal;
	const char szVariable[] = "variable";
	const char szVariables[] = "variables";

	sParam = "MONITOR_CONFIG_FILE";
	if ( !m_MissionReader.GetConfigurationParam(sParam, sVal) )
	{
		SetAppError(true, "Failed to load monitor config file");
		MOOSTrace(sParamError + sParam);
		// Rc is already false
	}
	else
	{
		// Load monitor configuration from the specified file
		// This will:
		//	- Populate m_MonitorTargets
		//	- Register all MOOS variables used in monitored CONDITIONS
		MOOSTrace( "\nReading from monitor configuration file:\n" +
				   sVal + "\n");

		if ( LoadMonitorTargetsFromFile(sVal) )
		{
			MOOSTrace( "*** Loaded %d monitor configuration blocks ***\n",
					   m_MonitorTargets.size() );

			//------------------------------------------------------
			// Display variables each configuration block monitors
			string sBar80 = string(79, '=') + "\n";
			vector<MonitorTarget>::iterator iter;
			for ( iter = m_MonitorTargets.begin();
				  iter != m_MonitorTargets.end(); iter++ )
			{
				set<string> Vars = iter->GetMoosVariableNames();
				int NumVars = Vars.size();
				MOOSTrace( sBar80 + "Config Block: " + iter->GetName() );
				MOOSTrace( " Monitors %d MOOS %s:\n", NumVars,
						   ((NumVars < 2) ? szVariable : szVariables) );


				string sVarList;
				for ( set<string>::iterator s = Vars.begin(); s != Vars.end();
					  s++ )
				{
					sVarList = sVarList + "   " + *s + "\n";
				}

				MOOSTrace(sVarList + "\n" + sBar80);
			}

			Rc = true;
		}
	}

	m_AppIsInitialized = true;
	return Rc;
}




//=============================================================================
/* Called when the application connects to the MOOS Database. */
bool pMissionMonitor::OnConnectToServer( void )
{
	if (m_AppIsInitialized)
	{
		//--------------------------------------------------
		// Subscribe to MOOS variables needed by the new
		// time slices
		//--------------------------------------------------
		for ( set<string>::iterator s = m_ConditionVarNames.begin();
			  s != m_ConditionVarNames.end(); s++ )
		{
			m_Comms.Register(*s, 0);
		}
	}

	return true;
}




//=============================================================================
/* Called when the application is disconnected from the MOOS Database. */
bool pMissionMonitor::OnDisconnectFromServer( void )
{
	return true;
}



//=============================================================================
bool pMissionMonitor::LoadMonitorTargetsFromFile( string& sFilePath )
{
    bool retval = false;

	if ( sFilePath.empty() == false )
	{
		MonitorConfigReader ConfigReader;

		//--------------------------------------------------
		// Parse the configuration file
		//--------------------------------------------------
		try
		{
			m_MonitorTargets = ConfigReader.ParseFile( sFilePath );
		}
		catch ( CMOOSException& e )
		{
			MOOSTrace("Error reading " + sFilePath + "\n" +
					  e.m_sReason + "\n");
			return false;
		}

		//--------------------------------------------------
		// Unsubscribe from variables named in the existing
		// variable name set
		//--------------------------------------------------
		for ( set<string>::iterator VarName = m_ConditionVarNames.begin();
			  VarName != m_ConditionVarNames.end(); VarName++ )
		{
			m_Comms.UnRegister(*VarName);
		}
		m_ConditionVarNames.clear();

		// Initialize the info buffer
		m_InfoBuffer.clearDeltaVectors();


		//--------------------------------------------------
		// Copy the names of MOOS variables used in time
		// slice CONDITIONS into the variable name set
		// Sum time slice durations
		// Build the m_SliceTimes set as we go
		//--------------------------------------------------
		for ( vector<MonitorTarget>::iterator iter = m_MonitorTargets.begin();
			  iter != m_MonitorTargets.end(); iter++ )
		{
			// Add variable names from each MonitorTarget to the set of
			// MOOS variable names to be parsed for in OnNewMail()
			const set<string>& TargetVars = iter->GetMoosVariableNames();
			for ( set<string>::iterator s = TargetVars.begin();
				  s != TargetVars.end(); s++ )
			{
				m_ConditionVarNames.insert( *s );
			}

		}

		//--------------------------------------------------
		// Subscribe to MOOS variables needed by the new
		// time slices
		//--------------------------------------------------
		for ( set<string>::iterator s = m_ConditionVarNames.begin();
			  s != m_ConditionVarNames.end(); s++ )
		{
			m_Comms.Register(*s, 0);
		}

		retval = true;
	}

	return retval;
}

