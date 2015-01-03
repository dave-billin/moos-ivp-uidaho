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
/** @file pAcousticCommsAgent.cpp

@brief
	Implementation of the pAcousticCommsAgent object

@author Dave Billin

*/
//=============================================================================

#include <time.h>
#include <string.h>
#include <math.h>
#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "TimeSliceConfigReader.h"
#include "pAcousticCommsAgent.h"


using namespace std;


//=============================================================================
/* Creates an instance of the object */
pAcousticCommsAgent::pAcousticCommsAgent( void )
: m_AppIsInitialized(false),
  m_InfoBuffer(),
  m_CommsCycleDuration_sec(0.0),
  m_MoosTimeAtMidnight(0.0)
{
}




//=============================================================================
/* Called when the object goes out of scope */
pAcousticCommsAgent::~pAcousticCommsAgent()
{
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
bool pAcousticCommsAgent::OnNewMail(MOOSMSG_LIST & NewMail)
{
	string sKey, sVal;
	double dVal;
	set<string>::iterator VarName;

	// If we're actually running...
	if ( !IsSimulateMode() )
	{
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
			/*CMOOSMsg& RxMsg = *iter;
			if( RxMsg.IsSkewed( MOOSTime() ) )
			{
				// << Handle skewed messages here >>>
				continue;
			}
			*/

			// TODO: Add a command to load a new cycle configuration file
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
	PACOUSTICCOMMSAGENT_CMD is recieved from the MOOS Database.

@param Msg
	A copy of the received command message

@return
	true on success; false on error

@see CMOOSApp::EnableCommandMessageFiltering
*/
bool pAcousticCommsAgent::OnCommandMsg(CMOOSMsg Msg)
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
bool pAcousticCommsAgent::Iterate( void )
{
	static TimeSlice* pLastActiveSlice = NULL;
	double SecSinceMidnight = HPMOOSTime() - m_MoosTimeAtMidnight;

	// Look up which Time Slice is currently active
	TimeSlice* pActiveSlice = GetActiveTimeSlice(SecSinceMidnight);
	if (pActiveSlice != NULL)
	{
		// Determine whether a new TimeSlice has become active
		if (pLastActiveSlice != pActiveSlice)
		{
			// Post the active time slice's name
			m_Comms.Notify("ACTIVE_COMMS_TIMESLICE", pActiveSlice->GetName());

			// Do TimeSlice work (check conditions, publish variables, etc...)
			pActiveSlice->Activate();

			// Register the new active slice
			pLastActiveSlice = pActiveSlice;
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
bool pAcousticCommsAgent::OnStartUp()
{
	bool Rc = false;
	string sParamError("Failed to read required mission file parameter: ");
	string sParam, sVal;

	sParam = "TIMING_CYCLE_FILE";
	if ( !m_MissionReader.GetConfigurationParam(sParam, sVal) )
	{
		MOOSTrace(sParamError + sParam);
	}
	else
	{
		// Load timing cycle configuration from the specified cycle file
		// This will:
		//	- Populate m_TimeSliceList
		//	- Register all MOOS variables used in time slice CONDITIONS
		//	- Populate m_CommsCycleDuration_sec (total cycle duration)
		MOOSTrace("\nReading time slice configuration file:" + sVal + "...\n");
		if ( LoadCommsCycleFromFile(sVal) )
		{
			MOOSTrace("*** Loaded %d time slice configurations ***\n",
					  m_TimeSliceList.size() );

			string sBar80 = string(79, '-') + "\n";
			vector<double>::iterator SliceTime = m_SliceTimes.begin();
			for (list<TimeSlice>::iterator iter = m_TimeSliceList.begin();
				 iter != m_TimeSliceList.end(); iter++)
			{
				MOOSTrace(sBar80);
				MOOSTrace("(%03d) \"%s\"  (%-0.3f...%-0.3f sec)\n",
						  iter->GetOrder(), iter->GetName().c_str(),
						  *SliceTime,
						  (*SliceTime + iter->GetDuration()) );

				SliceTime++;
			}
			MOOSTrace(sBar80);

			MOOSTrace("\nCommunications Cycle Duration: %-6.3f seconds\n\n",
					  m_CommsCycleDuration_sec );

			// Get MOOSTime() value at midnight
			m_MoosTimeAtMidnight = GetMoosTimeAtMidnight();

			Rc = true;
		}
	}

	return Rc;
}




//=============================================================================
/* Called when the application connects to the MOOS Database. */
bool pAcousticCommsAgent::OnConnectToServer( void )
{
	if (m_AppIsInitialized)
	{
		//--------------------------------------------------
		// Subscribe to MOOS variables used in time slices
		//--------------------------------------------------
		for ( set<string>::iterator Name = m_ConditionVarNames.begin();
			  Name != m_ConditionVarNames.end(); Name++ )
		{
			m_Comms.Register(*Name, 0);
		}
	}


	return true;
}




//=============================================================================
/* Called when the application is disconnected from the MOOS Database. */
bool pAcousticCommsAgent::OnDisconnectFromServer( void )
{
	/*-----------------------------------------------------
		The application is now disconnected from the
		MOOS Database.
	-----------------------------------------------------*/
	return true;
}


//=============================================================================
bool pAcousticCommsAgent::LoadCommsCycleFromFile( string& sFilePath )
{
    bool retval = false;

	if ( sFilePath.empty() == false )
	{
		TimeSliceConfigReader TsConfigReader;

		//--------------------------------------------------
		// Parse the configuration file
		//--------------------------------------------------
		try
		{
			m_TimeSliceList = TsConfigReader.ParseFile( sFilePath,
														m_InfoBuffer,
														m_Comms);
		}
		catch ( CMOOSException& e )
		{
			MOOSTrace("Error reading " + sFilePath + ": " + e.m_sReason);
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
		m_CommsCycleDuration_sec = 0.0;
		m_SliceTimes.clear();
		for ( list<TimeSlice>::iterator Slice = m_TimeSliceList.begin();
			  Slice != m_TimeSliceList.end(); Slice++ )
		{
			// Copy variable names from the slice into the variable name set
			// so that they can be parsed for in NewMail
			vector<string> SliceVarNames = Slice->GetMoosVariableNames();
			for ( vector<string>::iterator Name = SliceVarNames.begin();
				  Name != SliceVarNames.end(); Name++ )
			{
				m_ConditionVarNames.insert(*Name);
			}

			// Register time accumulated before the current slice begins
			m_SliceTimes.push_back( m_CommsCycleDuration_sec );

			// Accumulate slice durations
			m_CommsCycleDuration_sec += Slice->GetDuration();
		}


		//--------------------------------------------------
		// Subscribe to MOOS variables needed by the new
		// time slices
		//--------------------------------------------------
		for ( set<string>::iterator Name = m_ConditionVarNames.begin();
			  Name != m_ConditionVarNames.end(); Name++ )
		{
			m_Comms.Register(*Name, 0);
		}

		retval = true;
	}

	return retval;
}




//=============================================================================
double pAcousticCommsAgent::GetMoosTimeAtMidnight( void )
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
TimeSlice* pAcousticCommsAgent::GetActiveTimeSlice(double SecondsInCommsCycle)
{
	// Modulus by total comms cycle duration to get time in comms cycle
	SecondsInCommsCycle = fmod(SecondsInCommsCycle, m_CommsCycleDuration_sec);

	list<TimeSlice>::reverse_iterator Slice = m_TimeSliceList.rbegin();
	for ( vector<double>::reverse_iterator Time = m_SliceTimes.rbegin();
		  Time != m_SliceTimes.rend(); ++Time )
	{
		if ( SecondsInCommsCycle >= *Time )
		{
			return &(*Slice);
		}
		else
		{
			Slice++;
		}
	}

	return NULL;	// Return NULL if we failed to find an active slice
}


