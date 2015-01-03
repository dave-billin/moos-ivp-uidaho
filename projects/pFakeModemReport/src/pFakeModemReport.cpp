//=============================================================================
/*    Copyright (C) 2012  Brandt Pedrow

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
/** @file pFakeModemReport.cpp

@brief
	Implementation of the pFakeModemReport object

@author Brandt Pedrow

*/
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "pFakeModemReport.h"
#include <cmath>

using namespace std;
using namespace YellowSubNav;

bool RunIterate = false;
double SSH20;
const char* pFakeModemReport::sm_DefaultSubscribedNames[NUM_SUBSCRIBETO_IDS] = {
										"GPS_LATITUDE",
										"GPS_LONGITUDE",
										"DEPTH"
										};



//=============================================================================
/* Creates an instance of the object */
pFakeModemReport::pFakeModemReport( void )
: m_CriticalFaultExists(false),
  m_AppIsInitialized(false)
{
string s;

	// Populate default variable names subscribed to
	m_SubscribedVariables.reserve(NUM_SUBSCRIBETO_IDS);
	for (int i = 0; i < NUM_SUBSCRIBETO_IDS; i++)
	{
		s = sm_DefaultSubscribedNames[i];
		//MOOSTrace("m_SubscribedVariables[%d] = %s\n", i, s.c_str());	//&
		m_SubscribedVariables.push_back(s);
	}
	for (int i=0; i < 4; i++)
	{
		m_pBeacons[i] = new LblBeacon(m_ReferenceGeodesy);
	}
	m_PublishedVariables = "LBL_2WAYTRAVELTIME";
	// <<< Construct the MOOS Application here >>>
}




//=============================================================================
/* Called when the object goes out of scope */
pFakeModemReport::~pFakeModemReport()
{
	for (int i=0; i < 4; i++)
	{
		if ( m_pBeacons[i] != NULL )
		{
			delete m_pBeacons[i];
		}
	}
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
bool pFakeModemReport::OnNewMail(MOOSMSG_LIST & NewMail)
{

	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		UpdateMOOSVariables(NewMail);
		//double lat = GetMOOSVar(m_SubscribedVariables[GPS_LATITUDE])->GetDoubleVal();
		//double lon = GetMOOSVar(m_SubscribedVariables[GPS_LONGITUDE])->GetDoubleVal();
		//MOOSTrace("Lat is %f and Lon is %f \n", lat, lon);

/*
		// Walk through the list of received messages
		MOOSMSG_LIST::iterator iter;
		for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
		{
			// Identify skewed messages
			CMOOSMsg& RxMsg = *iter;
			if( RxMsg.IsSkewed( MOOSTime() ) )
			{
				// << Handle skewed messages here >>>
				continue;
			}

			// Ignore received pFakeModemReport_CMD messages if command message filtering is
			// enabled to prevent us from processing the same message twice.
			if ( m_bCommandMessageFiltering == true )
			{
				if ( MOOSStrCmp(RxMsg.GetKey(), GetCommandKey()) )
				{
					continue;
				}
			}



			//--------------------------------------------------------------
			//	<<< Parse incoming mail from the MOOS Database here >>>
			//--------------------------------------------------------------
		}*/
		//CMOOSVariable* LAT = GetMOOSVar(m_SubscribedVariables[GPS_LATITUDE]);
		//CMOOSVariable* LONG = GetMOOSVar(m_SubscribedVariables[GPS_LONGITUDE]);
		CMOOSVariable* iWhoiMicroModem_CMD = GetMOOSVar("iWhoiMicroModem_CMD");
		//CMOOSVariable* Depth = GetMOOSVar(m_SubscribedVariables[DEPTH_Sensor]);
		string CMD = iWhoiMicroModem_CMD->GetStringVal();
		string ChompedCMD = MOOSChomp(CMD);
		if ( MOOSStrCmp(ChompedCMD, "CMD=TxRemusPing") )
		{
				RunIterate = true;
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
	pFakeModemReport_CMD is recieved from the MOOS Database.

@param Msg
	A copy of the received command message

@return
	true on success; false on error

@see CMOOSApp::EnableCommandMessageFiltering
*/
bool pFakeModemReport::OnCommandMsg(CMOOSMsg Msg)
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
bool pFakeModemReport::Iterate( void )
{
	/*----------------------------------------------
		<<< Carry out the application's tasks >>>
	----------------------------------------------*/
	if (RunIterate == true)
	{
		RunIterate = false;
		TimeCalculation();
	}
	

	return true;
}




//=============================================================================
/* Called when the application first starts up
@details
	This function is called as the application first starts up before any
	calls to Iterate() begin.
*/
bool pFakeModemReport::OnStartUp()
{
	/*-----------------------------------------------------
		The application is starting up.  This is a
		good place to load settings from the mission
		file, or enable command message	filtering.
	-----------------------------------------------------*/
	LoadMissionParameters();

	// Now add the moos variables
	AddMOOSVariable(m_SubscribedVariables[GPS_LATITUDE], m_SubscribedVariables[GPS_LATITUDE],
			m_SubscribedVariables[GPS_LATITUDE], 0);
	AddMOOSVariable(m_SubscribedVariables[GPS_LONGITUDE], m_SubscribedVariables[GPS_LONGITUDE],
			m_SubscribedVariables[GPS_LONGITUDE],0);
	AddMOOSVariable("iWhoiMicroModem_CMD","iWhoiMicroModem_CMD","iWhoiMicroModem_CMD",0);
	AddMOOSVariable(m_SubscribedVariables[DEPTH_Sensor], m_SubscribedVariables[DEPTH_Sensor],
			m_SubscribedVariables[DEPTH_Sensor],0);
	m_AppIsInitialized = true;
	SetMOOSVar(m_SubscribedVariables[GPS_LATITUDE],46.730506,0);
	SetMOOSVar(m_SubscribedVariables[GPS_LONGITUDE],117.007792,0);
	SetMOOSVar(m_SubscribedVariables[DEPTH_Sensor],1,0);
	//Origin_Latitude = 46.730506
	//Origin_Longitude = 117.007792
	OnConnectToServer();
	return true;
}




//=============================================================================
/* Called when the application connects to the MOOS Database. */
bool pFakeModemReport::OnConnectToServer( void )
{
	// Add MOOS variables we want to monitor for fault conditions
	if (m_AppIsInitialized)
	{
		Registration();
	}
	return true;
}




//=============================================================================
/* Called when the application is disconnected from the MOOS Database. */
bool pFakeModemReport::OnDisconnectFromServer( void )
{
	/*-----------------------------------------------------
		The application is now disconnected from the
		MOOS Database.
	-----------------------------------------------------*/
	return true;
}
//==============================================================================
//Registers for various variables, called from OnConnectToServer and OnStartup
bool pFakeModemReport::Registration()
{
	RegisterMOOSVariables();
	/*
	m_Comms.Register("GPS_Latitude",0);
	m_Comms.Register("GPS_Longitude",0);
	m_Comms.Register("iWhoiMicroModem_CMD",0);
	m_Comms.Register("DEPTH_Sensor",0);
	*/
	return true;
}


//===============================================================================
//Shamelessly copy pastaded from Dave Billin's pVehicleEKF function
//The origin, and lbl beacon locations (either lat/long or local) HAVE to be in the global
//variable section of the .moos file.
bool pFakeModemReport::LoadMissionParameters( void )
{
	string s;
	double Lat, Lon, Depth;
	//const char* szBeaconParam[] = { "Latitude", "Longitude", "Depth" };
	//const char* szAltBeaconParam[] = { "North", "East", "Depth" };
	const char* szRequiredParamError = "ERROR: Failed to load required mission "
									   "file parameter %s!";
	m_MissionReader.GetValue("SSH20",SSH20);
	//---------------------------------------------------------------
	// Load REQUIRED coordinate system origin
	//---------------------------------------------------------------
	s = "LatOrigin";
	if (!m_MissionReader.GetValue(s, Lat))
	{
		return MOOSFail(szRequiredParamError, s.c_str());
	}
	m_MissionReader.GetValue(s,Lat);
	s = "LongOrigin";
	if (!m_MissionReader.GetValue(s, Lon))
	{
		return MOOSFail(szRequiredParamError, s.c_str());
	}
	m_MissionReader.GetValue(s,Lon);
	// Initialize coordinate system origin
	m_ReferenceGeodesy.Initialise(Lat, Lon);



	//---------------------------------------------------------------
	// Load REQUIRED LBL transponder beacon locations
	//---------------------------------------------------------------
	for (char BeaconId = 'A'; BeaconId <= 'D'; BeaconId++)
	{
		s = string("Beacon") + BeaconId;
		LblBeacon& NavBeacon = *m_pBeacons[BeaconId-'A'];

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
			return MOOSFail("Mission file is missing one or more location "
							"parameters for LBL Beacon %c!\n", BeaconId);
		}
	}
			if (m_MissionReader.GetConfigurationParam("PUBLISHTO_TIME", s))
			{
				m_PublishedVariables = s;
				//MOOSTrace("Creating MOOS Variable for publishing: %s\n", s.c_str());	//&
				if ( !AddMOOSVariable(s, "", s, 0) )
				{
					return MOOSFail( "ERROR: Failed to create a dynamic MOOS "
									 "variable for publishing to %s!\n",
									 s.c_str() );
				}
			}
	




	return true;
}

bool pFakeModemReport::TimeCalculation( void )
	{
	
	double MTimeBegin = MOOSTime(false);
	double time_to_recieve[4];
	double distance;
	//double SSH20 = 1.470; //(m/ms)

	CMOOSVariable* pLatVar = GetMOOSVar(m_SubscribedVariables[GPS_LATITUDE]);
	CMOOSVariable* pLonVar = GetMOOSVar(m_SubscribedVariables[GPS_LONGITUDE]);
	CMOOSVariable* pDepthVar = GetMOOSVar(m_SubscribedVariables[DEPTH_Sensor]);

	if ( !pLatVar || !pLonVar || !pDepthVar )
	{
		MOOSTrace("Failed to get variables!\n");
		return true;
	}
	double Lat, Lon, NAV_D;
	Lat = pLatVar->GetDoubleVal();
	Lon = pLonVar->GetDoubleVal();
	NAV_D = pDepthVar->GetDoubleVal();

	/*
	double Lat = GetMOOSVar(m_SubscribedVariables[GPS_LATITUDE])->GetDoubleVal();
	double Long = GetMOOSVar(m_SubscribedVariables[GPS_LONGITUDE])->GetDoubleVal();
	double NAV_X, NAV_Y;
	double NAV_Z = GetMOOSVar(m_SubscribedVariables[DEPTH_Sensor])->GetDoubleVal();
	*/
	double NAV_X, NAV_Y;
	m_ReferenceGeodesy.LatLong2LocalGrid(Lat,Lon,NAV_Y,NAV_X);
	

	//beacon 1
	 for (int x = 0; x < 4; x++)
	 {	
		double distance_squared = (NAV_X-m_pBeacons[x]->GetX())*(NAV_X-m_pBeacons[x]->GetX())+(NAV_Y-m_pBeacons[x]->GetY())*
		(NAV_Y-m_pBeacons[x]->GetY())+(NAV_D-m_pBeacons[x]->GetDepth())*(NAV_D-m_pBeacons[x]->GetDepth());
		distance = sqrt(distance_squared);
		time_to_recieve[x] = (distance*2/SSH20);
		}
	 string fakemodemreport = MOOSFormat("TimeA=%f,TimeB=%f,TimeC=%f,TimeD=%f",
			 time_to_recieve[0],time_to_recieve[1],time_to_recieve[2],time_to_recieve[3]);
	double Max_Time = max(max(time_to_recieve[0],time_to_recieve[1]),max(time_to_recieve[2],time_to_recieve[3]));	
	double MTimeEnd = MOOSTime(false);
	double DeltaT = (MTimeEnd - MTimeBegin)*1000;
	double Timetowait = Max_Time-DeltaT;
	double abs_time = abs(Timetowait);
	MOOSPause(abs_time);
	m_Comms.Notify(m_PublishedVariables,fakemodemreport);
	//m_Comms.Notify("timetowait", Timetowait);


		
	


	
return true;
	}








