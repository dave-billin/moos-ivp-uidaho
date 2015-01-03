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
/** @file pVehicleID.cpp

@brief
	Implementation of the pVehicleID object
	
@author Dave Billin

*/
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSSerialPort.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "pVehicleID.h"


using namespace std;


//=============================================================================
/* Creates an instance of the object */
pVehicleID::pVehicleID( void )
  : m_VehicleID(0)
{
	m_PublishedVariables = "VehicleID";
}




//=============================================================================
/* Called when the object goes out of scope */
pVehicleID::~pVehicleID()
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
bool pVehicleID::OnNewMail(MOOSMSG_LIST & NewMail)
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
				// << Handle skewed messages here >>>
				continue;
			}

			// Ignore received PVehicleID_CMD messages if command message filtering is
			// enabled to prevent us from processing the same message twice.
			if ( m_bCommandMessageFiltering == true )
			{
				if ( MOOSStrCmp(RxMsg.GetKey(), GetCommandKey()) )
				{
					continue;
				}
			}

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
/*
@param Msg
	A copy of the received command message

@return
	true on success; false on error

@see CMOOSApp::EnableCommandMessageFiltering
*/
bool pVehicleID::OnCommandMsg(CMOOSMsg Msg)
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
bool pVehicleID::Iterate( void )
{
	m_Comms.Notify(m_PublishedVariables,m_VehicleID);
	return true;
}




//=============================================================================
/* Called when the application first starts up
@details
	This function is called as the application first starts up before any
	calls to Iterate() begin.
*/
bool pVehicleID::OnStartUp( void )
{
	LoadMissionParameters();
	bool GotDestAddress = false;
	string s;
	if (m_MissionReader.GetValue("VEHICLE_CONFIG_FILE_PATH", s))
	{
		CMOOSFileReader Fr;
		if (!s.empty())
		{
			// Open the vehicle-specific config file and read in the vehicle ID
			if (Fr.SetFile(s))
			{
				// If we can read the vehicle ID, we'll use it as the radio modem's
				// Destination address.
				if (Fr.GetValue("AUV_VEHICLE_ID", m_VehicleID) )
				{
					GotDestAddress = true;

				}
			}
		}
	}


	// If we failed to load the Destination ID from the vehicle config file, load
	// the value of the DEFAULT_MODEM_ADDRESS parameter instead
	if (!GotDestAddress)
	{
		// Read acoustic ID from iWhoiMicroModem NVRAM settings instead
		if (m_MissionReader.GetConfigurationParam("DEFAULT_MODEM_ADDRESS", m_VehicleID))
		{
			return true;
		}
		else
		{
			// Exit on failure to obtain a destination address
			return MOOSFail("Failed to load modem address from mission file!  This value must be "
							"specified in the mission file with parameter DEFAULT_MODEM_ADDRESS or "
							"in the VEHICLE_ID parameter of a vehicle-specific configuration file.\n\n" );
		}
	}


	return true;
}




//=============================================================================
/* Called when the application connects to the MOOS Database. */
bool pVehicleID::OnConnectToServer( void )
{
	/*-----------------------------------------------------
		The application is now connected to the MOOS 
		Database.  This is a good place to register 
		MOOS variables the application subscribes to
	-----------------------------------------------------*/

	return true;
}




//=============================================================================
/* Called when the application is disconnected from the MOOS Database. */
bool pVehicleID::OnDisconnectFromServer( void )
{
	/*-----------------------------------------------------
		The application is now disconnected from the 
		MOOS Database.
	-----------------------------------------------------*/
	return true;
}

bool pVehicleID::LoadMissionParameters(void)
{
	//------------------------------------------------------
	// Load optional PUBLISHTO_nnn parameters
	//------------------------------------------------------
    string s;
    if (m_MissionReader.GetConfigurationParam("PUBLISHTO_VEHICLEID", s))
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






