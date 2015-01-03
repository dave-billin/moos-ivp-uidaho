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
/*! @file iXStreamRadioModem.cpp

@brief
	Implementation of the iXStreamRadioModem class

@author Dave Billin

@par Created For
	The University of Idaho Microelectronics Research and Communications
	Institute (MRCI)
*/
//=============================================================================

#include <stdint.h>
#include "iXStreamRadioModem.h"
#include <unistd.h> // For the sleep() function
#include <string.h>

using namespace std;

//===============================================
// FILE SCOPE CONSTANTS
//===============================================

// Quiet time preceding and following AT commands.  Must be a multiple of 100 ms
#define ATCOMMAND_GUARDTIME_MS 200




// Create the transmit and receive MOOS variables based on the registered application name
// to allow multiple radio modems to peacefully co-exist in a MOOS community
#define TRANSMIT_VARIABLE GetAppName() + "_TRANSMIT"
#define RECEIVE_VARIABLE GetAppName() + "_RECEIVE"


// Implement verbosity control
#ifndef XSTREAM_DEBUG_VERBOSITY
	#define XSTREAM_DEBUG_VERBOSITY 2
#endif
#if XSTREAM_DEBUG_VERBOSITY >= 1
	#define RADIO_MODEM_ERROR(_expr_) _expr_
#else
	#define RADIO_MODEM_ERROR(_expr_)
#endif
#if XSTREAM_DEBUG_VERBOSITY >= 2
	#define RADIO_MODEM_WARNING(_expr_) _expr_
#else
	#define RADIO_MODEM_WARNING(_expr_)
#endif
#if XSTREAM_DEBUG_VERBOSITY >= 3
	#define RADIO_MODEM_NOTE(_expr_) _expr_
#else
	#define RADIO_MODEM_NOTE(_expr_)
#endif


const string iXStreamRadioModem::ATCommand_OK("OK");
const string iXStreamRadioModem::ATCommand_Error("ERROR");


const char* iXStreamRadioModem::sm_szMoosSensorVariableParams[] =
	{
		"MOOSVAR_COMPASS_HEADING_DEG",
		"MOOSVAR_DEPTH_M",
		"MOOSVAR_BATTERY_VOLTS",
		"MOOSVAR_H20_LEAK_DETECTED",
		"MOOSVAR_TEMPERATURE_C",
		"MOOSVAR_LONGITUDE_DEG",
		"MOOSVAR_LATITUDE_DEG",
		"MOOSVAR_GPS_VELOCITY_MPS",
		"MOOSVAR_GPS_HPE_M",
		"MOOSVAR_GPS_HEADING_DEG",
		"MOOSVAR_GYRO_PITCH_RAD",
		"MOOSVAR_GYRO_ROLL_RAD",
		"MOOSVAR_MISSION_NUMBER",
		"MOOSVAR_RUN_NUMBER",
		"MOOSVAR_ACTUAL_THRUST_PERCENT",
		"MOOSVAR_LAST_ABORT_CODE",
		"MOOSVAR_LBL_BUOY_FLAGS",
		"MOOSVAR_COORDINATE_N",
		"MOOSVAR_COORDINATE_E"
	};

const char* iXStreamRadioModem::sm_szMoosSensorVarNames[] =
	{
		"COMPASS_HEADING_DEG",
		"DEPTH_M",
		"BATTERY_VOLTS",
		"H20_LEAK_DETECTED",
		"TEMPERATURE_C",
		"LONGITUDE_DEG",
		"LATITUDE_DEG",
		"GPS_VELOCITY_MPS",
		"GPS_HPE_M",
		"GPS_HEADING_DEG",
		"GYRO_PITCH_RAD",
		"GYRO_ROLL_RAD",
		"MISSION_NUMBER",
		"RUN_NUMBER",
		"ACTUAL_THRUST_PERCENT",
		"LAST_ABORT_CODE",
		"LBL_BUOY_FLAGS",
		"COORDINATE_N",
		"COORDINATE_E"
	};


//=============================================================================
// Helper function to replace non-printing characters in debugging Tx/Rx strings
static string ReplaceNonPrintables( string sSrc )
{
	string::size_type p;
	string sSubst;

	// Replace Newlines and Tabs with character equivalents
	while ( (p = sSrc.find_first_of("\n\r",0)) != string::npos )
	{
		switch (sSrc[p])
		{
		case '\n':
			sSubst = "\\n";
			break;

		case '\r':
			sSubst = "\\r";
			break;

		case '\t':
			sSubst = "\\t";
			break;

		default:
			sSubst = "  ";
			break;
		}

		sSrc = sSrc.replace(p, sSubst.length(), sSubst.c_str());
	}

	return sSrc;
}




//=============================================================================
// Converts a hexadecimal-encoded string value to a long
static long HexString2Long(string& sSrc)
{
	char* p;
	return strtol(sSrc.c_str(), &p, 16 );
}




unsigned int m_HoppingChannel;		//!< Frequency hopping channel
unsigned int m_AddressMask;			//!< Radio networking address mask
unsigned int m_VID;					//!< Modem vendor ID

//=============================================================================
iXStreamRadioModem::iXStreamRadioModem()
:	m_ATCommandModeIsActive(false),
	m_PrintTxData(false),
	m_PrintRxData(false),
	m_SensorReportPeriod_sec(0),
	m_DestinationAddress(0),
	m_HoppingChannel(0),
	m_AddressMask(0),
	m_VID(0),
	m_FirmwareVersion(0)
{
	m_SerialNumber[0] = m_SerialNumber[1] = 0;	// Init modem serial number

}






//=============================================================================
iXStreamRadioModem::~iXStreamRadioModem()
{

}





//=============================================================================
bool iXStreamRadioModem::OnStartUp()
{
	bool GotDestAddress = false;
	string s;
	string sBar = string(40, '=') + '\n';

	MOOSTrace("\n\n");
	MOOSTrace(sBar + GetAppName() + "\n");
	MOOSTrace( MOOSFormat("v%3.2f\n", IXSTREAMRADIOMODEM_SOFTWARE_VERSION) );
	MOOSTrace( "\nCreated by Dave Billin\n" + sBar + "\n" );

	//-----------------------------------------
	// Load mission file parameters
	//-----------------------------------------
	RADIO_MODEM_NOTE( MOOSTrace((char *)"Loading mission file parameters\n") );

	// Read path of vehicle-specific data from the mission file (global value)
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
				if (Fr.GetValue("AUV_VEHICLE_ID", m_DestinationAddress) )
				{
					RADIO_MODEM_NOTE( MOOSTrace((char *)"Loaded modem address from "
														"%s.\n", s.c_str()) );

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
		if (m_MissionReader.GetConfigurationParam("DEFAULT_MODEM_ADDRESS", m_DestinationAddress))
		{
			RADIO_MODEM_NOTE( MOOSTrace((char *)"Loaded modem address from mission file parameter "
												"DEFAULT_MODEM_ADDRESS\n") );
		}
		else
		{
			// Exit on failure to obtain a destination address
			return MOOSFail("Failed to load modem address from mission file!  This value must be "
							"specified in the mission file with parameter DEFAULT_MODEM_ADDRESS or "
							"in the VEHICLE_ID parameter of a vehicle-specific configuration file.\n\n" );
		}
	}

	// Bound modem address
	m_DestinationAddress = (m_DestinationAddress > 8) ?
								8 : m_DestinationAddress;

	// Load the remaining networking parameters
	if (!m_MissionReader.GetConfigurationParam("HOPPING_CHANNEL", m_HoppingChannel))
	{
		return MOOSFail("Failed to load network hopping channel from mission file!  This value "
						"must be specified in the mission file parameter HOPPING_CHANNEL.\n\n" );
	}

	if (!m_MissionReader.GetConfigurationParam("ADDRESS_MASK", s))
	{
		return MOOSFail("Failed to load radio network address mask from mission file!  This value "
						"must be specified in the mission file parameter ADDRESS_MASK.\n\n" );
	}
	else
	{
		m_AddressMask = (unsigned int)HexString2Long(s);
	}


	// Get the (optional) period in seconds at which sensor reports are transmitted to the
	// AUV Control Panel software
	m_MissionReader.GetConfigurationParam("SENSOR_REPORT_PERIOD_SEC", m_SensorReportPeriod_sec);

	// Load the names of MOOS variables containing sensors reported by the radio modem
	for (int i = 0; i < NUM_SENSOR_VARIABLES; i++)
	{
		s = sm_szMoosSensorVarNames[i];

		if (m_MissionReader.GetConfigurationParam(sm_szMoosSensorVariableParams[i], s))
		{
			if ( !AddMOOSVariable(sm_szMoosSensorVarNames[i], s, "", 0) )
			{
				return MOOSFail( "ERROR: Failed to create dynamic MOOS variable to subscribe to %s!\n",
						        s.c_str() );
			}
		}
		else
		{
			if (m_SensorReportPeriod_sec > 0)
			{
				MOOSTrace("WARNING: Sensor Reports are enabled, but no MOOS Variable name was\n"
						  "specified for mission parameter %s.  Subscribing to default variable\n"
						  "%s instead.\n",
						  sm_szMoosSensorVariableParams[i], sm_szMoosSensorVarNames[i] );
			}

		}

	}


	// Get debug parameter values
	m_MissionReader.GetConfigurationParam("PRINT_SERIAL_TX", m_PrintTxData);
	m_MissionReader.GetConfigurationParam("PRINT_SERIAL_RX", m_PrintRxData);

	// Setup serial port using settings from moos configuration file
	if ( !SetupPort() )
	{
		return MOOSFail("Failed to configure serial port!\n");
	}
	MOOSPause(300);

	// Configure the modem
	if ( !InitialiseSensor() )
	{
		return MOOSFail("Failed to initialize modem!\n");
	}


	// Print a summary of operating conditions

	MOOSTrace("\n" + sBar +
			  MOOSFormat("<< MODEM IS ONLINE >>\n"
						 "Baud Rate: %d\n"
						 "Modem Address: %d\n"
						 "Hopping Channel: %d\n"
						 "Address Mask: 0x%x\n",
						 m_Port.GetBaudRate(), m_DestinationAddress,
						 m_HoppingChannel, m_AddressMask ) +
			  sBar );

	// Register for sensor variables
	RegisterMOOSVariables();

	return true;
}






//=============================================================================
bool iXStreamRadioModem::OnConnectToServer()
{
	string sTxVariable = TRANSMIT_VARIABLE;
	string sRxVariable = RECEIVE_VARIABLE;
	const char szErr[] = "Failed to subscribe to MOOS variable";

	// Register for sensor variables
	if ( !RegisterMOOSVariables() )
	{
		MOOSTrace("WARNING: Failed to register MOOS variables for sensor data!\n");
	}

	// Register for generic transmit variable with MOOSDB
	if (!m_Comms.Register(sTxVariable, 0))
	{
		return MOOSFail((char *)"%s \"%s\"!\n", szErr, sTxVariable.c_str() );
	}

	// Register for generic receive variable with MOOSDB
	if (!m_Comms.Register(sRxVariable, 0))
	{
		return MOOSFail((char *)"%s \"%s\"!\n", sRxVariable.c_str() );
	}

	return true;
}






//=========================================================================
bool iXStreamRadioModem::InitialiseSensor( void )
{
	unsigned int UintValue;
	int UpdateCode = 0;
	bool SettingsDirty = false;
	bool ReturnCode = true;


	MOOSTrace("\n<< Configuring XStream modem settings >>\n");

	//-------------------------------
	// CONFIGURE THE MODEM
	//-------------------------------
	if ( !EnterATCommandMode() )
	{
		ReturnCode = MOOSFail("Failed to enter AT command mode when configuring the modem!\n");
	}
	else
	{
		//--------------------------------------
		// Configure AT command guard times
		//	Per the manual, guard time values
		//	are specified in increments of 100ms
		//--------------------------------------
		RADIO_MODEM_NOTE(MOOSTrace((char *)"\nConfiguring AT command guard times...\n"));
		SettingsDirty |= (UpdateModemRegister("AT", 2) > 0);
		SettingsDirty |= (UpdateModemRegister("BT", 2) > 0);


		//--------------------------------------
		// Set radio hopping channel
		//--------------------------------------
		RADIO_MODEM_NOTE(MOOSTrace((char *)"Setting hopping channel...\n"));
		UpdateCode = UpdateModemRegister("HP", m_HoppingChannel);
		if ( UpdateCode < 0 )
		{
			ReturnCode &= MOOSFail("Failed to set hopping channel!\n");
		}
		SettingsDirty |= (UpdateCode > 0);


		//------------------------------------------------------------
		// Set the modem's network address using Troy Cuff's one-hot
		// coding scheme.
		//------------------------------------------------------------
		RADIO_MODEM_NOTE(MOOSTrace((char *)"Setting modem address...\n"));
		UintValue = 1 << (m_DestinationAddress-1);
		UpdateCode = UpdateModemRegister("DT", UintValue);
		if ( UpdateCode < 0 )
		{
			ReturnCode &= MOOSFail("Failed to set modem's address!\n");
		}
		SettingsDirty |= (UpdateCode > 0);


		RADIO_MODEM_NOTE(MOOSTrace((char *)"Setting address mask...\n"));
		UpdateCode = UpdateModemRegister("MK", UintValue);
		if ( UpdateCode < 0 )
		{
			ReturnCode &= MOOSFail("Failed to set address mask!\n");
		}
		SettingsDirty |= (UpdateCode > 0);


		//--------------------------------------
		// Read the modem's firmware version
		//--------------------------------------
		if ( !GetModemRegister("VR", m_FirmwareVersion) )
		{
			MOOSTrace((char *)"Failed to read modem firmware version!\n");
		}
		else
		{
			MOOSTrace("\nModem Firmware Version: %d\n", m_FirmwareVersion);
		}

		//--------------------------------------
		// Read the modem's serial number
		//--------------------------------------
		GetModemRegister("SL", m_SerialNumber[0]);		// Serial number (low)
		GetModemRegister("SH", m_SerialNumber[1]);		// Serial number (high)
		if ( (m_SerialNumber[0] == 0) && (m_SerialNumber[1] == 0) )
		{
			MOOSTrace((char *)"Failed to read modem serial number!\n");
		}
		else
		{
			MOOSTrace("Modem Serial Number: 0x%04x%04x\n", m_SerialNumber[1], m_SerialNumber[0]);
		}

		MOOSTrace("\n\n");

	}

	if (m_ATCommandModeIsActive)
	{
		ExitATCommandMode();
	}

	return ReturnCode;
}






//=============================================================================
bool iXStreamRadioModem::SetupPort()
{
	list <string> PortSettingsList;
	string sValue;
	string sKey;
	const char* szMissionParams[] = { "PORT", "BAUDRATE", "HANDSHAKING", "VERBOSE",
									  "USECSMEXT" };

	// DB NOTE: When assembling elements in the PortSettingsList, no spaces are
	// allowed between the key, the equal sign ('='), and the value.

	// Disable serial port streaming
	PortSettingsList.push_back("STREAMING=FALSE");

	// Gather all remaining serial port settings from the mission file
	for (int i = 0; i < 5; i++)
	{
		sKey = szMissionParams[i];

		// Attempt to read the serial port setting from the mission file
		if ( m_MissionReader.GetConfigurationParam(sKey, sValue) )
		{
			PortSettingsList.push_back( MOOSFormat("%s=%s", szMissionParams[i], sValue.c_str()) );
		}
	}

	// Warn users who are trying to create a streaming serial port
	bool bStreaming;
	if (m_MissionReader.GetConfigurationParam("STREAMING", bStreaming))
	{
		if (bStreaming == true)
		{
			MOOSTrace("\n-------------------------------------------------------\n"
					  "NOTE: This MOOS instrument does not support streaming\n"
					  "serial ports.  The 'STREAMING' mission file parameter\n"
					  "was ignored.\n"
					  "-------------------------------------------------------\n\n");
		}
	}

	// Configure the serial port
    if(!m_Port.Configure(PortSettingsList))
    {
		return MOOSFail("Failed to configure serial port!!\n");
    }

    m_Port.Flush();		// Flush the serial port

	// Set termination character for receiving terminated serial messages
	m_Port.SetTermCharacter('\r');

	return true;
}






//=============================================================================
bool iXStreamRadioModem::Iterate( void )
{
	static double ReferenceSeconds = 0;
	double CurrentSeconds;

	ServiceAUVControlPanelRx(m_Port, m_Comms);		// Parse and handle incoming AUV radio comms

	CurrentSeconds = MOOSTime();
	if ( (CurrentSeconds - ReferenceSeconds) > m_SensorReportPeriod_sec)
	{
		// Assemble a sensor report and broadcast it
		m_SensorReport.VehicleId = m_DestinationAddress;
		m_SensorReport.CompassHeading_deg = (float)(GetMOOSVar(sm_szMoosSensorVarNames[COMPASS_HEADING_DEG])->GetDoubleVal());

		// Convert MOOS depth in meters to centimeters for dockside display
		m_SensorReport.Depth_cm = (float)(GetMOOSVar(sm_szMoosSensorVarNames[DEPTH_M])->GetDoubleVal() * 100.0);
		m_SensorReport.BatteryVolts = (float)(GetMOOSVar(sm_szMoosSensorVarNames[BATTERY_VOLTS])->GetDoubleVal());

		// Reverse logic on H20 leak for dockside display
		CMOOSVariable* pLeakVar = GetMOOSVar(sm_szMoosSensorVarNames[H20_LEAK_DETECTED]);
		string s = pLeakVar->GetStringVal();
		m_SensorReport.H20Leak = ( MOOSStrCmp(s, "FALSE") ) ? 1:0;

		m_SensorReport.Temperature_C = (float)(GetMOOSVar(sm_szMoosSensorVarNames[TEMPERATURE_C])->GetDoubleVal());
		m_SensorReport.Longitude_deg = (float)(GetMOOSVar(sm_szMoosSensorVarNames[LONGITUDE_DEG])->GetDoubleVal());
		m_SensorReport.Latitude_deg = (float)(GetMOOSVar(sm_szMoosSensorVarNames[LATITUDE_DEG])->GetDoubleVal());
		m_SensorReport.Velocity_mps = (float)(GetMOOSVar(sm_szMoosSensorVarNames[GPS_VELOCITY_MPS])->GetDoubleVal());
		m_SensorReport.GpsHPE = (float)(GetMOOSVar(sm_szMoosSensorVarNames[GPS_HPE_M])->GetDoubleVal());
		m_SensorReport.GpsHeading = (float)(GetMOOSVar(sm_szMoosSensorVarNames[GPS_HEADING_DEG])->GetDoubleVal());

		// DB: Note - pitch and roll are swapped here because MOOS conventions for X and Y axes are opposite those
		// used in the original AUV implementation.  Hence, Pitch (rotation about X-axis) is reported to dockside
		// as Roll...etc.  These are converted to degrees for Dockside as well.
		m_SensorReport.Acceleration_X = (float)(MOOSRad2Deg(GetMOOSVar(sm_szMoosSensorVarNames[GYRO_ROLL_RAD])->GetDoubleVal()));
		m_SensorReport.Acceleration_Y = (float)(MOOSRad2Deg(GetMOOSVar(sm_szMoosSensorVarNames[GYRO_PITCH_RAD])->GetDoubleVal()));

		m_SensorReport.MissionNumber = (char)(GetMOOSVar(sm_szMoosSensorVarNames[MISSION_NUMBER])->GetDoubleVal());
		m_SensorReport.RunNumber = (unsigned char)(GetMOOSVar(sm_szMoosSensorVarNames[RUN_NUMBER])->GetDoubleVal());
		m_SensorReport.PropSpeedIndex = (unsigned char)(GetMOOSVar(sm_szMoosSensorVarNames[ACTUAL_THRUST_PERCENT])->GetDoubleVal());
		m_SensorReport.LastAbortCode = (char)(GetMOOSVar(sm_szMoosSensorVarNames[LAST_ABORT_CODE])->GetDoubleVal());
		m_SensorReport.LBL_TransponderFlags = (char)(GetMOOSVar(sm_szMoosSensorVarNames[LBL_BUOY_FLAGS])->GetDoubleVal());
		m_SensorReport.Coordinate_North = (float)(GetMOOSVar(sm_szMoosSensorVarNames[COORDINATE_N])->GetDoubleVal());
		m_SensorReport.Coordinate_East = (float)(GetMOOSVar(sm_szMoosSensorVarNames[COORDINATE_E])->GetDoubleVal());

		m_Port.Write( m_SensorReport.Serialize(), m_SensorReport.GetNumSerialBytes() );

		/*
		char Tx[3] = { 'N', 0, 1 };			// Send "I'm here" message
		Tx[1] = (char)m_DestinationAddress;
		m_Port.Write(Tx, 3);
		*/
		ReferenceSeconds = CurrentSeconds;
	}

	return true;
}








//=============================================================================
/** Called when new mail has arrived from the MOOS Database.
@details
	This method will be called at a rate determined by the CommsTick
	mission file parameter.  In this function you'll most likely interate
	over the collection of new mail messages received or call
	m_Comms::PeekMail() to look for a specific message.

@param NewMail
	A reference to the list of received mail from the MOOS Database

@return
	true on success; false on error
*/
bool iXStreamRadioModem::OnNewMail(MOOSMSG_LIST& NewMail)
{
	// If we're actually running...
	if ( !IsSimulateMode() )
	{
		UpdateMOOSVariables(NewMail);

		// Walk through the list of received messages
/*		MOOSMSG_LIST::iterator iter;
		for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
		{
			// Identify skewed messages
			CMOOSMsg& RxMsg = *iter;
			if( RxMsg.IsSkewed( MOOSTime() ) )
			{
				RADIO_MODEM_WARNING( MOOSTrace("Discarding skewed Tx data message: " +
											   RxMsg.GetString() + "\n" ));
				continue;
			}

			if ( MOOSStrCmp(RxMsg.GetKey(), TRANSMIT_VARIABLE) )
			{
				if (!m_ATCommandModeIsActive)
				{
					string TxData = RxMsg.GetString();
					SendToModem(TxData + "\r");
				}
			}

		}
	*/

	}

	return true;
}







//=============================================================================
bool iXStreamRadioModem::EnterATCommandMode()
{
	string s("+++"); 	// opcode to enter AT command mode
	int i;
	bool ReturnCode = false;

	MOOSTrace("Entering AT Command Mode\n");

	RADIO_MODEM_NOTE( MOOSTrace("Telling base station to disable ROV commands\n") );
	// Tell the base station to stop sending ROV commands
	RM_ROVEnablementPacket RovPacket( (unsigned char)m_DestinationAddress,
									  false );
	m_Port.Write( RovPacket.Serialize(), RovPacket.GetNumBytes(), NULL);

	m_Port.Flush();

	// Enforce guard time preceding entry of command mode
	for (i = 0; i < 4; i++)
	{
		MOOSPause(500);
		MOOSTrace(".");
	}

	m_Port.Flush();
	SendToModem(s);

	// Enforce guard time after entry of command mode
	for (i = 0; i < 4; i++)
	{
		MOOSPause(500);
		MOOSTrace(".");
	}

	// Check the modem's response
	ReturnCode = CheckATCommandResponse(ATCommand_OK, s);
	m_ATCommandModeIsActive = ReturnCode;

	return ReturnCode; // Otherwise it failed
}








//=============================================================================
bool iXStreamRadioModem::ExitATCommandMode()
{
	string s = "ATCN\r"; 	// opcode to exit AT command mode
	bool ReturnCode = false;

	// If command mode isn't active, do nothing
	if (!m_ATCommandModeIsActive)
	{
		return true;
	}

	MOOSTrace("Exiting AT Command Mode\n");

	m_Port.Flush();
	SendToModem(s);

	MOOSPause(ATCOMMAND_GUARDTIME_MS);

	// STATUS:
	//	The serial port must be in streaming mode, so simply wait for
	//	the next telegram to arrive

	// Check the modem's response
	ReturnCode = CheckATCommandResponse(ATCommand_OK, s);
	m_ATCommandModeIsActive = !ReturnCode;

	// If we've left AT command mode, tell the base station it's OK to send ROV
	// commands.
	if (!m_ATCommandModeIsActive)
	{
		RADIO_MODEM_NOTE( MOOSTrace("Telling base station to enable ROV commands\n") );
		// Tell the base station it can start sending ROV commands
		RM_ROVEnablementPacket RovPacket((unsigned char)m_DestinationAddress,
										 true );
		m_Port.Write( RovPacket.Serialize(), RovPacket.GetNumBytes(), NULL);

	}

	return ReturnCode;
}










//=============================================================================
bool iXStreamRadioModem::GetModemRegister( string sRegisterId, unsigned int &OUT_RegisterValue )
{
	string s = "AT" + sRegisterId + "\r";
	bool ReturnCode = true;

	// NOTE: AT command pre guard time is not enforced.  Post-guard time will
	// fulfill this requirement for consecutive commands.

	m_Port.Flush();
	SendToModem(s);

	MOOSPause(ATCOMMAND_GUARDTIME_MS);	// Enforce post-command guard time

	// STATUS:
	//	The serial port must be in streaming mode, so simply wait for
	//	the next telegram to arrive

	// Check the modem's response
	if ( GetModemCommandResponse(s) )
	{
		if ( !MOOSStrCmp(s, ATCommand_Error) )
		{
			OUT_RegisterValue = atoi(s.c_str());
		}
		else
		{
			ReturnCode = false;
			RADIO_MODEM_ERROR( MOOSTrace("Modem returned ERROR reading modem register %s\n", sRegisterId.c_str()) );
		}
	}
	else
	{
		ReturnCode = false;
	}

	return ReturnCode;
}





//=============================================================================
int iXStreamRadioModem::UpdateModemRegister( string sRegisterId, unsigned int DesiredValue )
{
	unsigned int RegisterValue;
	if ( !GetModemRegister(sRegisterId, RegisterValue) )
	{
		return false;	// Failed to read the register value
	}

	if (RegisterValue != DesiredValue)
	{
		RADIO_MODEM_NOTE( MOOSTrace( MOOSFormat("AT register %s updated from %X to %X\n",
												sRegisterId.c_str(),
												RegisterValue,
												DesiredValue)));
		// Apply the desired value
		return ( SendATCommand( sRegisterId, DesiredValue ) ) ? 1:-1;
	}
	else
	{
		// Register already has the desired value
		return 0;
	}
}




//=============================================================================
bool iXStreamRadioModem::SendATCommand( string sCommandId, unsigned int CommandValue )
{
	string s;
	bool ReturnCode;

	//-------------------------------
	// Create the register command
	//-------------------------------
	string sCmd = "AT" + sCommandId + MOOSFormat("%x", CommandValue) + "\r";

	// NOTE: AT command pre-guard time is not enforced.  Post-guard time will
	// fulfill this requirement for consecutive commands.
	m_Port.Flush();
	SendToModem(sCmd);

	MOOSPause(ATCOMMAND_GUARDTIME_MS);	// Enforce post-command guard time

	// Check modem response
	ReturnCode = CheckATCommandResponse(ATCommand_OK, s);

	if (!ReturnCode)
	{
		if (MOOSStrCmp(s, ATCommand_Error) )
		{
			MOOSTrace("Modem returned ERROR in response to:\n"
					  "AT command: %s\n"
					  "Value: %d\n\n", sCommandId.c_str(), CommandValue );
		}
	}

	return ReturnCode;
}






//=============================================================================
bool iXStreamRadioModem::SendToModem( string sTxData )
{
	double TxTime;
	int NumDataBytes = sTxData.length();
	int NumBytesWritten;

	NumBytesWritten = m_Port.Write(sTxData.c_str(), NumDataBytes, &TxTime);

	if (m_PrintTxData)
	{
		string sTemp = ReplaceNonPrintables(sTxData);
		MOOSTrace("[Tx: %3.3f sec] \"%s\"\n", TxTime, sTemp.c_str() );
	}

	return NumBytesWritten == NumDataBytes;
}



//=============================================================================
bool iXStreamRadioModem::GetModemCommandResponse( string& OUT_sData )
{
	double TimeStamp;
	OUT_sData.clear();
	bool ReturnCode;
	string sRx;

	//char term = m_Port.GetTermCharacter();
	//ReturnCode = m_Port.IsStreaming();

	// NOTE: The port has been configured in NON-streaming mode with termination
	// character '\r' to allow modem responses to be easily received as
	// 'telegrams'

	// Wait up to 2 seconds for a response from the modem
	ReturnCode = m_Port.GetTelegram(sRx, 2, &TimeStamp);
	OUT_sData = sRx;

	if (m_PrintRxData && ReturnCode)
	{
		MOOSTrace( "[Rx: %3.3f sec] \"%s\"\n", TimeStamp, OUT_sData.c_str() );
	}

	return ReturnCode;
}




//=============================================================================
bool iXStreamRadioModem::CheckATCommandResponse(
                                            string const& sExpectedResponse,
                                            string& OUT_ReceivedResponse )
{
	OUT_ReceivedResponse.clear();

	// Check the modem's response
	if ( GetModemCommandResponse(OUT_ReceivedResponse) )
	{
	    return MOOSStrCmp(OUT_ReceivedResponse, sExpectedResponse);
	}
	else
	{
		return false;
	}
}

