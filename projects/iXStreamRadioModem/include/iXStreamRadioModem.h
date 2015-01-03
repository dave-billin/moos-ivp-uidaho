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
/*! @file iXStreamRadioModem.h

@brief
	Declaration of a MOOS instrument class used to implement communication
	via an XStream OEM RF Module (Radio Modem).

@author Dave Billin

@par Created For
	The University of Idaho Microelectronics Research and Communications
	Institute (MRCI)
*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _IXSTREAMRADIOMODEM_H_
#define _IXSTREAMRADIOMODEM_H_

#include <string>
#include "MOOS/libMOOS/MOOSLib.h"	// MOOS core library

#include "AuvControlPanelClient.h"	// Classes for AUV control panel comms




//=====================================
// CONSTANTS AND MACROS
//=====================================

// The application's software revision
#define IXSTREAMRADIOMODEM_SOFTWARE_VERSION 0.1


/** @def XSTREAM_DEBUG_VERBOSITY
@brief
	Controls the amount of debug messages printed by this module. Valid
	parameter values are:
		- 0: Debugging messages disabled
		- 1: Error messages only
		- 2: Warning and error messages
		- 3: Note, warning, and error messages
*/
#define XSTREAM_DEBUG_VERBOSITY 3





//=============================================================================
/** This is the main application object for the iXStreamRadioModem
	MOOS application.
*/
//=============================================================================
class iXStreamRadioModem : public CMOOSInstrument
{
public:

	//=========================================================================
	/** Creates an instance of the MOOS application object */
	iXStreamRadioModem();

	//=========================================================================
	/** Called when the application closes */
	virtual ~iXStreamRadioModem();


	//=========================================================================
    /** Called when the application first starts up
	@details
		- Initializes iXStreamRadioModem settings from the mission file.
		- Sets AT and BT register values to 2 (minimum allowed settings) to
		  provide faster AT command	communications.
		- Determines the current sub number in network and returns it if no
		  errors.
	*/
    bool OnStartUp( void );


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called from OnStartUp before iXStreamRadioModem connects to the MOOS
		database to initialize and configure the XStream modem module */
	bool InitialiseSensor( void );

	//=========================================================================
	/** Called to set up the serial port connection using parameter values from
		the mission file. */
	bool SetupPort();



	//=========================================================================
    /** This function is where data is received from the modem.

	@details
		The rate at which Iterate() is called is determined by the value of the
		AppTick parameter specified in the (.moos) mission file.  Data received
		from the XStream modem module is published to the MOOS variable
		[APP_NAME]_RxData.  This allows multiple instances of
		iXStreamRadioModem with different registered app names to run in a MOOS
		community.
	*/
    bool Iterate( void );


	//=========================================================================
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
	bool OnNewMail(MOOSMSG_LIST & NewMail);



protected:

	//=========================================================================
	/** Causes the XStream radio modem module to enter command mode for
		configuration.  While in this mode data cannot be sent or received.
	*/
	bool EnterATCommandMode();

	//=========================================================================
	/** Causes the XStream radio modem module to exit command mode. */
	bool ExitATCommandMode();


	//=========================================================================
	/** Returns the value of a modem configuration register.

	@param[in] sRegisterId
		A 2-character string identifying the modem register to return

	@param[out] OUT_RegisterValue
		A reference to the variable to be populated with the 16-bit register
		value.

	@precondition
		The modem is assumed to already be in AT command mode

	@return
		true if the register was read; else false on error.
	*/
	bool GetModemRegister( std::string sRegisterId,
	                       unsigned int &OUT_RegisterValue );


	//=========================================================================
	/** Checks the value of a specified modem register against a desire value
	  	and applies the desired value if a match is not found

	@param sRegisterId
		The 2-character string identifying the modem register to check

	@param DesiredValue
		The desired value of the register

	@return
		- 0 if the modem register value matches the desired value
		- 1 if the modem register was updated with the desired value
		- (-1) on failure to read the modem register or apply the desired
		  value
	 */
	int UpdateModemRegister( std::string sRegisterId,
	                         unsigned int DesiredValue );



	//=========================================================================
	/** Sets the value of a modem configuration register

	@param[in] sCommandId
		A 2-character string identifying the command or modem register to write
		to

	@param[in] CommandValue
		The 16-bit command/register value

	@precondition
		The modem is assumed to already be in AT command mode

	@return
		true if the specified value was written to the register; else false
	*/
	bool SendATCommand( std::string sCommandId, unsigned int CommandValue );


	//=========================================================================
	/** Wrapper function for sending data to the modem
	@param[in] sTxData		Data Bytes to be sent to the modem
	@return
		true if all data was sent; else false
	*/
	bool SendToModem( std::string sTxData );

	//=========================================================================
	/** Function to read an AT command response.  Waits up to 2 seconds to
		receive data from the modem before giving up

	@param[out] OUT_sData
		Reference to a string where received data will be placed

	@return
		true if a response was received; else false on error or if 2 second
		timeout elapsed
	*/
	bool GetModemCommandResponse( std::string& OUT_sData );

	//=========================================================================
	/** Reads and checks an AT command response against an expected value
	@param sExpectedResponse
		The expected command response

	@param OUT_ReceivedResponse
		Reference to a string where the actual modem response will be placed

	@return
		true if the command response is equal to sExpectedResponse; else false
		if not equal or if no response was received.
	*/
	bool CheckATCommandResponse( std::string const& sExpectedResponse,
	                             std::string& OUT_ReceivedResponse );


	//=========================================================================
	/**	A really hacked-in job of implementing the AUV radio communications as
		a temporary work-around.

	@details
		This is very much a hack job to get the Radio Modem functioning with
		the current radio comms implementation alongside Rabbit-based AUV's.
		Since the dockside software sends commands over the radio modem that
		don't include any any terminator character, we're locked into a comms
		implementation specific to the AUV.  We do packet decoding and
		reconstruction here instead of simply passing terminated data/commands
		along to another MOOS app that can handle it. Hopefully, the dockside
		software will be improved sometime soon so we can eliminate	this
		problem and move to a more generalized radio communication system.
	 */
	void AUVRadioCommRxHack(void);

private:
	bool m_ATCommandModeIsActive;		//!< true if modem is in command mode
	bool m_PrintTxData;					//!< true to print Tx data to stdio
	bool m_PrintRxData;					//!< true to print Rx data to stdio

	unsigned int m_SensorReportPeriod_sec;	/**< Period at which sensor reports
												 are sent to the AUV Control
												 Panel software Default period
												 is 1 second */

	unsigned int m_DestinationAddress;	//!< Address used for receiving
	unsigned int m_HoppingChannel;		//!< Frequency hopping channel
	unsigned int m_AddressMask;			//!< Radio networking address mask
	unsigned int m_VID;					//!< Modem vendor ID

	unsigned int m_FirmwareVersion;		//!< Firmware version reported by modem
	unsigned int m_SerialNumber[2];		//!< Serial number reported by modem

	static const std::string ATCommand_OK;		// Command OK string
	static const std::string ATCommand_Error;	// Command error string

	/** Enumeration used to index sm_MoosSensorVariableParams and
		sm_MoosSensorVarNames during runtime
	 */
	enum e_SensorVariable_NameIndex
	{
		COMPASS_HEADING_DEG = 0,
		DEPTH_M,
		BATTERY_VOLTS,
		H20_LEAK_DETECTED,
		TEMPERATURE_C,
		LONGITUDE_DEG,
		LATITUDE_DEG,
		GPS_VELOCITY_MPS,
		GPS_HPE_M,
		GPS_HEADING_DEG,
		GYRO_PITCH_RAD,
		GYRO_ROLL_RAD,
		MISSION_NUMBER,
		RUN_NUMBER,
		ACTUAL_THRUST_PERCENT,
		LAST_ABORT_CODE,
		LBL_BUOY_FLAGS,
		COORDINATE_N,
		COORDINATE_E,
		NUM_SENSOR_VARIABLES
	};
	static const char* sm_szMoosSensorVariableParams[];
	static const char* sm_szMoosSensorVarNames[];

	RadioModemSensorReport m_SensorReport;

};

#endif	// end #ifndef _IXSTREAMRADIOMODEM_H_
