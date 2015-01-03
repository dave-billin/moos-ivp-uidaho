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
/** @file iWhoiMicroModem.h

@brief
	Declaration of a MOOS instrument class used to implement communication 
	via a Woods Hole Oceanographic Institute (WHOI) Acoustic Micro Modem
	
@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the 
	University of Idaho, USA.
*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef _CWHOIMICROMODEM_H_
#define _CWHOIMICROMODEM_H_

#include <string>
#include <map>			// STL map class for associative array storage
#include "MOOS/libMOOS/MOOSLib.h"

#include "YellowSubUtils.h"
#include "WhoiCommandManager.h"		// Tx Manager worker thread



// Software version of iWhoiMicroModem
#define IWHOIMICROMODEM_VERSION 1.0


/** @def WHOI_PARAMCONFIG_TIMEOUT_SEC
@brief	
	Timeout (seconds) used when configuring the modem on startup

@details
	When iWhoiModem is launched, it first configures the modem
	with all of the NVRAM parameter values defined in the mission 
	file.  For each parameter value, a corresponding CCFG (set 
	modem parameter) command is sent to the modem.  iWhoiMicroModem 
	then waits up to WHOI_PARAMCONFIG_TIMEOUT_SEC seconds for the
	modem to echo the parameter back in a CACFG message, indicating
	it was received and applied.
*/
#define WHOI_PARAMCONFIG_TIMEOUT_SEC	2



class WhoiCommandManager;

//=====================================
// CONFIGURATION CONSTANTS

/** @def WHOI_DEBUG_VERBOSITY
@brief 
	Used to set the amount of debugging messages posted by this module.  Valid
	parameter values are:
		- 0: Debugging messages disabled
		- 1: Error conditions only
*/
#define WHOI_DEBUG_VERBOSITY	1





//=============================================================================
/** Utility class used to wrap a single set of sequential data Bytes received 
	over the streaming serial port */
class RxSerialDataPacket
{
public:
	//=========================================================================
	/** Creates an instance of the object
	@param RxDataBytes
		A pointer to the received data Bytes this object will be populated
		with

	@param NumDataBytes
		The number of Bytes pointed to by RxDataBytes
	*/
	RxSerialDataPacket( char* RxDataBytes, int NumDataBytes );

	/** Called when the object goes out of scope */
	~RxSerialDataPacket();

	std::string sSerialBytes;	/**< All received serial Bytes */
	double RxMsTime;		/**< Local ms count when the data was received */
};








//=============================================================================
/** An object encapsulating communication and control with a WHOI Acoustic 
	 Micro-Modem module connected via serial port.

 @internal
*/
//=============================================================================
class iWhoiMicroModem : public CMOOSInstrument
{
private:

	//=============================================================================
	// Internal class used to represent a serial data Listener
	class SerialRxListener
	{
	public:

		/** Creates the listener object
		@param sMessageKeys
			The message type keys being listened for (i.e. CAACK, CADRQ, etc.)
		@param vDestBufferRef
			Reference to a std::vector to be populated with fields of received message
		@param TimeStampRef
			Reference to a double to be populated with the message time stamp
		@param ListenerSemaphoreRef
			Reference to the semaphore object to be posted when the requested
			message is received
		@param ShouldBePersistent
			true if the listener wants to keep receiving notifications of the
			specified message; false to only listen for the message once
		*/
		SerialRxListener( std::string MessageTypeKeys,
						  std::vector<std::string>& vDestBufferRef,
						  double& TimeStampRef,
						  YellowSubUtils::Semaphore& ListenerSemaphoreRef )
			: sMessageKeys(MessageTypeKeys),
			  vDestFields(vDestBufferRef),
			  MessageTimeStamp(TimeStampRef),
			  SEM_Listener(&ListenerSemaphoreRef)
		{;}

		// Called when the object goes out of scope.  Does not change the state of 
		// SEM_Listener
		~SerialRxListener()	{;}

		//----------------------
		// Public Attributes
		//----------------------
		std::string sMessageKeys;

		// Reference to a std::vector to be populated with received message fields
		std::vector<std::string>& vDestFields;

		// Reference to a double to be populated with the message timestamp
		double& MessageTimeStamp;

		// Semaphore to be signaled when the requested message has arrived and
		// been copied into DestBuffer
		YellowSubUtils::Semaphore* SEM_Listener;

		//----------------------
		// Operators
		//----------------------
		SerialRxListener& operator= (const SerialRxListener& SourceObj)
		{
			if (this != &SourceObj)		// Handle self-assignment
			{
				sMessageKeys = SourceObj.sMessageKeys;
				vDestFields = SourceObj.vDestFields;
				MessageTimeStamp = SourceObj.MessageTimeStamp;
				SEM_Listener = SourceObj.SEM_Listener;
			}
			return *this;
		}
	};


public:

	//==========================================================================
	/** ID's corresponding to NMEA Identifier strings used by the modem
	@details
		Messages sent from modem to host computer contain "M2H" in their name.
		Messages sent from host computer to modem have a name containing "H2M".
	*/
	enum e_WhoiRxNmeaMessageTypeIds
	{
		// *** IMPORTANT: Don't modify the order of these declarations! ***

		// Messages sent from Modem to Host computer
		ID_M2H_CAACK = 0,	/**< Positive acknowledgement that a frame of data has
								 been received from another acoustic device */

		MSGID_M2H_CACFG,		/**< Modem reports the value of a configuration parameter */
		MSGID_M2H_CACLK,		/**< Modem reports the current time */
		MSGID_M2H_CACYC,		/**< Modem reports a synchronous acoustic network command */
		MSGID_M2H_CADBG,		/**< Low-level debugging messages */
		MSGID_M2H_CADOP,		/**< Modem reports relative Doppler (m/sec) */
		MSGID_M2H_CADQF,		/**< Modem reports decision quality factor for last FSK packet */
		MSGID_M2H_CADRQ,		/**< Modem requests (ASCII or Binary) data to be sent.  This may
								 occur when:
									- The modem has signalled another network device that it
									  is going to send data
									- Another network device has requested that this ID send
									  data to it */
		MSGID_M2H_CAERR,		/**< Modem is reporting an error */
		MSGID_M2H_CAMFD,		/**< Modem reports comms matched filter detector value */
		MSGID_M2H_CAMSA,		/**< Modem reports that a sleep command was received acoustically */
		MSGID_M2H_CAMSC,		/**< Echo of sleep command sent from host to modem */
		MSGID_M2H_CAMSG,		/**< Modem reports a system information message such as a timeout
								 when expecting a data reply */
		MSGID_M2H_CAMSR,		/**< An acoustic reply to a sleep command was received */
		MSGID_M2H_CAMUA,		/**< A mini-packet was received acoustically */
		MSGID_M2H_CAMUC,		/**< Echo of user mini-packet command */
		MSGID_M2H_CAMUR,		/**< Modem reports that the reply to a mini-packet was received */
		MSGID_M2H_CAREV,		/**< Modem reports application name and revision number */
		MSGID_M2H_CARXA,		/**< Modem reports received ASCII data frame */
		MSGID_M2H_CARXD,		/**< Modem reports received binary (hex) data frame */
		MSGID_M2H_CARXP,		/**< Modem reports the start of a (data?) packet */
		MSGID_M2H_CATXA,		/**< Modem confirms receipt of data in CCTXA to transmit */
		MSGID_M2H_CATXD,		/**< Modem confirms receipt of data in CCTXD to transmit */
		MSGID_M2H_CATXF,		/**< Modem reports end of packet transmission */
		MSGID_M2H_CATXP,		/**< Modem reports start of packet transmission */
		MSGID_M2H_SNMFD,		/**< Modem reports nav matched-filter detector value(s) */
		MSGID_M2H_CACST,		/**< Modem reports Cycle Rx statistics */
		MSGID_M2H_CAXST,		/**< Modem reports Cycle Tx statistics */
		MSGID_NUM_M2H_IDS,		/**< Used internally for bounds-checking.  Not a valid ID */

		MSGID_M2H_GPS_PASSTHRU	/**< GPS NMEA sentence from the modem's aux serial port.
									 These are treated separately from all other modem
									 to host messages. */

		// Messages sent from Host computer to modem
		//MSGID_H2M_CCCFG,		/**< Sent to set the value of a modem NVRAM parameter */
		//MSGID_H2M_CCCFQ,		/**< Sent to query the value of a modem NVRAM parameter */
		//MSGID_H2M_CCCLK,		/**< Sent to set the real-time clock time */
		//MSGID_H2M_CCCYC,		/**< Sent to initiate an acoustic communication */
		//MSGID_H2M_CCMEC,		/**< Sent to control external hardware I/O lines */
		//MSGID_H2M_CCMPC,		/**< Command to send an active navigation cycle (ping) */
		//MSGID_H2M_CCMSC,		/**< Command telling the modem to sleep */
		//MSGID_H2M_CCMUC,		/**< Command to send a user mini-packet */
		//MSGID_H2M_CCPDT,		/**< Command to ping a REMUS digital transponder */
		//MSGID_H2M_CCTXA,		/**< Command to send ASCII data */
		//MSGID_H2M_CCTXD,		/**< Command to send binary (hex-encoded) data */
		//MSGID_NUM_H2M_IDS		/**< Used internally for bounds-checking.  Not a valid ID */
	};


	//========================================
	// Object creation and destruction
	//========================================
	/** Creates an instance of the iWhoiMicroModem class	*/
	iWhoiMicroModem ();
	
	/** Destructor called when the object goes out of scope */
	virtual ~iWhoiMicroModem ();

	
	//========================================
	// Methods inherited from CMOOSApp
	//========================================
	//! @name Methods inherited from the CMOOSApp class
	//! @{
	
	//=========================================================================
	/** Called periodically to run the modem */
	bool Iterate();	
	
	//=========================================================================
	/** Called when new mail or notifications arrive from the MOOS database */
	bool OnNewMail(MOOSMSG_LIST &NewMail);
	
	//=========================================================================
	/** Called when a command message arrives from the MOOS database */
	bool OnCommandMsg(CMOOSMsg Msg);
	
	//=========================================================================
	/** Called to make a status string for this instrument */
	std::string MakeStatusString();
	
	//=========================================================================
	/** Called once when the application starts up just before the main 
		 MOOSApp execution loops is entered 
	*/
	bool OnStartUp();
	
	//=========================================================================
	/** Called when a connection is made to the MOOS database */
	bool OnConnectToServer();
	
	//=========================================================================
	/** Called when the connection to the MOOS database is lost or closed */
	bool OnDisconnectFromServer();
	//! @}
	
	
	
	//========================================
	// Methods inherited from CMOOSInstrument
	//========================================
	//! @name Methods inherited from the CMOOSInstrument class
	//! @{
	
	//=========================================================================
	/** Called from within OnStartUp() to perform device-specific initialization 
	*/
	bool InitialiseSensor();
	
	//=========================================================================
	/** Configures the serial port, m_Port, associated with this instrument 
		 using parameters read from the mission file
	*/
	bool SetupPort();


	//=========================================================================
	/** Registers the calling function as a listener to be notified when a 
		specified NMEA sentence is received from the modem

	@details
		When the specified message type string for a listener is received, the 
		message fields (including the messge type string) are tokenized and 
		copied into a specified string array, and a time stamp for the received 
		data is reported.  Following this, a semaphore registered for the 
		listener is Posted, allowing the listener to be notified that the 
		requested data has arrived.

		After calling this function, the caller can block on the semaphore it
		registers in order to wait for the specified message to be received.

	@param sMessageKey
		A string containing the message type key of the NMEA sentence to be 
		listened for (e.g. "CAACK", "CCTXP", etc.).  To listen for more than 
		one message, simply provide a comma-separated list of message type key
		strings.  The first of these strings received will be posted to the
		listener

	@param OUT_DestBuffer
		A std::vector of strings to be populated with the comma-delimited fields of
		the message when it is received

	@param OUT_TimeStamp
		A double to be populated with a time stamp reporting when the message
		was received

	@param RxSemaphore
		A semaphore to be posted when the specified message has arrived and
		been placed into sOUT_DestBuffer
	*/
	void AddRxListener( const std::string& sMessageKey,
						std::vector<std::string>& OUT_DestBuffer,
						double& OUT_TimeStamp,
						YellowSubUtils::Semaphore& RxSemaphore );


	//============================================================================
	/** A function is used to assign the priorities of transmitted message types 
		during runtime */
	static void SetMessagePriority( std::string sMessageID, unsigned int Priority );


	//============================================================================
	//! Registers MOOS variables published and subscribed to by iWhoiMicroModem
	void RegisterMoosVariables( void );


	//-------------------------------------------------------------------------
	// Allow the Command manager access to the MOOS Instrument's serial port 
	// and CommsClient objects
	friend class WhoiCommandManager;
	//-------------------------------------------------------------------------

	// Make the app a singleton
	//MOOS_DeclareSingleton_SingleThreaded_Minimal(iWhoiMicroModem);

protected:

	bool m_ModemIsConnected;	/** Initially set to false; set to true once 
									a valid NMEA sentence is received from
									the modem */

	//=========================================
	/*	Inherited attributes from CMOOSApp
	//=========================================
	CMOOSCommClient m_Comms;	// Connection to the MOOS database
	CProcessConfigReader m_MissionReader; // Used to read mission files
	MOOSVARMAP m_MOOSVars;	// MOOS variables this app subscribes to
	bool m_bSimMode;			// true if the app is running with a simulator
	long m_lServerPort;		// Port used to listen for new connections
	std::string m_sServerHost;	// Host name of the MOOS database server
	std::string m_sServerPort;	// String version of m_lServerPort
	bool m_bServerSet;			// true if the server has been set
	bool m_bUseMOOSComms;		// true to enable MOOS communications
	std::string m_sAppName;		// The name of this application
	int m_nCommsFreq;		// Frequency (Hz) of MOOS variable updates
	double m_dfFreq		// Frequency (Hz) at which this app will iterate
	std::string m_sMissionFile			// Name of the mission file in use
	bool m_bCommandMessageFiltering	// true to enable separate command messages
	double m_dfAppStartTime		// MOOS time when this app was started
	double 	m_dfLastRunTime	// MOOS time when the last iteration ran
	bool 	m_bDebug					// (???) Internal use only
	*/
	
	//============================================
	/*	Inherited attributes from CMOOSInstrument
	//============================================
	// True if raw incoming data should be published to the MOOS database
	bool CMOOSInstrument::m_bPublishRaw	
 	
	// A platform-independent serial port connected to the instrument
	CMOOSLinuxSerialPort CMOOSInstrument::m_Port

	// Place holder for the name of this sensor resource (rarely used)
	std::string CMOOSInstrument::m_sResourceName
 	*/

	//============================================================================
	//! A wrapper function for m_Comms.Notify() to ensures thread-safety
	bool NotifyThreadSafe( const std::string &sVar,
	                       const std::string& sVal, double dfTime = -1);
	YellowSubUtils::TimedLock m_NotifyLock;

	//============================================================================
	/** Converts a serial message ID string to a corresponding message ID from
		e_WhoiRxNmeaMessageTypeIds
	
	@param sMessageTypeKey
		A string containing a message identifier found at the beginning of a NMEA
		sentence sent/received by the modem

	@return
		A value from e_WhoiRxNmeaMessageTypeIds corresponding to sMessageTypeKey
		or -1 if the key was not found
	*/
	const int MessageTypeStringToMessageId( const std::string& sMessageTypeKey );


	//============================================================================
	/** Converts a message type ID from e_WhoiRxNmeaMessageTypeIds to its 
		corresponding serial message ID string
	
	@param MessageId
		A message type ID from e_WhoiRxNmeaMessageTypeIds

	@return
		A string containing the NMEA message identifier string corresponding to
		the value of MessageTypeId (i.e. "CARXD", "CADQF", etc.)
	*/
	const std::string MessageIdToMessageTypeString( int MessageId );



	//============================================================================
	/** Returns a string containing a NMEA checksum calculated as the bitwise 
		exclusive-or of all characters in a specified string

	@param sNmeaSentence
		Characters preceding the checksum field of a NMEA sentence.  NOTE: if a
		leading '$' character is included, it will be skipped when calculating
		the checksum.

	@return
		The NMEA checksum for the message (not including the '*') for the field
	*/
	std::string CreateNmeaChecksum( std::string& sNmeaSentence );



	//============================================================================
	/** Assembles a NMEA sentence from a std::vector of strings and sends it to the
		modem over the application's serial port using a thread-safe call.

	@details
		This function first assembles a NMEA sentence from a std::vector of string
		data corresponding to the comma-delimited fields of the corresponding NMEA
		sentence to be sent to the modem.  It then calculates and appends the  
		checksum and terminating Bytes for the NMEA sentence, and sends the data 
		to the modem.

	@param vNmeaFields
		Reference to a std::vector of strings containing command/data fields
		corresponding to the comma-delimited fields (not including checksum) of 
		the NMEA sentence to be sent to the modem.

	@return
		true on success; false if vNmeaFields is empty or if not all data was
		successfully sent to the serial port
	*/
	bool SendNmeaVectorToModem( std::vector<std::string>& vNmeaFields );



	//============================================================================
	/** Thread-safe function to send a string of data to the modem
	
	@param sTxData
		String containing Bytes to be sent to the modem
	
	@return
		true if the data was sent successfully; else false
	*/
	bool SendToModem( const std::string& sTxData );

private:
	bool m_ShouldPrintModemTx;	/**< true to print NMEA strings sent to the modem */
	bool m_ShouldPrintModemRx;	/**< true to print NMEA strings received from the modem */
	bool m_EnablePromiscuousDataRx;	/**< true to publish received ASCII and binary data 
										 sent ANY acoustic ID */
	bool m_EnablePromiscuousMiniPacketRx;	/**< true to publish received user mini 
												 packets sent to ANY acoustic ID */

	//! Modem NVRAM parameters indexed by their 3-letter ID strings
	std::map <const std::string, int> m_NvramParamValues;

	/** Modem message type ID's from e_WhoiRxNmeaMessageTypeIds indexed by their
		corresponding NMEA message ID strings */
	std::map <const std::string, int> m_MessageTypeIdTable;

	/** Modem NMEA message ID strings indexed by their corresponding value in
		e_WhoiRxNmeaMessageTypeIds */
	std::map <int, const std::string> m_MessageIdStringTable;

	//! Modem command type ID's indexed by their corresponding command keys
	std::map <const std::string, int> m_ModemCommandIdTable;

	//============================================================================
	/** Lookup table used regulate the priority of the various commands that may
		be sent to the modem. */
	std::vector<int> m_vCommandPriorityTable;
	YellowSubUtils::TimedLock m_CommandPriorityTableLock;

	/** A std::vector of listener objects used to notify threads waiting for a
		particular NMEA sentence to be received from the modem */
	std::vector <SerialRxListener> m_RxListeners;
	YellowSubUtils::TimedLock m_RxListenerLock;	// Used to sync listener std::vector
	
	/** Worker threads to carry out sending serial messages and hanshaking with the 
		modem */
	WhoiCommandManager* m_CommandMgr;


	// Prevent automatic generation of copy constructor and assignment operator 
	iWhoiMicroModem (const iWhoiMicroModem&);
    const iWhoiMicroModem& operator= (const iWhoiMicroModem&);
	
	const std::string EmptyString;

	YellowSubUtils::TimedLock m_ModemTxLock;

	//=========================================================================
	/** A function to read a block of data received on the serial port

	@param OUT_vRxNmeaFields
		A std::vector of strings containing the data fields of a received NMEA
		sentence

	@param OUT_TimeStamp
		A double to be populated with a time stamp indicating when the message
		was received

	@return
		true if a received message was returned in OUT_sRxData; else false if
		a complete message (ending in the assigned terminator character has
		not been received.
	*/
	bool GetRxData( std::vector<std::string>& OUT_vRxNmeaFields, double& OUT_TimeStamp );



	//=========================================================================
	/** Helper function used to dispatch data received on the serial port to
		an appropriate handler function 
	
	@param vNmeaFields
		A std::vector of strings containing the data fields of a receive NMEA
		sentence

	@param RxTimeStamp
		A time stamp indicating when the message was received

	@return
		true if the message was handled; else false if the message was invalid
		or an error occurred
	*/
	bool DispatchRxMessage( std::vector<std::string>& vNmeaFields, double RxTimeStamp = -1);



	//=========================================================================
	//! Helper function to load settings from the configuration file
	bool LoadConfiguration( void );


	//=========================================================================
	/** This function handles serial messages received from the modem that are
		used internally by iWhoiMicroModem

	@param MessageTypeId
		Integer type of the message

	@param vMessageFields
		A std::vector of strings containing the comma-delimited fields of the
		received message

	@return
		true if the message should be passed on to listeners; else false to
		prevent listeners from seeing the message
	*/
	bool InternalRxHandlerDispatch( int MessageTypeId, std::vector<std::string>& vMessageFields );


	//=========================================================================
	/* Helper function used to wait for messages during startup initialization
	@param sMessageKey
		The string ID of the message to wait for (not including the leading 
		'$' character)
	@param TimeoutSec
		Maximum time (seconds) to wait for the messages
	@param OUT_vMessageFields
		Vector to be populated with received message fields
	@return
		true if we got the messages; false if the timeout expired
	*/
	bool WaitForRxMessageDuringInit( std::string sMessageKey, int TimeoutSec,
									 std::vector<std::string>& OUT_vMessageFields );

	//=========================================================================
	// Internal handlers for received serial messages
	//=========================================================================
	bool CACFG_Handler( std::vector<std::string>& vMessageTokens );	// NVRAM parameter value report
	bool CACLK_Handler( std::vector<std::string>& vMessageTokens );	// RTC time report
	bool CACYC_Handler( std::vector<std::string>& vMessageTokens );	// Acoustic comm init echoed/received
	bool CADBG_Handler( std::vector<std::string>& vMessageTokens );	// Low-level debug messages
	bool CADOP_Handler( std::vector<std::string>& vMessageTokens );	// Relative Doppler
	bool CADQF_Handler( std::vector<std::string>& vMessageTokens );	// Data quality factor for last FSK packet
	bool CAERR_Handler( std::vector<std::string>& vMessageTokens );	// Error message
	bool CAMFD_Handler( std::vector<std::string>& vMessageTokens );	// Matched filter detector info for comms
	bool CAMSG_Handler( std::vector<std::string>& vMessageTokens );	// Bad CRC and data packet timeout report
	bool CAMUA_Handler( std::vector<std::string>& vMessageTokens );	// User mini-packet received acoustically
	bool CAREV_Handler( std::vector<std::string>& vMessageTokens );	// Modem firmware revision info
	bool CARXA_Handler( std::vector<std::string>& vMessageTokens );	// Received ASCII data frame
	bool CARXD_Handler( std::vector<std::string>& vMessageTokens );	// Received (hex-coded) binary data frame
	bool CARXP_Handler( std::vector<std::string>& vMessageTokens );	// Incoming packet detected
	bool SNMFD_Handler( std::vector<std::string>& vMessageTokens );	// Matched-filter detector info for nav pings
	bool CACST_Handler( std::vector<std::string>& vMessageTokens );	// Comm Rx Statistics message
	bool CAXST_Handler( std::vector<std::string>& vMessageTokens );	// Comm Tx Statistics message
	bool GPS_PassThru_Handler( std::vector<std::string>& vNmeaFields ); // GPS data from aux serial port

	/* Received messages from the modem that must be implemented:
	CCCYC	- Received when somebody else sends a cycle-init or when we just sent a CCCYC ourselves
	CACYC	- Received when the vehicle we specified with a CCCYC sends an acknowledgement
	CADRQ	- Received when the modem is ready to send a 32-Byte (binary or ASCII) data packet
	CARXA	- Received when a 32-Byte ASCII data packet is received
	CARXD	- Received when a 32-Byte binary data packet is received
	CAMSG	- Received when the modem gets a 32-Byte data packet with a bad CRC
			    - or -
			  no data packet is received by the modem within 3 seconds of a CADRQ

	SNTTA	- Received when the replies to a transponder PING come in
	*/
};



#endif	// END #ifdef _CWHOIMICROMODEM_H_
