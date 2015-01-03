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
/** @file iWhoiMicroModem.cpp

@brief
	Implementation of the iWhoiMicroModem object

@author Dave Billin

*/
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include <string.h>
#include <vector>
#include <time.h>
#include "iWhoiMicroModem.h"

using namespace::std;
using YellowSubUtils::TimedLock;
using YellowSubUtils::Semaphore;

#ifndef WHOI_DEBUG_VERBOSITY
	#define WHOI_DEBUG_VERBOSITY 0
#endif

#if WHOI_DEBUG_VERBOSITY & 1
	#define WHOI_VERBOSELVL1( _expr_ )	_expr_
#else
	#define WHOI_VERBOSELVL1( _expr_ )
#endif





/** A helper function to tokenize a std::string into a vector of strings
@details
	This has been modified to return empty strings for consecutive
	delimiters.  This is needed for some received messages which denote
	empty fields with consecutive commas (SNTTA is one example)

@param sSource
	The string to be tokenized

@param sTokens
	The vector to be populated with tokens

@sDelimiters
	One or more delimiter characters to tokenize around

@return
	true if at least one delimiter was found in sTokens
*/
static bool VectorizeString( const string& sSource, vector<string>& vTokens,
							const string& sDelimiters = ",")
{
    // Skip sDelimiters at beginning.
    string::size_type lastPos = sSource.find_first_not_of(sDelimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos     = sSource.find_first_of(sDelimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        vTokens.push_back(sSource.substr(lastPos, pos - lastPos));

        // Skip sDelimiters.  Note the "not_of"
        lastPos = sSource.find_first_not_of(sDelimiters, pos);

		// For each delimiter between pos and the next token
		// add an empty string
		if ( (pos != string::npos) && (lastPos != string::npos) )
		{
			for ( unsigned int i = (pos + 1); (i < lastPos) && (i < sSource.length()); i++ )
			{
				if ( sDelimiters.find(sSource[i]) != string::npos )
				{
					vTokens.push_back( string() );
				}
			}
		}
		// Find next "non-delimiter"
        pos = sSource.find_first_of(sDelimiters, lastPos);
    }

	return !vTokens.empty();
}









//=============================================================================
/*	Creates an instance of the iWhoiMicroModem class	*/
iWhoiMicroModem::iWhoiMicroModem()
: m_ModemIsConnected(false),
  m_NotifyLock(false),
  m_ShouldPrintModemTx(false),
  m_ShouldPrintModemRx(false),
  m_EnablePromiscuousDataRx(false),
  m_EnablePromiscuousMiniPacketRx(false),
  m_CommandPriorityTableLock(false),
  m_RxListenerLock(false),
  m_CommandMgr(NULL),
  m_ModemTxLock(false)
{
	// *** DEBUG
	//vector<string> vTest;
	//VectorizeString("a,,,bc,d",vTest, ",");
	//vTest.clear();
	//VectorizeString("a,b,c,d",vTest, ",");

	// Enable <PROCESSNAME>_CMD message filtering
	EnableCommandMessageFiltering(false);

	//----------------------------------------------------
	// Initialize the priority table for modem commands
	m_vCommandPriorityTable.resize(WhoiCommandManager::NUM_COMMAND_TYPE_IDS, 0);


	//----------------------------------------------------
	// Initialize MessageID<-->IDString conversion tables
	string sKey;
	string sMessageIDs =	"CAACK,CACFG,CACLK,CACYC,CADBG,CADOP,CADQF,CADRQ,"
							"CAERR,CAMFD,CAMSA,CAMSC,CAMSG,CAMSR,CAMUA,CAMUC,"
							"CAMUR,CAREV,CARXA,CARXD,CARXP,CATXA,CATXD,CATXF,"
							"CATXP,SNMFD,CACST,CAXST";

	for (int i = 0; i < MSGID_NUM_M2H_IDS; i++)
	{
		sKey = MOOSChomp(sMessageIDs, ",", true);
		m_MessageTypeIdTable.insert( make_pair(sKey, i) );
		m_MessageIdStringTable.insert( make_pair(i, sKey) );
	}

	//----------------------------------------------------
	// Initialize Modem command ID table
	// The order of the keys *must* correspond to the order of elements in
	// e_CommandTypeIds
	string sCommandKeys =	"TxUserMiniPacket,WriteASCIIData,ReadASCIIData,"
							"WriteBinaryData,ReadBinaryData,TxMiniPacketPing,"
							"TxFmSweepPing,TxRemusPing,TxNarrowBandPing,"
							"SetRealtimeClock,SetNvramParam,GetNvramParam,"
							"SetIOLineState,ModemSleep,AutoLevelAgc"
							",MeasureNoiseLevel";

	for (int i = 0; i < MSGID_NUM_M2H_IDS; i++)
	{
		sKey = MOOSChomp(sCommandKeys, ",", true);
		m_ModemCommandIdTable.insert( make_pair(sKey, i) );
	}


	//------------------------------------------------------
	// Set initial acoustic ID.  Working ID will be loaded from
	// the mission file later.
	unsigned int AcousticId = 0;

	// Create the Tx manager but do not start it yet.
	// We'll start it in OnStartup() after all configuration is loaded
	m_CommandMgr = new WhoiCommandManager( *this, AcousticId );
}




//=============================================================================
/*	Destructor called when the object goes out of scope */
iWhoiMicroModem::~iWhoiMicroModem()
{
	string sDummy;

	// Cancel any pending commands in the queue
	//m_CommandMgr->PostCommand( WhoiCommandManager::ClearCommandQueue,
	//						   sDummy, sDummy,
	//						   WhoiCommandManager::ClearCommandQueue );

	// Wait for Command Manager thread to exit
	m_CommandMgr->Stop();
}






//=============================================================================
/*	Called periodically to run the modem */
bool iWhoiMicroModem::Iterate()
{
	static vector<string> vRxData;
	static double RxTimeStamp;

	//---------------------------------------------------
	// Receive and parse data from the serial port

	// NOTE: if the serial port is not in streaming mode, GetRxData will
	// block for a minimum of 10 ms.
	vRxData.clear();
	while ( GetRxData(vRxData, RxTimeStamp) )
	{
		DispatchRxMessage(vRxData, RxTimeStamp);
	}

	return true;
}



//=============================================================================
/*	Called when new mail or notifications arrive from the MOOS database */
bool iWhoiMicroModem::OnNewMail(MOOSMSG_LIST &NewMail)
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
				//WHOI_VERBOSELVL1( MOOSTrace( "Identified skewed message: %s=\"%s\"\n",
				//		   RxMsg.m_sKey.c_str(),
				//		   RxMsg.GetAsString().c_str() ) );
				continue;
			}

			// Identify command messages and process them in the command parser
			if ( MOOSStrCmp(RxMsg.GetKey(), GetAppName() + "_CMD") )
			{
				OnCommandMsg(RxMsg);
			}

			// Handle Modem Tx enablement
			else if ( MOOSStrCmp(RxMsg.GetKey(), GetAppName() + "_TxEnabled") )
			{
				if ( RxMsg.IsString() )
				{
					bool TxEnablement;
					if ( MOOSValFromString( TxEnablement, RxMsg.GetString(), "Enabled", true) )
					{
						m_CommandMgr->SetTxCommandEnablement(TxEnablement);
					}
					else
					{
						MOOSTrace(GetAppName() + "_TxEnabled: missing field: \"Enabled\".  Message ignored.\n");
					}
				}
				else
				{
					MOOSTrace(GetAppName() + "_TxEnabled: value must be a string in the form "
							  "\"Enabled=[true/false]\". Message ignored.\n");
				}
			}
			else
			{
				MOOSTrace("Unrecognized mail: [%s] %s\n", RxMsg.GetKey().c_str(), RxMsg.GetAsString().c_str() );
			}
		}
	}


	// If we're not running, we're simulating...
	else
	{

	}

	return true;
}




//=============================================================================
/*	Called when a command message arrives from the MOOS database */
bool iWhoiMicroModem::OnCommandMsg(CMOOSMsg Msg)
{
	string sError;

	if ( !Msg.IsString() )
	{
		MOOSTrace("%s: message must be of type string\n", Msg.GetKey().c_str() );
	}
	else
	{
		string sMessage = Msg.GetString();

		// Enqueue modem commands received from the MOOSDB
		// Parameters will be validated by the worker thread
		string sCommandKey;
		if ( !MOOSValFromString( sCommandKey, sMessage, "Cmd", true) )
		{
			sError = "*** Invalid Command Message received from " + Msg.GetSource() + " ***\n"
					 "Commands must have the format:\n\t\"Cmd=[key],param1=value1,"
					 "param2=value2...paramN=valueN\".\n";
		}
		else
		{
			int CommandTypeId;

			// Convert the command type to its corresponding integer ID in
			map <const string, int>::iterator iter = m_ModemCommandIdTable.find(sCommandKey);
			if ( iter == m_ModemCommandIdTable.end() )
			{
				sError = "*ERROR* Unrecognized modem command: " + sCommandKey + "\n";
			}
			else
			{
				CommandTypeId = m_ModemCommandIdTable[sCommandKey];

				// Detect unsupported commands
				switch (CommandTypeId)
				{
				case WhoiCommandManager::SendUserMiniPacket_CCMUC:
				case WhoiCommandManager::WriteBinaryData_CCTXD:
				case WhoiCommandManager::SendRemusPing_CCPDT:
					m_CommandMgr->PostCommand(CommandTypeId, sMessage, m_vCommandPriorityTable[CommandTypeId] );
					break;

				default:
					MOOSTrace("** The %s command is not currently implemented **\n", sCommandKey.c_str() );
					break;
				}
			}
		}
	}

	if ( !sError.empty() )
	{
		MOOSTrace(sError);
	}
	return false;
}




//=============================================================================
/*	Called to make a status string for this instrument */
std::string iWhoiMicroModem::MakeStatusString()
{
	// For now, just return the default status string
	string sStatus = CMOOSApp::MakeStatusString();
	return sStatus;
}




//=============================================================================
/*	Called once when the application starts up just before the main
	MOOSApp execution loops is entered
*/
bool iWhoiMicroModem::OnStartUp()
{
	const char szBar[] = "===============================";
	bool Success = true;
	string sConfigFilePath;

	// Print startup banner
	printf("%s\n"
		   " iWhoiMicroModem v%3.2f\n"
		   "%s\n\n\n",
		   szBar, IWHOIMICROMODEM_VERSION, szBar);

	// Configure the serial port with settings in the mission file
	Success &= SetupPort();

	// Do Initialization and load settings from mission file
	if (Success)
	{
		Success &= InitialiseSensor();
	}

	// Start the command manager worker thread
	m_CommandMgr->Start();
	return Success;
}



//=============================================================================
/*	Called when a connection is made to the MOOS database */
bool iWhoiMicroModem::OnConnectToServer()
{
	bool Success = true;

	// *** Register for the variables and commands we will listen to ***
	RegisterMoosVariables();

	return Success;
}




//=============================================================================
/*	Called when the connection to the MOOS database is lost or closed */
bool iWhoiMicroModem::OnDisconnectFromServer()
{
	return true;
}



//=============================================================================
/*	Called from within OnStartUp() to perform device-specific initialization
	NOTE: this should be called AFTER SetupPort()
*/
bool iWhoiMicroModem::InitialiseSensor()
{
	vector <string> vMessageFields(3);
	vector<string> vRxNmeaFields;
	string sLine(25, '-');

	if (!LoadConfiguration())
	{
		MOOSFail("Failed to load configuration from mission file!\n");
	}

	MOOSTrace("\n\n" + sLine + "\n* Configuring the Modem *\n" + sLine + "\n");
	NotifyThreadSafe( GetAppName() + "_HardwareState", "Initializing" );

	// Send all NVRAM parameter values loaded from the mission file to the modem
	map <const string, int>::iterator iter;
	vMessageFields[0] = string("CCCFG");
	for (iter = m_NvramParamValues.begin(); iter != m_NvramParamValues.end(); iter++)
	{
		vMessageFields[1] = iter->first;
		vMessageFields[2] = MOOSFormat("%d", iter->second);
		if ( !SendNmeaVectorToModem( vMessageFields ) )
		{
			MOOSFail("\n\nCRITICAL ERROR: Failed to send data to the modem!\n\n");
		}

		vRxNmeaFields.clear();
		if ( !WaitForRxMessageDuringInit("CACFG", 2, vRxNmeaFields) )
		{
			// Retry parameter setting after 3 sec
			SendNmeaVectorToModem( vMessageFields );
			if ( !WaitForRxMessageDuringInit("CACFG", 2, vRxNmeaFields) )
			{
				MOOSTrace("\nWarning: modem failed to acknowledge parameter "
						  "%s.\n", iter->first.c_str() );
			}
		}
		else
		{
			if (vMessageFields[1] != iter->first)
			{
				MOOSTrace("\nWarning: failed to configure parameter "
						  "%s.\n", iter->first.c_str() );
			}
			else
			{
				MOOSTrace(".");
			}
		}
	}

	// Enable reporting of packet transmission start and end (CATXP and CATXF messages)
	vMessageFields[1] = "TXP";
	vMessageFields[2] = "1";
	SendNmeaVectorToModem(vMessageFields);
	WaitForRxMessageDuringInit("CACFG", 3, vRxNmeaFields);

	vMessageFields[1] = "TXF";
	vMessageFields[2] = "1";
	SendNmeaVectorToModem(vMessageFields);
	WaitForRxMessageDuringInit("CACFG", 3, vRxNmeaFields);

	bool ShouldSetModemClock = true;

	// Don't set modem's clock if it's supposed to be set from a GPS
	if ( m_NvramParamValues.find("SCG") != m_NvramParamValues.end() )
	{
		if (m_NvramParamValues["SCG"] != 0)
		{
			ShouldSetModemClock = false;
			MOOSTrace("\n\nModem's clock will be set from GPS\n");
		}
	}

	if (ShouldSetModemClock)
	{
		MOOSTrace("\n\nSetting the modem's clock\n");

		// Set the modem's clock
		time_t UnixTime;
		UnixTime = (time_t)MOOSTime();
		struct tm* LocalTime = localtime(&UnixTime);

		vMessageFields.clear();
		vMessageFields.push_back("CCCLK");
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_year + 1900) );
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_mon) );
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_mday) );
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_hour) );
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_min) );
		vMessageFields.push_back( MOOSFormat("%d",LocalTime->tm_sec) );

		SendNmeaVectorToModem( vMessageFields );
		if ( !WaitForRxMessageDuringInit("CACLK", 3, vMessageFields) )
		{
			MOOSTrace("WARNING: Failed to set the modem clock!\n");
		}
	}


	sLine.clear();
	sLine =  string(60, '=');
	MOOSTrace( "\n\n\n" + sLine + "\n" );
	MOOSTrace( MOOSFormat( "Acoustic Network ID:  %d\n\n",
						   m_CommandMgr->GetAcousticNetworkId()) );

	if (m_EnablePromiscuousDataRx)
	{
		MOOSTrace(" * Promiscuous ASCII/Binary data reporting is ENABLED\n");
	}

	if (m_EnablePromiscuousMiniPacketRx)
	{
		MOOSTrace(" * Promiscuous Mini-Packet reporting is ENABLED\n");
	}

	MOOSTrace(sLine + "\n\nListening for modem heartbeat...\n\n\n");
	return true;
}




//=============================================================================
/*	Configures the serial port, m_Port, associated with this instrument
	using parameters read from the mission file
*/
bool iWhoiMicroModem::SetupPort()
{
	bool OperationSuccessful;

	// Call base class method to configure the serial port
	// using settings from the mission file
	OperationSuccessful = CMOOSInstrument::SetupPort();

	// Set the terminating character for NMEA sentences
	m_Port.SetTermCharacter('\n');

	// Any further settings can be made here...
	return OperationSuccessful;
}




//============================================================================
//! Registers MOOS variables published and subscribed to by iWhoiMicroModem
void iWhoiMicroModem::RegisterMoosVariables( void )
{
	// --- Register to receive modem commands via IWHOIMICROMODEM_CMD variable ---
	//string sAppName = this->GetAppName();
	//std::transform(sAppName.begin(), sAppName.end(), sAppName.begin(), toupper);
	//m_Comms.Register( sAppName + "_CMD", 0 );

	// --- Register to modem command variable ---
	string sCommandCode = GetAppName() + "_CMD";
	m_Comms.Register( sCommandCode, 0 );

	// Indicates the state of the WHOI modem module as a string
	//	Disconnected - The modem is not responding
	//	Connected - The modem is responding to serial commands
	NotifyThreadSafe( GetAppName() + "_HardwareState",
					(m_ModemIsConnected) ? "Connected" : "Disconnected",
					MOOSTime() );
}



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
	A vector of strings to be populated with the comma-delimited fields of
	the message when it is received

@param OUT_TimeStamp
	A double to be populated with a time stamp reporting when the message
	was received

@param RxSemaphore
	A semaphore to be posted when the specified message has arrived and
	been placed into sOUT_DestBuffer
*/
void iWhoiMicroModem::AddRxListener( const string& sMessageKey,
									 vector<string>& OUT_DestBuffer,
									 double& OUT_TimeStamp,
									 Semaphore& RxSemaphore )
{
	// Validate parameters
	if ( sMessageKey.empty() )
	{
		return;
	}

	// Add a Serial Rx listener to the listen queue
	SerialRxListener NewListener( sMessageKey, OUT_DestBuffer, OUT_TimeStamp, RxSemaphore );
	m_RxListenerLock.Lock();
	m_RxListeners.push_back(NewListener);
	m_RxListenerLock.UnLock();
}




//=========================================================================
/** A function to read a block of data received on the serial port

@param OUT_vRxNmeaFields
	A vector of strings from the fields of a received NMEA sentence

@param OUT_TimeStamp
	The time stamp of the received message

@return
	true if a received message was returned in OUT_sRxData; else false if
	a complete message (ending in the assigned terminator character has
	not been received.
*/
bool iWhoiMicroModem::GetRxData( vector<string>& OUT_vRxNmeaFields, double& OUT_TimeStamp )
{
	bool Rc = false;
	string sRxData;

	if ( m_Port.IsStreaming() )
	{
		// Return accumulated messages from a streaming port in the order
		// they arrived
		Rc = m_Port.GetEarliest(sRxData, OUT_TimeStamp);
	}
	else
	{
		Rc = m_Port.GetTelegramOrAccumulate(sRxData, 0, &OUT_TimeStamp);
	}

	// If we received data
	if ( !sRxData.empty() )
	{
		// If PRINT_MODEM_RX = true in mission file, print received NMEA sentences to stdio
		if (m_ShouldPrintModemRx)
		{
			MOOSTrace( "[WHOI Rx]: " + sRxData + "\n" );
		}

		// Received NMEA sentences must start with '$'
		// The terminating \r\n is stripped by CMOOSSerialPort's
		// telegram implementation
		if ( sRxData[0] != '$' )
		{
			WHOI_VERBOSELVL1( MOOSTrace("[iWhoiMicroModem::DispatchRxMessage] "
										"Invalid NMEA sentence: missing '$'\n" ) );
			return false;
		}

		// DoNMEACheckSum will modify contents of its argument, so we make a copy
		if ( !CMOOSInstrument::DoNMEACheckSum(string(sRxData)) )
		{
			WHOI_VERBOSELVL1( MOOSTrace("[iWhoiMicroModem::GetRxData] NMEA sentence checksum failed!") );
			return false;
		}

		// Tokenize NMEA sentence characters following the '$'
		OUT_vRxNmeaFields.clear();
		MOOSAssert( VectorizeString(sRxData.erase(0,1), OUT_vRxNmeaFields, ",*\r\n"),
					(char*)"Failed to vectorize received serial data!\n" );

		// Strip the checksum field
		string::size_type AsteriskIndex = sRxData.find('*', 0);
		if ( AsteriskIndex != string::npos )
		{
			OUT_vRxNmeaFields.pop_back();
		}
	}

	return Rc;
}



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
bool iWhoiMicroModem::WaitForRxMessageDuringInit( string sMessageKey, int TimeoutSec,
												  vector<string>& OUT_vMessageFields )
{
	double RxTimeStamp, TimeRef, ElapsedSec;

	TimeRef = MOOSTime();
	do
	{
		MOOSPause(5);	// Poll for received message every 5 ms

		// If we received a reply, check it against the parameter we sent
		if ( GetRxData(OUT_vMessageFields, RxTimeStamp) )
		{
			if ( MOOSStrCmp(OUT_vMessageFields[0], sMessageKey) )
			{
				return true;
			}
		}

		ElapsedSec = MOOSTime() - TimeRef;
	}
	while (ElapsedSec < (double)TimeoutSec);

	return false;
}






//=========================================================================
/** Helper function used to dispatch data received on the serial port to
	an appropriate handler function

@param vNmeaFields
	A vector of strings containing the data fields of a receive NMEA
	sentence

@param RxTimeStamp
	A time stamp indicating when the message was received

@return
	true if the message was handled; else false if the message was invalid
	or an error occurred
*/
bool iWhoiMicroModem::DispatchRxMessage( vector<string>& vNmeaFields, double RxTimeStamp )
{
	int MessageTypeID;

	if (vNmeaFields.empty())	// Validate vector param
		return false;

	// Extract the Message Type ID and remove the message id from the received data
	string MessageTypeKey = vNmeaFields[0];
	MessageTypeID = MessageTypeStringToMessageId( MessageTypeKey );

	// Handle received GPS pass-through messages
	if (MessageTypeID == MSGID_M2H_GPS_PASSTHRU)
	{
		return GPS_PassThru_Handler(vNmeaFields);
	}

	// Handle received messages internally
	bool PassToListeners = InternalRxHandlerDispatch(MessageTypeID, vNmeaFields);

	if (PassToListeners)
	{
		// Put the message key back on the front of the vector
		vNmeaFields.insert( vNmeaFields.begin(), MessageTypeKey );

		m_RxListenerLock.Lock();	// Lock the listener list while we search it

		// Notify all listeners for this message type
		vector<SerialRxListener>::iterator TheListener = m_RxListeners.begin();
		while ( TheListener != m_RxListeners.end() )
		{
			if ( TheListener->sMessageKeys.find(MessageTypeKey) != string::npos )
			{
				TheListener->vDestFields = vNmeaFields;
				TheListener->MessageTimeStamp = RxTimeStamp;
				if ( TheListener->SEM_Listener != NULL)
				{
					TheListener->SEM_Listener->Post();
				}

				// Remove matching listeners from the list
				TheListener = m_RxListeners.erase(TheListener);
			}
			else
			{
				TheListener++;
			}
		}

		m_RxListenerLock.UnLock();	// Unlock the listener list
	}

	return true;
}

//============================================================================
// A wrapper function for m_Comms.Notify() to ensures thread-safety
bool iWhoiMicroModem::NotifyThreadSafe(const string &sVar, const string& sVal, double dfTime)
{
	bool Rc;

	m_NotifyLock.Lock(TimedLock::TIMEOUT_INFINITE);
	Rc = m_Comms.Notify(sVar, sVal, dfTime);
	m_NotifyLock.UnLock();

	return Rc;
}



//============================================================================
/* Converts a serial message ID string to a corresponding message ID from
	e_WhoiRxNmeaMessageTypeIds

@param sMessageTypeKey
	A string containing a message identifier found at the beginning of a NMEA
	sentence sent/received by the modem

@return
	A value from e_WhoiRxNmeaMessageTypeIds corresponding to sMessageTypeKey
	or -1 if the key was not found
*/
const int iWhoiMicroModem::MessageTypeStringToMessageId( const string& sMessageTypeKey )
{
	// DB: Identify GPS pass-through sentences by keys beginning with "GP" or "PG"
	string sTemp = sMessageTypeKey.substr(0,2);
	if ( MOOSStrCmp(sTemp, "GP") || MOOSStrCmp(sTemp, "PG") )
	{
		return MSGID_M2H_GPS_PASSTHRU;
	}
	else
	{
		map<const string, int>::iterator iter = m_MessageTypeIdTable.find(sMessageTypeKey);
		return ( iter != m_MessageTypeIdTable.end() ) ? iter->second : -1;
	}
}


//============================================================================
/* Converts a message type ID from e_WhoiRxNmeaMessageTypeIds to its
	corresponding serial message ID string

@param MessageId
	A message type ID from e_WhoiRxNmeaMessageTypeIds

@return
	A string containing the NMEA message identifier string corresponding to
	the value of MessageTypeId (i.e. "CARXD", "CADQF", etc.)
*/
const string iWhoiMicroModem::MessageIdToMessageTypeString( int MessageId )
{
	map<int, const string>::iterator iter = m_MessageIdStringTable.find(MessageId);
	return ( iter != m_MessageIdStringTable.end() ) ? iter->second : "";
}




//============================================================================
/* Assembles a NMEA sentence around a specified command and data set, and
	sends it to the modem over the application's serial port

@details
	This function first assembles a NMEA sentence from a vector of string
	data corresponding to the comma-delimited fields of the corresponding NMEA
	sentence to be sent to the modem.  It then calculates and appends the CRC
	value and terminating Bytes for the NMEA sentence, and sends the data to
	the modem.

@param vNmeaFields
	Reference to a vector of strings containing command/data fields
	corresponding to the comma-delimited fields (not including CRC) of the
	NMEA sentence to be sent to the modem.

@return
	true on success; false if vNmeaFields is empty or if not all data was
	successfully sent to the serial port
*/
bool iWhoiMicroModem::SendNmeaVectorToModem( vector<string>& vNmeaFields )
{
	bool Rc = false;
//	char Checksum = 0;		// NMEA checksum as the XOR of all characters between the $ and *
//	char szHexBuffer[6];	// Used to print hex values

	if (!vNmeaFields.empty())
	{
		// Combine elements in vNmeaFields into a comma-separated
		// NMEA string
		string sNmeaSentence = "$";
		vector<string>::iterator iter = vNmeaFields.begin();
		do
		{
			sNmeaSentence += *iter + ',';
		}
		while ( ++iter != vNmeaFields.end() );

		// No comma before checksum field!
		sNmeaSentence.erase( sNmeaSentence.length() - 1 );

		// Calculate and append NMEA checksum field
		sNmeaSentence = sNmeaSentence + "*" + CreateNmeaChecksum(sNmeaSentence);

		// Send the data to the modem
		Rc = SendToModem( sNmeaSentence );
	}
	else
	{
		Rc = false;
	}

	return Rc;
}




//============================================================================
/** Returns a string containing a NMEA checksum calculated as the bitwise
	exclusive-or of all characters in a specified string

@param sNmeaSentence
	Characters preceding the checksum field of a NMEA sentence.  NOTE: if a
	leading '$' character is included, it will be skipped when calculating
	the checksum.
*/
string iWhoiMicroModem::CreateNmeaChecksum( string& sNmeaSentence )
{
	string sChecksum = "";
	unsigned char Accum = 0;
	char szHexBuffer[6];

	unsigned int StartIndex = sNmeaSentence.find_first_not_of("$");
	if (StartIndex != string::npos)
	{
		for (unsigned int i = StartIndex; i < sNmeaSentence.length(); i++)
		{
			Accum ^= sNmeaSentence[i];
		}

		sprintf(szHexBuffer, "%02X\r\n", Accum);
		sChecksum = szHexBuffer;
	}

	// Return the calculated checksum or an empty string on error
	return sChecksum;
}





//============================================================================
/** Thread-safe function to send a string of data to the modem

@param sTxData
	String containing Bytes to be sent to the modem

@return
	true if the data was sent successfully; else false
*/
bool iWhoiMicroModem::SendToModem( const string& sTxData )
{
	bool Rc = false;

	if ( !sTxData.empty() )
	{
		m_ModemTxLock.Lock();		// Lock modem for sending

		// Note timeout parameter is only populated in Linux,
		// so it's not used here.
		unsigned int NumBytesWritten = m_Port.Write( sTxData.c_str(),
		                                             sTxData.length(), 0 );
		Rc = ( NumBytesWritten == sTxData.length() );

		m_ModemTxLock.UnLock();

		// If PRINT_MODEM_TX = true in mission file, print transmitted data
		// to stdio
		if (m_ShouldPrintModemTx)
		{
			MOOSTrace( "[WHOI Tx]: " + sTxData + "\n" );
		}
	}

	return Rc;
}




bool iWhoiMicroModem::LoadConfiguration( void )
{
	bool Success, BoolValue;
	int IntValue;
	string sKey;
	string sCommandPriorityKeys =   "PRIORITY_SendUserMiniPacket,"
									"PRIORITY_WriteASCIIData,"
									"PRIORITY_ReadASCIIData,"
									"PRIORITY_WriteBinaryData,"
									"PRIORITY_ReadBinaryData,"
									"PRIORITY_SendMiniPacketPing,"
									"PRIORITY_SendFmSweepPing,"
									"PRIORITY_SendRemusPing,"
									"PRIORITY_SendNarrowBandPing,"
									"PRIORITY_SetRealtimeClock,"
									"PRIORITY_SetNvramParamValue,"
									"PRIORITY_GetNvramParamValue,"
									"PRIORITY_SetIOLineState,"
									"PRIORITY_ModemSleep,"
									"PRIORITY_AutoLevelAgc,"
									"PRIORITY_MeasureNoiseLevel";

	// NOTE: SRC has been neglected here, as it is loaded in OnStartup
	string sConfigParamKeys =	"AGC,AGN,ASD,BDD,BND,BR2,BW0,CPR,CRL,CST,CTO,DBG,DGM,"
								"DOP,DQF,DTH,DTO,DTP,ECD,EFF,EFB,FCO,FML,FMD,GPS,HFC,"
								"IRE,MOD,MFD,MSE,MCM,MPR,MVM,NDT,NPT,NRL,NRV,PAD,PCM,"
								"PRL,PTH,POW,PTO,REV,RSP,RXA,RXD,RXP,SCG,SGP,SHF,SNR,"
								"SNV,SRC,TAT,TOA,TXD,XST";

	Success = true;

	// NOTE: vehicle ID gets loaded from AUV_VEHICLE_ID at startup

	//------------------------------------------------------
	// Load WHOI command priorities from configuration file
	m_CommandPriorityTableLock.Lock();

	// Loop through the Command Priority key table,
	// load each key and priority from the configuration file
	m_vCommandPriorityTable[WhoiCommandManager::ClearCommandQueue] = 10000;	// Always top priority!
	for (int i = 0; i < WhoiCommandManager::NUM_COMMAND_TYPE_IDS; i++)
	{
		sKey = MOOSChomp(sCommandPriorityKeys, ",", true);

		if ( !sKey.empty() )
		{
			// Read priority from the config file
			if (m_MissionReader.GetConfigurationParam(sKey, IntValue))
			{
				// Assign the priority
				m_vCommandPriorityTable[i] = IntValue;
			}
			else
			{
				m_vCommandPriorityTable[i] = 0;
				MOOSTrace("Warning: Priority not specified for command %s."
						  "Using default value of 0.\n", sKey.c_str() );
			}
		}
	}
	//while ( !sKey.empty() );

	m_CommandPriorityTableLock.UnLock();




	//-----------------------------------------------------------
	// Read NVRAM parameter values from the configuration file
	do
	{
		sKey = MOOSChomp(sConfigParamKeys, ",", true);

		if ( !sKey.empty() )
		{
			// Read parameter value from the configuration file
			if (m_MissionReader.GetConfigurationParam(sKey, IntValue))
			{
				// If it was found, assign the value
				m_NvramParamValues.insert( pair<string, int>(sKey, IntValue) );
			}
		}
	}
	while ( !sKey.empty() );

	// Set the SRC NVRAM value from the Acoustic Network ID loaded in OnStartup
	string sConfigFilePath;
	m_NvramParamValues.insert( pair<string, int>("SRC", (int)m_CommandMgr->GetAcousticNetworkId()) );

	if (m_MissionReader.GetConfigurationParam("PRINT_MODEM_SETTINGS", BoolValue))
	{
		string sLine('-',45);

		// print NVRAM settings loaded from the file
		MOOSTrace( "\n" + sLine + "\n**  NVRAM SETTINGS  **\n" + sLine + "\n\n");

		map<const string, int>::iterator iter;
		for(iter = m_NvramParamValues.begin(); iter != m_NvramParamValues.end(); ++iter )
		{
			MOOSTrace("%s:\t%d\n", iter->first.c_str(), iter->second);
		}
		MOOSTrace(sLine + "\n\n");
	}

	// Read path of vehicle-specific data from the mission file (global value)
	if (m_MissionReader.GetValue("VEHICLE_CONFIG_FILE_PATH", sConfigFilePath))
	{
		CMOOSFileReader Fr;
		if (!sConfigFilePath.empty())
		{
			// Open the vehicle-specific config file and read in the vehicle ID
			if (Fr.SetFile(sConfigFilePath))
			{
				if (Fr.GetValue("VEHICLE_ID", IntValue) )
				{
					m_CommandMgr->SetAcousticNetworkId(IntValue);
				}
			}
		}
	}

	// If we failed to load the Acoustic ID from the vehicle config file, use
	// the SRC parameter in the NVRAM settings
	if (IntValue < 0)
	{
		MOOSTrace("WARNING! failed to load the path of the file containing "
				  "vehicle-specific data from the mission file.  Using the "
				  "value of SRC from NVRAM settings in the mission file "
				  "instead.\n");

		// Read acoustic ID from iWhoiMicroModem NVRAM settings instead
		if (m_MissionReader.GetConfigurationParam("SRC", IntValue))
		{
			m_CommandMgr->SetAcousticNetworkId(IntValue);
		}
		else
		{
			MOOSFail("Failed to load acoustic network ID!  Value must be "
					 "specified in the mission file as AUV_VEHICLE_ID or "
					 "iWhoiMicroModem:SRC\n");
		}
	}

	//MOOSTrace("\n-----------------------------------------\n"
	//		  "** Operating as Acoustic Network ID %d **"
	//		  "\n-----------------------------------------\n\n",
	//		  m_CommandMgr->GetAcousticNetworkId());


	// Determine which debugging messages are enabled
	m_MissionReader.GetConfigurationParam("PRINT_MODEM_TX", m_ShouldPrintModemTx);
	m_MissionReader.GetConfigurationParam("PRINT_MODEM_RX", m_ShouldPrintModemRx);
	m_MissionReader.GetConfigurationParam("ENABLE_PROMISCUOUS_DATARX", m_EnablePromiscuousDataRx);
	m_MissionReader.GetConfigurationParam("ENABLE_PROMISCUOUS_MINIPACKETRX", m_EnablePromiscuousMiniPacketRx);

	return Success;
}







//=========================================================================
/* This function handles serial messages received from the modem that are
	used internally by iWhoiMicroModem

@param MessageTypeId
	Integer type of the message

@param vMessageFields
	A vector of strings containing the comma-delimited fields of the
	received message

@return
	true if the message should be passed on to listeners; else false to
	prevent listeners from seeing the message
*/
bool iWhoiMicroModem::InternalRxHandlerDispatch( int MessageTypeId, vector<string>& vMessageFields )
{
	bool ReturnVal = false;
	vMessageFields.erase( vMessageFields.begin() );	// Get rid of the message ID string

	switch (MessageTypeId)
	{
		case MSGID_M2H_CACFG:		// Modem reports the value of a configuration parameter
			ReturnVal = CACFG_Handler(vMessageFields);
			break;

		case MSGID_M2H_CACLK:		// Modem reports the current RTC time
			ReturnVal = CACLK_Handler(vMessageFields);
			break;

		case MSGID_M2H_CACYC:		// Modem reports a synchronous acoustic comm init (could be local or remote...)
			ReturnVal = CACYC_Handler(vMessageFields);
			break;

		case MSGID_M2H_CADBG:		// Low-level debugging messages
			ReturnVal = CADBG_Handler(vMessageFields);
			break;

		case MSGID_M2H_CADOP:		// Modem reports relative Doppler (m/sec)
			ReturnVal = CADOP_Handler(vMessageFields);
			break;

		case MSGID_M2H_CADQF:		// Modem reports data quality factor for last FSK packet
			ReturnVal = CADQF_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAERR:		// Modem is reporting an error
			ReturnVal = CAERR_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAMFD:		// Modem reports matched filter info for communication
			ReturnVal = CAMFD_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAMSG:		// Modem reports a system information message such as a timeout when expecting a data reply
			ReturnVal = CAMSG_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAMUA:		// A user mini-packet was received acoustically
			ReturnVal = CAMUA_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAREV:		// Modem reports application name and revision number
			ReturnVal = CAREV_Handler(vMessageFields);
			break;

		case MSGID_M2H_CARXA:		// Modem reports received ASCII data frame
			ReturnVal = CARXA_Handler(vMessageFields);
			break;

		case MSGID_M2H_CARXD:		// Modem reports received binary (hex) data frame
			ReturnVal = CARXD_Handler(vMessageFields);
			break;

		case MSGID_M2H_CARXP:		// Modem reports the start of a (data?) packet
			ReturnVal = CARXP_Handler(vMessageFields);
			break;

		case MSGID_M2H_SNMFD:		// Modem reports nav ping matched-filter info
			ReturnVal = SNMFD_Handler(vMessageFields);
			break;

		case MSGID_M2H_CACST:		// Modem reports received binary (hex) data frame
			ReturnVal = CACST_Handler(vMessageFields);
			break;

		case MSGID_M2H_CAXST:		// Modem reports the start of a (data?) packet
			ReturnVal = CAXST_Handler(vMessageFields);
			break;

		default:
			ReturnVal = true;	// Pass messages not handled internally along to
			break;				// message listeners
	}

/*
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
	return ReturnVal;
}





//=========================================================================
// Handler function for received CACFG messages from modem
// in which the modem reports an NVRAM parameter value
bool iWhoiMicroModem::CACFG_Handler( vector<string>& vMessageFields )
{
	map<const string, int>::iterator iter;
	string sParamName, sParamValue;
	int ParamValue;

	// Ensure we have an even number of parameter/value pairs
	if ( !vMessageFields.empty() && ((vMessageFields.size() & 1) != 0) )
	{

		for (unsigned int i = 0; i < vMessageFields.size(); i += 2)
		{
			sParamName = vMessageFields[i];
			sParamValue = vMessageFields[i+1];
			if (sParamName.empty() || sParamValue.empty())
			{
				continue;
			}
			else
			{
				ParamValue = atoi( sParamValue.c_str() );

				// Update the NVRAM parameter value currently registered
				iter = m_NvramParamValues.find(sParamName);
				if ( iter != m_NvramParamValues.end() )
				{
					if (ParamValue != iter->second)
					{
						iter->second = ParamValue;
					}
				}
			}
		}
	}

	return false;		// Don't pass on NVRAM parameters to listeners
}




//=========================================================================
// Handler function for received CACLK messages from modem
// in which the modem reports the real-time clock (RTC) time
bool iWhoiMicroModem::CACLK_Handler( vector<string>& vTokens )
{
	if (vTokens.size() >= 6)
	{
		string sTimeValue = "Year=" + vTokens[0] + ",Month=" + vTokens[1] +
							",Day=" + vTokens[2] + ",Hours=" + vTokens[3] +
							",Minutes=" + vTokens[4] + ",Seconds=" + vTokens[5];

		NotifyThreadSafe(GetAppName() + "_RTC_Time", sTimeValue);
	}
	return true;
}




//=========================================================================
// Handler function for received CACYC messages from modem
// in which the modem reports a comms cycle init (either as a result of the
// host starting acoustic comms or a remote device requesting to talk
bool iWhoiMicroModem::CACYC_Handler( vector<string>& vTokens )
{
	if (vTokens.size() >= 6)
	{
		int CurrentCmd = m_CommandMgr->GetCurrentExecutingCommandId();

		// If the cycle init originated from data we're sending, don't
		// report it to the MOOSDB
		if ( (CurrentCmd == WhoiCommandManager::WriteASCIIData_CCTXA) ||
			 (CurrentCmd == WhoiCommandManager::WriteBinaryData_CCTXD) )
		{
			return true;
		}

		int NetworkId = m_CommandMgr->GetAcousticNetworkId();
		if ( (atoi(vTokens[1].c_str()) == NetworkId) || (atoi(vTokens[2].c_str()) == NetworkId) )
		{
			string sTimeValue = "CMD=" + vTokens[0] + ",SrcID=" + vTokens[1] +
								",DestID=" + vTokens[2] + ",PacketType=" + vTokens[3] +
								",AckRequested=" + vTokens[4] + ",NumFrames=" + vTokens[5];

			NotifyThreadSafe(GetAppName() + "_RxCycleInit", sTimeValue);
		}
	}
	return true;
}




//=========================================================================
// Handler function for received CADBG messages from modem
// in which the modem reports low-level debugging information
bool iWhoiMicroModem::CADBG_Handler( vector<string>& vTokens )
{
	if ( !vTokens.empty() )
	{
		NotifyThreadSafe(GetAppName() + "_LowLevelDebug", "Message=" + vTokens[0]);
	}

	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CADOP messages from modem
// in which the modem reports relative doppler speed (m/sec)
bool iWhoiMicroModem::CADOP_Handler( vector<string>& vTokens )
{
	if ( !vTokens.empty() )
	{
		NotifyThreadSafe(GetAppName() + "_DopplerSpeed", "MetersPerSecond=" + vTokens[0]);
	}
	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CADQF messages from modem
// in which the modem reports the data quality factor for the last received
// FSK packet
bool iWhoiMicroModem::CADQF_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 2 )
	{
		string sValue = "DQF=" + vTokens[0] + ",PacketType=" + vTokens[1];
		NotifyThreadSafe(GetAppName() + "_FSKDataQuality", sValue);
	}

	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CAERR messages from modem
// in which the modem reports an error
bool iWhoiMicroModem::CAERR_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 3 )
	{
		string sValue = "Time=" + vTokens[0] + ",Module=" + vTokens[1] +
						",ErrorNumber=" + vTokens[2];

		NotifyThreadSafe(GetAppName() + "_ErrorMsg", sValue);
	}

	return true;
}




//=========================================================================
// Handler function for received CAMFD messages from modem
// in which the modem reports matched filter information for communications
bool iWhoiMicroModem::CAMFD_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 4 )
	{
		string sValue = "PeakValue=" + vTokens[0] + ",PowerdB=" + vTokens[1] +
						",Ratio=" + vTokens[2] + ",SPLdB=" + vTokens[3];

		NotifyThreadSafe(GetAppName() + "_MatchedFilterCOMMS", sValue);
	}

	return false;	// Don't pass this on to the listener queue
}




//=========================================================================
// Handler function for received CAMSG messages from modem
// in which the modem reports a bad CRC or data packet timeout
bool iWhoiMicroModem::CAMSG_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 2 )
	{
		string sValue = "Type=" + vTokens[0] + ",Number=" + vTokens[1];
		NotifyThreadSafe(GetAppName() + "_TransactionMessage", sValue);
	}
	return true;
}



//=========================================================================
// Handler function for received CAMUA messages from modem
// in which the modem reports that a user mini-packet was received
// acoustically
bool iWhoiMicroModem::CAMUA_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 3 )
	{
		if ( m_EnablePromiscuousMiniPacketRx ||
			(vTokens[1] == MOOSFormat("%u", m_CommandMgr->GetAcousticNetworkId())) )
		{
			string sValue = "SrcID=" + vTokens[0] +
							",DestID=" + vTokens[1] +
							",Data=" + vTokens[2];

			NotifyThreadSafe(GetAppName() + "_RxUserMiniPacket", sValue);
		}
	}
	return true;
}



//=========================================================================
// Handler function for received CAREV messages from modem
// in which the modem reports its application name and firmware revision
// number
bool iWhoiMicroModem::CAREV_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 3 )
	{
		string sValue = "IDENT=" + vTokens[1] +
						",Revision=" + vTokens[2];
		string sVarName;

		if (!m_ModemIsConnected)
		{
			m_ModemIsConnected = true;
			NotifyThreadSafe(GetAppName() + "_HardwareState", "Ready");
			MOOSTrace("\n\n*** The Micro-Modem is on-line ***\n\n");
		}

		sVarName = (MOOSStrCmp(vTokens[1], "COPROC")) ?
					(GetAppName() + "_CoprocRevision"):"uMODEM_SoftwareRevision";

		NotifyThreadSafe(sVarName, sValue);
	}

	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CARXA messages from modem
// in which the modem reports a received ASCII data frame
bool iWhoiMicroModem::CARXA_Handler( vector<string>& vTokens )
{
	return true;
}



//=========================================================================
// Handler function for received CARXD messages from modem
// in which the modem reports a received (hex-coded) binary data frame
bool iWhoiMicroModem::CARXD_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 5 )
	{
		// Only publish data addressed to our acoustic network ID unless
		// promiscuous mode is enabled
		if ( m_EnablePromiscuousDataRx ||
			 (vTokens[0] == MOOSFormat("%u", m_CommandMgr->GetAcousticNetworkId())) )
		{
			string sValue = "SrcID=" + vTokens[0] +
							",DestID=" + vTokens[1] +
							",AckReq=" + vTokens[2] +
							",FrameNumber=" + vTokens[3] +
							",Data=" + vTokens[4];

			NotifyThreadSafe(GetAppName() + "_RxBinaryData", sValue);
		}
	}

	return true;
}



//=========================================================================
// Handler function for received CARXP messages from modem
// in which the modem reports that an incoming packet is detected
bool iWhoiMicroModem::CARXP_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 1 )
	{
		string sValue = "Type=" + vTokens[0];

		NotifyThreadSafe(GetAppName() + "_RxPacketStart", sValue);
	}

	return true;
}



//=========================================================================
// Handler function for received SNMFD messages from modem
// in which the modem reports matched filter detector info for nav pings
bool iWhoiMicroModem::SNMFD_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 4 )
	{
		string sValue = "Channel=" + vTokens[0] + ",Peak=" + vTokens[1] +
						",Power=" + vTokens[2] + ",Ratio=" + vTokens[3];

		NotifyThreadSafe(GetAppName() + "_MatchedFilterNAV", sValue);
	}

	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CAXST messages from modem
// in which the modem reports communication cycle Tx statistics
bool iWhoiMicroModem::CAXST_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 15 )
	{
		string sValue = "Date=" + vTokens[0] +
						",Time=" + vTokens[1] +
						",TimingMode=" + vTokens[2] +
						",Mode=" + vTokens[3] +
						",ProbeLength=" + vTokens[4] +
						",Bandwidth=" + vTokens[5] +
						",CarrierHz=" + vTokens[6] +
						",PacketRate=" + vTokens[7] +
						",SrcID=" + vTokens[8] +
						",DestID=" + vTokens[9] +
						",AckRequest=" + vTokens[10] +
						",ExpectedFrames=" + vTokens[11] +
						",FramesSent=" + vTokens[12] +
						",PacketType=" + vTokens[13] +
						",NumBytes=" + vTokens[14];

		NotifyThreadSafe(GetAppName() + "_CycleTxStats", sValue);
	}

	return false;	// Don't pass this on to the listener queue
}



//=========================================================================
// Handler function for received CACST messages from modem
// in which the modem reports communication cycle Rx statistics
bool iWhoiMicroModem::CACST_Handler( vector<string>& vTokens )
{
	if ( vTokens.size() >= 27 )
	{
		string sValue = "Mode=" + vTokens[0] +
						",ArrivalTime=" + vTokens[1] +
						",TOAMode=" + vTokens[2] +
						",MFDPeak=" + vTokens[3] +
						",MFDPower=" + vTokens[4] +
						",MFDRatio=" + vTokens[5] +
						",SPL=" + vTokens[6] +
						",SHF_AINPSHIFT=" + vTokens[7] +
						",SHF_AINSHIFT=" + vTokens[8] +
						",SHF_MFDSHIFT=" + vTokens[9] +
						",SHF_P2BSHIFT=" + vTokens[10] +
						",PacketRate=" + vTokens[11] +
						",SrcID=" + vTokens[12] +
						",DestID=" + vTokens[13] +
						",PskErrCode=" + vTokens[14] +
						",PacketType=" + vTokens[15] +
						",ExpectedFrames=" + vTokens[16] +
						",NumBadFrames=" + vTokens[17] +
						",SNR_RSS=" + vTokens[18] +
						",SNR_In=" + vTokens[19] +
						",SNR_Out=" + vTokens[20] +
						",SymbolsSNR=" + vTokens[21] +
						",MSE=" + vTokens[22] +
						",DQF=" + vTokens[23] +
						",DOP=" + vTokens[24] +
						",StdevNoise=" + vTokens[25];

		NotifyThreadSafe(GetAppName() + "_CycleRxStats", sValue);
	}

	return false;	// Don't pass this on to the listener queue
}


//=========================================================================
// Handler function for GPS "GP" NMEA sentences passed through from the
// modem's auxilliary serial port
bool iWhoiMicroModem::GPS_PassThru_Handler( vector<string>& vNmeaFields )
{
	stringstream ss;

	// Add the NMEA fields to the message
	for ( vector<string>::iterator iter = vNmeaFields.begin();
		  iter != vNmeaFields.end(); iter++ )
	{
		ss << *iter << ',';
	}

	NotifyThreadSafe(GetAppName() + "_GpsThru", ss.str() );

	return false;
}
