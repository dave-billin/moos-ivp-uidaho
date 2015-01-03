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
/** @file WhoiCommandManager.cpp

@brief
	Implementation of the WhoiCommandManager class defined in
	WhoiCommandManager.h

@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the
	University of Idaho, USA.
*/
//=============================================================================
#include <iostream>
#include <sstream>
#include "WhoiCommandManager.h"

using namespace std;
using YellowSubUtils::TimedLock;
using YellowSubUtils::Semaphore;


typedef void (WhoiCommandManager::*pHandlerFn)(const std::string& );


// Packet type ID's used when sending binary and ASCII data frames
enum e_CCCYC_TxDataPacketTypeIds
{
	PacketType_80bps=0,		/* 80 bps FH-FSK */
	PacketType_250bps,		/* 250 bps 1/31 spreading */
	PacketType_500bps,		/* 500 bps 1/15 spreading */
	PacketType_1200bps,		/* 1200 bps 1/7 spreading */
	PacketType_1300bps,		/* 1300 bps 1/6 rate block code */
	PacketType_5300bps,		/* 5300 bps 9/14 rate block code */
	PACKETTYPE_NUMPACKETTYPEIDS	/* Used for bounds-check not a valid type */
};


static int TxDataPacketMinSizeTable[PACKETTYPE_NUMPACKETTYPEIDS] =
		{	32,		/* 80 bps FH-FSK */
			32,		/* 250 bps 1/31 spreading */
			64,		/* 500 bps 1/15 spreading */
			256,	/* 1200 bps 1/7 spreading */
			256,	/* 1300 bps 1/6 rate block code */
			256	};	/* 5300 bps 9/14 rate block code */

static int TxDataPacketMaxSizeTable[PACKETTYPE_NUMPACKETTYPEIDS] =
		{	32,		/* 80 bps FH-FSK */
			96,		/* 250 bps 1/31 spreading */
			192,	/* 500 bps 1/15 spreading */
			512,	/* 1200 bps 1/7 spreading */
			512,	/* 1300 bps 1/6 rate block code */
			2048 };	/* 5300 bps 9/14 rate block code */


//=========================================================================
// A file scope function to which a CMOOSThread object can be attached to
// associate with a particular instance of the WhoiCommandManager */
static bool WhoiCmdMgrThreadLauncher( void * pObject)
{
	WhoiCommandManager* pWhoiMgrInstance = (WhoiCommandManager*)pObject;
	return pWhoiMgrInstance->CommandManagerThreadBody();
}





//=========================================================================
WhoiCommandManager::WhoiCommandManager( iWhoiMicroModem& ModemObjectRef,
										  unsigned int AcousticNetworkId )
	: m_ModemObject(ModemObjectRef),
	  m_TxCommandsEnabled(true),
	  m_CommandMgrThread(),
	  m_CommandQueueLock(false),
	  m_CommandQueueSemaphore(0),
	  m_ListenSemaphore(0),
	  m_CurrentExecutingCommandId( ClearCommandQueue ),
	  m_ReplyTimeStamp(0.0)

{
	SetAcousticNetworkId(AcousticNetworkId);
	m_CommandMgrThread.Initialise(WhoiCmdMgrThreadLauncher, this);
}


WhoiCommandManager::~WhoiCommandManager()
{

}




//=========================================================================
/* Called to post a new command to be executed on the WHOI modem

@param CommandTypeID
	The type of the command to be executed from e_CommandTypeIds

@param sCommandParameters
	A string containing comma-separated parameter values for the command

@param CommandPriority
	The priority to be assigned to the command (0 = lowest priority)
*/
void WhoiCommandManager::PostCommand( const unsigned int CommandTypeID,
									   const string& sCommandParameters,
									   const unsigned int CommandPriority )
{
	if (CommandTypeID < NUM_COMMAND_TYPE_IDS)
	{
		if (m_ModemObject.m_ModemIsConnected)
		{
			QueuedCommandEvent Cmd( CommandTypeID, sCommandParameters,
									 CommandPriority );

			// Make sure we can lock the Tx queue
			if ( !m_CommandQueueLock.Lock(250) )
			{
				MOOSTrace("[WhoiCommandManager::PostCommand] FAILED TO LOCK TX QUEUE!\n");
			}
			else
			{
				m_CommandQueue.push(Cmd);	// Add the command to the queue

				// Post the semaphore to wake up the worker thread
				m_CommandQueueSemaphore.Post();
				m_CommandQueueLock.UnLock();
			}
		}
	}
}



//=========================================================================
/* Starts the command manager worker thread */
void WhoiCommandManager::Start(void)
{
	if ( !m_CommandMgrThread.IsThreadRunning() )
	{
		m_CommandMgrThread.Start();
	}
}


//=========================================================================
/* Stops the command manager worker thread */
void WhoiCommandManager::Stop(void)
{
	if ( m_CommandMgrThread.IsThreadRunning() )
	{
		m_CommandMgrThread.Stop();
	}
}



//=========================================================================
// Sets the ID of this vehicle on the acoustic network
void WhoiCommandManager::SetAcousticNetworkId(unsigned int NewId)
{
	m_AcousticNetworkId = NewId;
	m_sAcousticNetworkId = MOOSFormat("%d", NewId);
}



//=========================================================================
/* Posts a string value to a Notify target specified in a modem command
	parameter string if one exists

@param sParamString
	Modem command parameter string that may or may not contain a
	Notify=[TARGET] pair

@param sNotifyValue
	The value to be posted to the Notify target if it exists
*/
void WhoiCommandManager::PostToNotifyTarget( const string& sParamString, const string sNotifyValue )
{
	string sNotifyTarget;
	if ( MOOSValFromString( sNotifyTarget, sParamString, "Notify", true) )
	{
		m_ModemObject.NotifyThreadSafe( sNotifyTarget, sNotifyValue, MOOSTime() );
	}
}




//=========================================================================
/* This is the execution body of the secondary thread used to carry out
	commands posted to the Command Queue
*/
bool WhoiCommandManager::CommandManagerThreadBody( void )
{
	// A handler function lookup table
	pHandlerFn CommandFunctionTable[NUM_COMMAND_TYPE_IDS] =
					  { &WhoiCommandManager::SendUserMiniPacket,
						&WhoiCommandManager::WriteASCIIData,
						&WhoiCommandManager::ReadASCIIData,
						&WhoiCommandManager::WriteBinaryData,
						&WhoiCommandManager::ReadBinaryData,
						&WhoiCommandManager::DoMiniPacketPing,
						&WhoiCommandManager::DoFmSweepPing,
						&WhoiCommandManager::DoRemusPing,
						&WhoiCommandManager::DoNarrowBandPing,
						&WhoiCommandManager::SetRealtimeClock,
						&WhoiCommandManager::SetNvramParamValue,
						&WhoiCommandManager::GetNvramParamValue,
						&WhoiCommandManager::SetHardwareIOLineState,
						&WhoiCommandManager::SendModemSleepCommand,
						&WhoiCommandManager::DoAgcAutoLevelAdjust,
						&WhoiCommandManager::SendMeasureNoiseLevel,
						&WhoiCommandManager::DoClearCommandQueue };

	do
	{
		// Signal that the Command Manager thread is idle until we start
		// executing a command
		m_CurrentExecutingCommandId = -1;

		// Wake up every 100ms to check whether we need to exit
		m_CommandQueueSemaphore.Wait(100);

		if ((!m_CommandMgrThread.IsQuitRequested()) &&
			(m_CommandQueue.size() > 0) )
		{
			// Get the next command from the queue
			m_CommandQueueLock.Lock();
			const QueuedCommandEvent TheCommand = m_CommandQueue.top();
			m_CommandQueue.pop();
			m_CommandQueueLock.UnLock();

			// If the modem isn't connected, post a NoHardware error to the Notify
			// target of the command if one exists
			if (!m_ModemObject.m_ModemIsConnected)
			{
				PostToNotifyTarget(TheCommand.sCommandParams, "Error=NoHardware");
			}

			// Call the appropriate command handler
			else if (TheCommand.CommandTypeID < NUM_COMMAND_TYPE_IDS)
			{
				// Signal which command we're executing
				m_CurrentExecutingCommandId = TheCommand.CommandTypeID;

				// Get a pointer to the member function to be called
				pHandlerFn TheHandler = CommandFunctionTable[TheCommand.CommandTypeID];

				// WHOA!  Talk about some MESSED UP syntax!  Check it out:
				// Here's the function call...
				(this->*TheHandler)( TheCommand.sCommandParams );
			}
		}

	}
	while ( !m_CommandMgrThread.IsQuitRequested() );

	return true;
}



//=========================================================================
/* Waits for a CAERR (error report) or a specified message to arrive from
	the modem

@param sMessageKey
	String containing the message key to wait for (not including the
	leading '$' character from the NMEA sentence)

@param TimeoutMs
	Maximum number of milliseconds to wait for the message to arrive

@param OUT_vMessageFields
	A vector of strings to be populated with the fields of the target
	message if it is received before the timeout elapses

@return
	- 1 if the specified message was received before the timeout expired
	- 0 if the timeout expired before the message was received
	- (-1) if a CAERR (error report) message was received before the
	  specified message arrived.
*/
int WhoiCommandManager::WaitForModemMessage( string& sMessageKey, int TimeoutMs,
											  vector<string> OUT_vMessageFields )
{
	return 0;
}





//! @internal
void WhoiCommandManager::SendUserMiniPacket( const string& sCommandParameters )
{
	// Make sure outgoing acoustic traffic is enabled
	if (!m_TxCommandsEnabled)
	{
		PostToNotifyTarget(sCommandParameters, "Error=TxDisabled");
		return;
	}

	// Validate parameters
	string sDest, sPayload;
	bool ParamsAreValid = false;
	if ( MOOSValFromString( sDest, sCommandParameters, "Dest", true) &&
		 MOOSValFromString( sPayload, sCommandParameters, "Payload", true) )
	{
		int PayloadValue = atoi(sPayload.c_str());
		if ( (sPayload.length() == 4) &&
			(PayloadValue < 0x1fff) )
		{
			ParamsAreValid = true;
		}
	}

	// Set up source ID.  If a "Src" field has been included in the command,
	// use it.  Otherwise, use the default acoustic ID
	string sSrc = m_sAcousticNetworkId;
	MOOSValFromString( sSrc, sCommandParameters, "Source", true );

	if (!ParamsAreValid)
	{
		// Report bad param(s) to the command's Notify target if there is one
		PostToNotifyTarget(sCommandParameters, "Error=BadParam");
		return;
	}
	else
	{
		// Set up the modem command
		m_vModemCommand.clear();
		m_vModemCommand.push_back("CCMUC");
		//m_vModemCommand.push_back(m_sAcousticNetworkId);
		m_vModemCommand.push_back(sSrc);
		m_vModemCommand.push_back(sDest);
		m_vModemCommand.push_back(sPayload);

		m_ModemObject.AddRxListener( "CAMUC", m_vModemReply, m_ReplyTimeStamp,
									 m_ListenSemaphore );
		m_ListenSemaphore.Reset();	// Reset the semaphore
		m_ModemObject.SendNmeaVectorToModem(m_vModemCommand);	// Send command to the modem

		// Wait for mini-packet echo from the modem
		if ( m_ListenSemaphore.Wait(4000) )
		{
			PostToNotifyTarget(sCommandParameters, "Error=None");
		}
		else
		{
			PostToNotifyTarget(sCommandParameters, "Error=Fail");
		}
	}
}

//=============================================================================
void WhoiCommandManager::WriteASCIIData( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::ReadASCIIData( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::WriteBinaryData( const string& sCommandParameters )
{
	string sDest, sPayload;
	int PacketType, NumPacketBytes, AckTimeout;
	bool ParamsAreValid = false;

	// Make sure outgoing acoustic traffic is enabled
	if (!m_TxCommandsEnabled)
	{
		PostToNotifyTarget(sCommandParameters, "Error=TxDisabled");
		return;
	}

	// Confirm that we have all parameters
	if ( MOOSValFromString( sDest, sCommandParameters, "Dest", true) &&
		 MOOSValFromString( PacketType, sCommandParameters, "PacketType", true) &&
		 MOOSValFromString( AckTimeout, sCommandParameters, "AckTimeoutSec", true) &&
		 MOOSValFromString( sPayload, sCommandParameters, "Payload", true) )
	{
		// Verify the packet type
		if (PacketType < PACKETTYPE_NUMPACKETTYPEIDS)
		{
			NumPacketBytes = (sPayload.length() >> 1);	// Divide by 2 to get # Bytes

			// Validate the size of the packet payload
			// Mod by 1 to make sure the number of Byte chars is even
			if ( ((sPayload.length() & 1) == 0) &&
				 (NumPacketBytes <= TxDataPacketMaxSizeTable[PacketType]) &&
				 (NumPacketBytes >= TxDataPacketMinSizeTable[PacketType]) )
			{
					ParamsAreValid = true;
			}
		}
	}

	if (!ParamsAreValid)
	{
		// Report bad param(s) to the command's Notify target if there is one
		PostToNotifyTarget(sCommandParameters, "Error=BadParam");
		return;
	}
	else
	{
		string sAckFlag = (AckTimeout > 0) ? "1":"0";

		// Calculate the number of packet frames
		int NumFrames = (NumPacketBytes / TxDataPacketMinSizeTable[PacketType]);
		if ((NumPacketBytes % TxDataPacketMinSizeTable[PacketType]) != 0)
		{
			NumFrames++;
		}

		m_vModemCommand.clear();
		m_vModemCommand.push_back("CCCYC");
		m_vModemCommand.push_back("1");						// CMD (DB: only 0 and 1 seem to work here...)
		m_vModemCommand.push_back(m_sAcousticNetworkId);	// SRC
		m_vModemCommand.push_back(sDest);					// DEST
		m_vModemCommand.push_back( MOOSFormat("%d", PacketType) );	// Packet Type
		m_vModemCommand.push_back( sAckFlag );				// ACK
		m_vModemCommand.push_back( MOOSFormat("%d", NumFrames) );	// Number of data frames

		// Set up the messages we want to listen for
		// DB: I'm testing with WHOI version "AUV 0.93.1.35" "COPROC 0.10.0.46"
		// and I'm finding that after sending a CCCYC, the modem fails to request
		// the data to be sent using a CADRQ message every other CCCYC.
		// The workaround for now is to listen for the CACYC message to trigger
		// sending data to the modem, rather than the CADRQ message.
		m_ModemObject.AddRxListener( "CAERR,CADRQ", m_vModemReply, m_ReplyTimeStamp,
									 m_ListenSemaphore );
		m_ListenSemaphore.Reset();	// Reset the semaphore
		m_ModemObject.SendNmeaVectorToModem(m_vModemCommand);	// Send command to the modem

		// Listen for error or modem to request the data to send
		if ( !m_ListenSemaphore.Wait(3000) )
		{
			PostToNotifyTarget(sCommandParameters, "Error=Fail");
			return;
		}

		// Catch errors signalled by the modem
		if ( m_vModemReply[0] == "CAERR" )
		{
			PostToNotifyTarget(sCommandParameters, "Error=Fail");
			return;
		}
		// Otherwise, if modem requested the data to be sent
		else
		{
			// Assemble the data and send it
			m_vModemCommand.clear();
			m_vModemCommand.push_back("CCTXD");
			m_vModemCommand.push_back(m_sAcousticNetworkId);	// Source
			m_vModemCommand.push_back(sDest);					// Dest
			m_vModemCommand.push_back(sAckFlag);				// Ack flag
			m_vModemCommand.push_back( sPayload );				// Hex data

			// Now wait for the modem to acknowledge the data
			m_ModemObject.AddRxListener( "CATXD", m_vModemReply, m_ReplyTimeStamp,
										 m_ListenSemaphore );
			m_ListenSemaphore.Reset();	// Reset the semaphore
			m_ModemObject.SendNmeaVectorToModem(m_vModemCommand);	// Send command to the modem
			if ( !m_ListenSemaphore.Wait(2000) )
			{
				PostToNotifyTarget(sCommandParameters, "Error=Fail");
				return;
			}

			// DB: the WHOI Software Interface document incorrectly lists
			// the packets reporting the start an end of packet transmission as
			// CCTXP and CCTXF respectively.  The modem software I am testing with
			// actually sends "CATXP" and "CATXF".

			// Wait for the modem to signal the end of the transmission
			m_ModemObject.AddRxListener( "CATXF", m_vModemReply, m_ReplyTimeStamp,
										 m_ListenSemaphore );
			m_ListenSemaphore.Reset();	// Reset the semaphore
			m_ModemObject.SendNmeaVectorToModem(m_vModemCommand);	// Send command to the modem
			// DB: Haven't been able to test high data-rate packets yet, so I'm setting
			// the timeout for this command based on the time required to send a single
			// 32-Byte data frame as packet type 0.
			if ( !m_ListenSemaphore.Wait(10000 * (NumPacketBytes >> 5)) )
			{
				PostToNotifyTarget(sCommandParameters, "Error=Fail");
				return;
			}

			string sTxTimeStamp = "TxTimeStamp=" + MOOSFormat("%f", MOOSTime());

			// If we're not listening for an acknowledgement from the receiver
			if (AckTimeout < 1)
			{
				PostToNotifyTarget(sCommandParameters, "Error=None," + sTxTimeStamp);
				return;
			}

			// Otherwise, wait for an acknowledgement from the receiving device
			m_ModemObject.AddRxListener( "CAACK", m_vModemReply, m_ReplyTimeStamp,
										 m_ListenSemaphore );
			m_ListenSemaphore.Reset();	// Reset the semaphore
			if ( m_ListenSemaphore.Wait(AckTimeout*1000) )
			{
				PostToNotifyTarget(sCommandParameters, "Error=None" + sTxTimeStamp);
			}
			else
			{
				// Signal that ack timeout elapsed
				PostToNotifyTarget(sCommandParameters, "Error=Timeout");
			}

		}
	}
}

//=============================================================================
void WhoiCommandManager::ReadBinaryData( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::DoMiniPacketPing( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::DoFmSweepPing( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::DoRemusPing( const string& sCommandParameters )
{
	bool ParamsAreValid = false;

	// Make sure outgoing acoustic traffic is enabled
	if (!m_TxCommandsEnabled)
	{
		PostToNotifyTarget(sCommandParameters, "Error=TxDisabled");
		return;
	}

	// Confirm that we have all command parameters
	string sGroup, sChannel, sSync, sSyncTimeoutms, sListenTimeoutms, sBeacons;
	if ( MOOSValFromString( sGroup, sCommandParameters, "Group", true) &&
		 MOOSValFromString( sChannel, sCommandParameters, "Channel", true) &&
		 MOOSValFromString( sSync, sCommandParameters, "Sync", true) &&
		 MOOSValFromString( sSyncTimeoutms, sCommandParameters, "SyncTimeoutms", true) &&
		 MOOSValFromString( sListenTimeoutms, sCommandParameters, "ListenTimeoutms", true) &&
		 MOOSValFromString( sBeacons, sCommandParameters, "Beacons", true) )
	{
		if (sBeacons.length() <= 4)
		{
			ParamsAreValid = true;
		}
	}

	if (!ParamsAreValid)
	{
		// Report bad param(s) to the command's Notify target if there is one
		PostToNotifyTarget(sCommandParameters, "Error=BadParam");
		return;
	}
	else
	{
		char szBeaconIds[] = "ABCD";
		string sVal;

		// Set up the modem command
		m_vModemCommand.clear();
		m_vModemCommand.push_back("CCPDT");
		m_vModemCommand.push_back(sGroup);
		m_vModemCommand.push_back(sChannel);

		// Setup hardware sync flag
		sVal = (MOOSStrCmp(sSync, "true")) ? "1":"0";
		m_vModemCommand.push_back(sVal);

		m_vModemCommand.push_back(sSyncTimeoutms);
		m_vModemCommand.push_back(sListenTimeoutms);

		// Set up beacon flags
		for (unsigned int i = 0; i < 4; i++)
		{
			sVal = (sBeacons.find(szBeaconIds[i], 0) != string::npos) ? "1":"0";
			m_vModemCommand.push_back(sVal);
		}

		m_ModemObject.AddRxListener( "SNTTA", m_vModemReply, m_ReplyTimeStamp,
									 m_ListenSemaphore );
		m_ListenSemaphore.Reset();	// Reset the semaphore
		m_ModemObject.SendNmeaVectorToModem(m_vModemCommand);	// Send command to the modem

		// Wait for mini-packet echo from the modem
		if ( m_ListenSemaphore.Wait(4000) )
		{
			// STATUS: we got an SNTTA message.  Validate the travel times
			if (m_vModemReply.size() >= 6)
			{
				sVal.clear();
				sVal = sVal + "Error=None,";
				for (int i = 1; i <= 4; i++)
				{
					if ( !m_vModemReply[i].empty() )
					{
						sVal = sVal + string("Time") + szBeaconIds[i-1]
									+ "=" + m_vModemReply[i] + ",";
					}
				}
				sVal = sVal + "PingTime=" + m_vModemReply[5];

				PostToNotifyTarget(sCommandParameters, sVal);
			}
		}
		else
		{
			PostToNotifyTarget(sCommandParameters, "Error=Fail");
		}
	}
}

//=============================================================================
void WhoiCommandManager::DoNarrowBandPing( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::SetRealtimeClock( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::SetNvramParamValue( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::GetNvramParamValue( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::SetHardwareIOLineState( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::SendModemSleepCommand( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::DoAgcAutoLevelAdjust( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::SendMeasureNoiseLevel( const string& sCommandParameters )
{
}

//=============================================================================
void WhoiCommandManager::DoClearCommandQueue( const string& sCommandParameters )
{
}
