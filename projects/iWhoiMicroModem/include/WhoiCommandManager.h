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
/** @file WhoiCommandManager.h

@brief
	Declaration of the WhoiCommandManager class used to manage sending of
	commands and data to the modem
	
@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the 
	University of Idaho, USA.
*/
//=============================================================================

#ifndef _CWHOITXMANAGER_H_
#define _CWHOITXMANAGER_H_

#include <string>
#include <queue>
#include <vector>

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Utils/MOOSThread.h"	// MOOS thread class
#include "YellowSubUtils.h"
#include "iWhoiMicroModem.h"
#include "QueuedCommandEvent.h"




// Pre-declaration of classes used in WhoiCommandManager
class iWhoiMicroModem;



//=============================================================================
/** An object that manages one or more worker threads and a Tx queue for 
	sending commands and data to the WHOI Micro Modem
*/
class WhoiCommandManager
{
public:

	/** Type IDs of commands that can be carried out on the WHOI modem */
	enum e_CommandTypeIds
	{
		SendUserMiniPacket_CCMUC=0,	/** CCMUC - Sends a user (13-bit) 
										mini-packet */

		WriteASCIIData_CCTXA,		/** Sends one or more frames of ASCII 
										data to a specified acoustic ID */

		ReadASCIIData_CCTXA,		/** Requests one or more frames of ASCII
										data from a specified acoustic ID */

		WriteBinaryData_CCTXD,		/** Sends one or more frames of 
										hex-encoded binary data to a specified
										acoustic ID*/

		ReadBinaryData_CCTXD,		/** Requests one or more frames of 
										hex-encoded binary data from a 
										specified acoustic ID */

		SendMiniPacketPing_CCMPC,	/** Sends a mini-packet PING */
		SendFmSweepPing_CCRSP,		/** Sends an FM sweep PING */
		SendRemusPing_CCPDT,		/** Pings the REMUS digital transponder */
		SendNarrowBandPing_CCPNT,	/** Pings a narrowband transponder */
		SetRealtimeClock_CCCLK,		/** Sets the modem's real-time clock */
		SetNvramParamValue_CCCFG,	/** Sets the value of an NVRAM parameter */
		GetNvramParamValue_CCCFQ,	/** Gets the value of an NVRAM parameter */
		SetIOLineState_CCCMEC,		/** Sets the state of an external hardware
										I/O line */
		ModemSleep_CCMSC,			/** Tells the modem to sleep for a 
										specified time */
		AutoLevelAgc_CCAGC,			/** Causes the modem to automatically 
										adjust its automatic gain control based
										on the signal level at the receiver */
		MeasureNoiseLevel_CCCFR,	/** Causes the modem to integrate and 
										measure the noise level at its receiver
										over a specified period of time */

		/* DB: Preserve enum order from here on... */
		ClearCommandQueue,			/**	Clears all pending commands in the
										command queue (careful with this 
										one...) */
		NUM_COMMAND_TYPE_IDS		/** Used for error-checking.  NOT A VALID
										COMMAND TYPE ID */
	};

	//=========================================================================
	/** Creates an instance of the WhoiCommandManager

	@param ModemObjectRef
		A reference to the iWhoiMicroModem object hosting the manager.  This is
		necessary to allow the Tx Manager to register as a listener to receive
		NMEA sentences from the modem

	@param AcousticNetworkId
		ID of the local device on the acoustic network.  This is value is used 
		as the SRC field of various acoustic communications.
	*/
	WhoiCommandManager( iWhoiMicroModem& ModemObjectRef, 
						 unsigned int AcousticNetworkId );

	//=========================================================================
	/** Called when the object goes out of scope */
	~WhoiCommandManager();


	//=========================================================================
	/** Returns true if the Command Manager worker thread is running */
	bool IsRunning( void )	{ return m_CommandMgrThread.IsThreadRunning(); }

	//=========================================================================
	/** Called to post a new command to be executed on the WHOI modem

	@param CommandTypeID
		The type of the command to be executed from e_CommandTypeIds 

	@param sCommandParameters
		A string containing comma-separated parameter values for the command

	@param CommandPriority
		The priority to be assigned to the command (0 = lowest priority)
	*/
	void PostCommand( const unsigned int CommandTypeID, 
					  const std::string& sCommandParameters,
					  unsigned int CommandPriority );


	//=========================================================================
	//! Sets the ID of this vehicle on the acoustic network
	void SetAcousticNetworkId(unsigned int NewId);
	unsigned int GetAcousticNetworkId(void)	{ return m_AcousticNetworkId; }

	//=========================================================================
	/** This is the execution body of the secondary thread used to carry out
		commands posted to the Command Queue
	*/
	bool CommandManagerThreadBody( void );


	//=========================================================================
	/** Starts the command manager worker thread */
	void Start(void);

	//=========================================================================
	/** Stops the command manager worker thread */
	void Stop(void);

	//=========================================================================
	/** Returns a member of e_CommandTypeIds containing the command that is 
		currently executing or -1 if the CommandManager thread is idle */
	int GetCurrentExecutingCommandId(void)	
			{ return m_CurrentExecutingCommandId; }

	//=========================================================================
	/** Used to control whether Tx commands should be carried out or disabled.
	@param TxShouldBeEnabled
		Set to true if commands resulting in outgoing acoustic traffic should 
		be processed or false to disable these commands.  If Tx commands are
		disabled, they will simply be discarded when they are removed from the
		command queue.
	*/
	void SetTxCommandEnablement( bool TxShouldBeEnabled )
	{ m_TxCommandsEnabled = TxShouldBeEnabled; }

	//! Returns true if processing of Tx commands is currently enabled.
	bool GetTxCommandEnablement( void ) { return m_TxCommandsEnabled; }

private:

	iWhoiMicroModem& m_ModemObject;		// Reference to the main thread
    bool m_TxCommandsEnabled;   // false to disable all Tx operations
	CMOOSThread m_CommandMgrThread;		// Internal worker thread
	YellowSubUtils::TimedLock m_CommandQueueLock;   // Tx queue mutex

    //  Used to allow the CommandManager's worker thread to sleep until
    //  new commands are posted
	YellowSubUtils::Semaphore m_CommandQueueSemaphore;

    // Semaphore used to block while waiting for a message from the
    // modem
	YellowSubUtils::Semaphore m_ListenSemaphore;

    unsigned int m_AcousticNetworkId;	// ID on the acoustic network
	std::string m_sAcousticNetworkId;		// ID as a string
	int m_CurrentExecutingCommandId;	// ID of current command being executed

	std::vector<std::string> m_vModemCommand;	// Vector used for sending modem commands
	std::vector<std::string> m_vModemReply;		// Vector used for listening for Rx messages
	double m_ReplyTimeStamp;			// Used when listening for Rx messages


	/** A queue of data messages to be sent via serial port to the modem.
		Elements of the queue are stored in descending order (i.e. highest
		priority elements are nearest the top of the queue */
	std::priority_queue <QueuedCommandEvent> m_CommandQueue;

	

	//=========================================================================
	/** Posts a string value to a Notify target specified in a modem command
		parameter string if one exists

	@param sParamString
		Modem command parameter string that may or may not contain a 
		Notify=[TARGET] pair

	@param sNotifyValue
		The value to be posted to the Notify target if it exists
	*/
	void PostToNotifyTarget( const std::string& sParamString,
	                         const std::string sNotifyValue );


	//=========================================================================
	/** Waits for a CAERR (error report) or a specified message to arrive from 
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
	int WaitForModemMessage( std::string& sMessageKey, int TimeoutMs,
							 std::vector<std::string> OUT_vMessageFields );
		



	/** @name Command handler functions
	@brief	
		These functions carry out commands posted to the Command queue.  
	@details
		All of these functions take the same arguments: a string 
		containing the relevant parameters needed for the command, and 
		the name of the MOOS variable to notify of the results.  No
		validation is conducted on these parameters, as they are 
		assumed to be constructed by a calling function in a closed 
		system (i.e. the iWhoiMicroModem class).  See the calling
		functions in iWhoiMicroModem for parameter documentation.
	
	*/
	//@{

	//! @internal
	void SendUserMiniPacket( const std::string& sCommandParameters );

	//! @internal
	void WriteASCIIData( const std::string& sCommandParameters );
	
	//! @internal
	void ReadASCIIData( const std::string& sCommandParameters );

	//! @internal
	void WriteBinaryData( const std::string& sCommandParameters );

	//! @internal
	void ReadBinaryData( const std::string& sCommandParameters );

	//! @internal
	void DoMiniPacketPing( const std::string& sCommandParameters );

	//! @internal
	void DoFmSweepPing( const std::string& sCommandParameters );

	//! @internal
	void DoRemusPing( const std::string& sCommandParameters );

	//! @internal
	void DoNarrowBandPing( const std::string& sCommandParameters );

	//! @internal
	void SetRealtimeClock( const std::string& sCommandParameters );

	//! @internal
	void SetNvramParamValue( const std::string& sCommandParameters );

	//! @internal
	void GetNvramParamValue( const std::string& sCommandParameters );

	//! @internal
	void SetHardwareIOLineState( const std::string& sCommandParameters );

	//! @internal
	void SendModemSleepCommand( const std::string& sCommandParameters );

	//! @internal
	void DoAgcAutoLevelAdjust( const std::string& sCommandParameters );

	//! @internal
	void SendMeasureNoiseLevel( const std::string& sCommandParameters );

	//! @internal
	void DoClearCommandQueue( const std::string& sCommandParameters );

	//-------------------------------------------------------------------------
	// Prevent automatic generation of copy constructor and assignment operator 
	WhoiCommandManager (const WhoiCommandManager&);
    const WhoiCommandManager& operator= (const WhoiCommandManager&);
};










#endif	// END #ifndef _CWHOITXMANAGER_H_
