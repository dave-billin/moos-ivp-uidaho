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
/** @file YellowSubDAQModule.h
 *
 * @brief
 *  An object to wrap status and communications with the YellowSub DSP data
 *  acquisition module.
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef _YELLOWSUBDAQMODULE_H_
#define _YELLOWSUBDAQMODULE_H_

#include <stdint.h>

#include "YellowSubUtils.h"
#include "LibBunnySock.h"
#include "UAVnet_PacketTypes.h"

/** An object to wrap status and BunnySock communications with a YellowSub
 *  DSP data acquisition module.
 */
class YellowSubDAQModule: public BunnySock::BunnySockListener
{
public:

   //=========================================================================
   /** Creates an instance of the object and starts its network connection
    *  thread
    *
    * @param HostName
    *	Hostname or IP address of the DAQ module to connect to
    *
    * @param Port
    *	Network port the target DAQ module is listening on for a BunnySock
    *	TCP connection
    *
    * @param ConnectionTimeoutMs
    *  Connection timeout to use for the BunnySock connection to the DAQ
    *  module in milliseconds
    *
    * @param Verbosity
    *	Verbosity level to use when printing debug messages (0 = none, 1+ gives
    *	increasing amounts of debugging messages
    *
    * @throw
    * 	A CMOOSException on failure to create a BunnySock network socket to
    * 	communicate with the DAQ module
    */
   YellowSubDAQModule(std::string& HostName, uint16_t Port,
                      uint32_t ConnectionTimeoutMs, int Verbosity = 0,
                      int BunnySockVerbosity = 0);

   /** Called when the object goes out of scope */
   virtual ~YellowSubDAQModule();

   //=========================================================================
   /** Returns true if a TCP connection with the DAQ module is open */
   bool IsConnected(void) const;

   //=========================================================================
   /** Starts the DAQ module recording (this has no effect if the module is
    *  currently recording data).
    *
    * @param VehicleID
    * 	ID of the vehicle making the recording
    *
    * @param MissionID
    * 	ID of the mission associated with this recording.
    *
    * @param RunID
    * 	ID of the run number in the mission being recorded.
    *
    * @param NudgeID
    * 	Maintained for legacy compatibility - set to zero.
    *
    * @return
    * 	true if the record command was sent to the DAQ module; else false if
    * 	the DAQ module is not connected.
    */
   bool StartRecording(int16_t VehicleID, int16_t MissionID, int16_t RunID,
                       int16_t NudgeID);

   //=========================================================================
   /** Stops a DAQ module recording (this has no effect if a recording is not
    *  in progress.
    *
    * @return
    * 	true if a stop command was sent to the DAQ module; else false if the
    * 	DAQ module is not connected.
    */
   bool StopRecording(void);

   //=========================================================================
   /** Sends a status query to the DAQ module */
   bool QueryEngineStatus(void);

   /** Returns the DAQ's reported status after QueryEngineStatus() has been
    *  called
    */
   int GetEngineStatus(void) const
   { return m_DAQStatus; }

   //=========================================================================
   /** Sends telemetry information to the DAQ module.
    *
    * @details
    * 	If a DAQ module recording is in progress, telemetry information will be
    * 	interleaved with sensor data in the recorded file.
    *
    * @param North
    * 	Vehicle North coordinate in local coordinate system
    *
    * @param East
    * 	Vehicle East coordinate in local coordinate system
    *
    * @param Depth
    * 	Vehicle depth in meters
    *
    * @param Roll
    * 	Vehicle roll in radians bounded to +/- PI
    *
    * @param Pitch
    *  Vehicle pitch in radians bounded to +/- PI
    *
    * @param Heading
    * 	Vehicle heading in degrees
    */
   bool SendTelemetry(float North, float East, float Depth, float Roll,
                      float Pitch, float Heading);

   //========================================
   // BunnySockListener interface functions
   //========================================

   //=========================================================================
   /** @internal */
   void OnPacketReceived(BunnySock::BunnySockPacket& RxPacket,
         BunnySock::BunnySockNode& Node, double TimeStamp_sec);

   //=========================================================================
   /** @internal */
   void OnConnectionEvent(BunnySockListener::ConnectionEventId EventId,
                          BunnySock::BunnySockNode& Node, double TimeStamp_sec);

   //=========================================================================
   /** @enum e_dre_States
    * @brief    States of the DAQ module's recording engine
    */
   enum e_dre_States
   {
      NOT_CONNECTED = -3, /**< The DAQ module is not connected */
      STALLED = -2, /**< Record engine is stalled in an error state */
      NO_DISK = -1, /**< No disk present to record to or disk not ready */

      IDLE, /**< Engine is idle (ADC data is being received but not
       written to disk) */
      PREPARING, /**< Engine is preparing to record to a new file on the
       disk */
      RECORDING, /**< Engine is writing sampled data from the ADC's to
       disk */
      STOPPING /**< Engine is stopping the recording process and
       closing the current file */
   };

private:
   int m_Verbosity; /**< Verbosity used for debugging messages */
   BunnySock::BunnySockTcpNode* m_pNode; /**< BunnySock connection to DSP */

   int m_DAQStatus; /**< Status reported by the DAQ module */

   //=========================================================================
   /** Helper function used to handle received DSP query/status packets */
   void HandleStatusPacket(CommandPacket_t* pPacket);

   //=========================================================================
   /** Helper function that returns the number of milliseconds elapsed since
    *  midnight of the current day */
   uint32_t GetMillisecondsSinceMidnight(void) const;

   //=========================================================================
   /** Helper function to send a command packet to the DAQ module
    *
    * @param CommandID
    * 	ID of the command to send
    *
    * @param pParamValues
    *	Pointer to an array of command parameter values corresponding to
    *	command parameters 0...n
    *
    * @param NumParams
    * 	The number of command parameters pParamValues points to
    *
    * @return
    * 	true if the command was sent; else false if the DAQ module is not
    * 	connected.
    */
   bool SendDAQCommand(int16_t CommandID, int16_t* pParamValues,
                       uint32_t NumParams);
};

#endif	// END #ifndef _YELLOWSUBDAQMODULE_H_
