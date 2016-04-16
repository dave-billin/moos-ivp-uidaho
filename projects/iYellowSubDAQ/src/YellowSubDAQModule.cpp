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
/** @file YellowSubDAQModule.cpp
 *
 * @brief
 *  Implementation of the YellowSubDAQModule object
 *
 * @author Dave Billin
 */
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "LibBunnySock.h"
#include "YellowSubDAQModule.h"

using namespace std;
using namespace YellowSubUtils;
using namespace BunnySock;

//====================================
// Local scope constants and macros
//====================================
#define LOCAL_DEVICEID		2	// Send packets using KIRK's device ID
#define DAQ_DEVICEID		5	// BunnySock ID of the DAQ module

// Macros to implement verbosity-dependent conditional statements
#define VERBOSE1(_expr_)	if (m_Verbosity >= 1) { _expr_; }
#define VERBOSE2(_expr_)	if (m_Verbosity >= 2) { _expr_; }
#define VERBOSE3(_expr_)	if (m_Verbosity >= 3) { _expr_; }


//=============================================================================
YellowSubDAQModule::YellowSubDAQModule(string& sHostName,
                                       uint16_t Port,
                                       uint32_t ConnectionTimeoutMs,
                                       int Verbosity,
                                       int BunnySockVerbosity)
  : m_Verbosity(Verbosity), m_pNode(NULL), m_DAQStatus(NOT_CONNECTED)
{
   BunnySockTcpNode* pTcpNode;

   pTcpNode = new BunnySockTcpNode(BunnySockTcpNode::CLIENT, /* mode */
   sHostName, /* Remote host */
   Port, /* Remote Port */
   LOCAL_DEVICEID,/* Device ID to send as */
   1, /* Retry period (sec) */
   ConnectionTimeoutMs, /* Connection
    timeout (ms) */
   BunnySockVerbosity); /* Verbosity */

   // Verify that we created a BunnySock node
   if (pTcpNode == NULL)
   {
      string s = "YellowSubDAQModule: Failed to create a BunnySock TCP "
            "client node for connecting to " + sHostName
            + MOOSFormat(":%d", Port) + "\n";
      throw CMOOSException(s);
   }

   m_pNode = pTcpNode;
   m_pNode->AddListener(*this);	// Register for BunnySock packets and events
   m_pNode->Start();	// Start the BunnySock connection's network thread
}

//=============================================================================
YellowSubDAQModule::~YellowSubDAQModule()
{
   if (m_pNode != NULL)
   {
      delete m_pNode;
   }
}

//=============================================================================
bool YellowSubDAQModule::IsConnected(void) const
{
   return m_pNode->IsConnected();
}

//=============================================================================
bool YellowSubDAQModule::StartRecording(int16_t VehicleID, int16_t MissionID,
                                        int16_t RunID, int16_t NudgeID)
{
   int16_t ParamValues[4];
   ParamValues[0] = VehicleID;
   ParamValues[1] = MissionID;
   ParamValues[2] = RunID;
   ParamValues[3] = NudgeID;

   bool Rc = SendDAQCommand(CMD_START_RECORDER, ParamValues, 4);
   if (Rc && (m_Verbosity > 1))
   {
      MOOSTrace("Sent command to start recording\n");
   }

   return Rc;
}

//=============================================================================
bool YellowSubDAQModule::StopRecording(void)
{
   bool Rc = SendDAQCommand(CMD_STOP_RECORDER, NULL, 0);

   if (Rc && (m_Verbosity > 1))
   {
      MOOSTrace("Sent command to stop recording\n");
   }

   return Rc;
}

//=============================================================================
bool YellowSubDAQModule::QueryEngineStatus(void)
{
   bool Rc = SendDAQCommand(CMD_QUERY_RECORDER, NULL, 0);

   if (Rc && (m_Verbosity > 3))
   {
      MOOSTrace("Sent command to query DAQ engine status\n");
   }

   return Rc;
}

//=============================================================================
bool YellowSubDAQModule::SendDAQCommand(int16_t CommandID,
                                        int16_t* pParamValues, uint32_t NumParams)
{
   BunnySockPacket CommandPacket;
   CommandPacket_t* pPacket;

   pPacket = reinterpret_cast<CommandPacket_t*>(CommandPacket.GetRawBytes());

   CommandPacket.SetHeader(TYPE_COMMAND, LOCAL_DEVICEID, DAQ_DEVICEID,
         GetMillisecondsSinceMidnight());

   pPacket->CommandId = CommandID;

   int16_t* pParam = pPacket->Parameter;
   // Copy parameter values into the packet
   for (uint32_t i = 0; i < NumParams; i++)
   {
      *pParam++ = *pParamValues++;
   }

   bool Rc = m_pNode->SendPacket(CommandPacket);

   if (Rc == false)
   {
      MOOSTrace("Failed to send command to DAQ module!\n");
   }

   return Rc;
}

//=============================================================================
bool YellowSubDAQModule::SendTelemetry(float North, float East, float Depth,
                                       float Roll, float Pitch, float Heading)

{
   BunnySockPacket TelemetryPacket;

   TelemetryPacket.SetHeader(TYPE_TELEMETRY, LOCAL_DEVICEID, DAQ_DEVICEID,
         GetMillisecondsSinceMidnight());

   DspTelemetryPacket_t* pPacket =
         reinterpret_cast<DspTelemetryPacket_t*>(TelemetryPacket.GetRawBytes());

   pPacket->EKF_north_pos = North;
   pPacket->EKF_east_pos = East;
   pPacket->depth_cm = Depth * 100;
   pPacket->Pitch_deg = static_cast<float>(MOOSRad2Deg(Pitch));
   pPacket->Roll_deg = static_cast<float>(MOOSRad2Deg(Roll));
   pPacket->CompassHeading_deg = Heading;

   bool Rc = m_pNode->SendPacket(TelemetryPacket);

   return Rc;
}

//=============================================================================
void YellowSubDAQModule::OnPacketReceived(BunnySockPacket& RxPacket,
                                          BunnySockNode& Node,
                                          double TimeStamp_sec)
{
   CommandPacket_t* pPacket;

   if (RxPacket.GetHeader()->PacketType == TYPE_COMMAND)
   {
      pPacket = static_cast<CommandPacket_t*>(RxPacket.GetRawBytes());

      switch (pPacket->CommandId)
      {
         MOOSTrace("Received status packet from DAQ\n");

         // Handle a status report from the DSP
      case CMD_QUERY_RECORDER:
      {
         HandleStatusPacket(pPacket);
         break;
      }

      default:
         return;
      }
   }
}

//=============================================================================
void YellowSubDAQModule::OnConnectionEvent(
                                 BunnySockListener::ConnectionEventId EventId,
                                 BunnySockNode& Node,
                                 double TimeStamp_sec)
{
   string s;

   switch (EventId)
   {
      case BunnySockListener::CONNECTED:
         s = "Connected to the DAQ module\n";
         QueryEngineStatus();	// Send a status query to the module
         break;

         // If SCOTTY disconnects, reset all reported variable values
      case BunnySockListener::CONNECTION_TIMEOUT:
         s = "Connection to the DAQ module timed out!\n";
         m_DAQStatus = NOT_CONNECTED;
         break;

      case BunnySockListener::CONNECTION_ERROR:
         s = "An error occurred on the connection to the DAQ module!\n";
         m_DAQStatus = NOT_CONNECTED;
         break;

      case BunnySockListener::DISCONNECTED:
         s = "Connection to the DAQ module closed.\n";
         m_DAQStatus = NOT_CONNECTED;
         break;

      default:
         return;
   }

   MOOSTrace(s);	// Print the event
}

//=============================================================================
void YellowSubDAQModule::HandleStatusPacket(CommandPacket_t* pPacket)
{
   if (pPacket)
   {
      // Grab the status of the DAQ record engine
      m_DAQStatus = pPacket->Parameter[4];
   }

   if (m_Verbosity >= 2)
   {
      const char* szDreStates[] = { "Not Connected", "Stalled", "No Disk",
            "Idle", "Preparing", "Recording", "Stopping" };

      string s;

      if ((m_DAQStatus >= YellowSubDAQModule::NOT_CONNECTED)
            && (m_DAQStatus <= YellowSubDAQModule::STOPPING))
      {
         s = szDreStates[m_DAQStatus + 3];
      }
      else
      {
         s = "Unknown";
      }

      MOOSTrace("DAQ engine State: " + s + "\n");
   }
}

//=============================================================================
uint32_t YellowSubDAQModule::GetMillisecondsSinceMidnight(void) const
{
   PrecisionTimeInterval TimeSinceMidnight =
                              PrecisionTime::Midnight() - PrecisionTime(0, 0);
   return TimeSinceMidnight.As(PrecisionTimeInterval::MILLISECONDS);
}

