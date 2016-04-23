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
/** @file BunnySockTcpNode.cpp
 *
 * @brief
 *  Implementation of the BunnySockTcpNode class
 *
 * @author Dave Billin
 */
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "FileSelector.h"
#include "LibBunnySock.h"
#include "BunnySockTcpNode.h"

using namespace std;
using namespace YellowSubUtils;

namespace BunnySock
{

const string BunnySockTcpNode::sBunnySockTcpNode("<BunnySockTcpNode>");

#pragma pack(push, 1)

//-----------------------------------------------------------
/** @typedef bsockKeepAlivePacket_t
 * Structure of a keep-alive packet sent periodically between
 * BunnySock nodes followed by payload data
 */
struct bsockKeepAlivePacket_t
{
   bsockKeepAlivePacket_t(uint16_t DeviceID)
   {
      memset(Unused, 0,
      BUNNYSOCK_PACKET_SIZE - sizeof(bsock_PacketHeader_t) - sizeof(float));
      Header.DestId = PACKETTYPE_BUNNYSOCK_KEEPALIVE;
      Header.SourceId = DeviceID;
      SoftwareVersion = LIBBUNNYSOCK_VERSION;
   }
   bsock_PacketHeader_t Header; /**< The packet's header */
   float SoftwareVersion; /**< Sofware version of the packet's sender */
   char Unused[BUNNYSOCK_PACKET_SIZE - sizeof(bsock_PacketHeader_t)
         - sizeof(float)]; /**< Unused payload Bytes */
};

//=============================================================================
/** A file scope function used to start a BunnySock TCP node object's worker
 * thread.
 *
 * @param pObject
 *	Pointer to the target TCP node object
 */
static bool BunnySockTcpThreadLauncher(void* pObject)
{
   reinterpret_cast<BunnySockTcpNode*>(pObject)->WorkerThreadBody();
   return true;
}

#pragma pack(pop)

//=============================================================================
BunnySockTcpNode::BunnySockTcpNode(void) :
      BunnySockNode(), m_sPeerHostname("0.0.0.0"), m_Port(0), m_ConnectionMode(
            -1), m_RetryPeriodSec(2000), m_ConnectionTimeout_ms(
            BUNNYSOCK_DEFAULT_CONNECTION_TIMEOUT_MS), m_PeerDeviceId(0), m_TxLock(
            false), m_pSocket(NULL), m_WorkerThread()
{
   m_IsConnected = false;
}

//=============================================================================
BunnySockTcpNode::BunnySockTcpNode( int ConnectionMode,
                                    string const& sTargetHostName,
                                    uint16_t Port, uint16_t DeviceId,
                                    uint32_t RetryPeriodSec,
                                    uint32_t ConnectionTimeoutMs,
                                    uint16_t Verbosity )
  : BunnySockNode(),
    m_sPeerHostname(sTargetHostName),
    m_Port(Port),
    m_ConnectionMode(ConnectionMode),
    m_RetryPeriodSec(RetryPeriodSec),
    m_ConnectionTimeout_ms(ConnectionTimeoutMs),
    m_PeerDeviceId(0),
    m_TxLock(false),
    m_pSocket(NULL),
    m_WorkerThread()
{
   m_IsConnected = false;
   bsockKeepAlivePacket_t* pKeepAlivePacketData;

   // Construct a keep-alive packet
   pKeepAlivePacketData =
         (bsockKeepAlivePacket_t*) m_KeepAlivePacket.GetRawBytes();
   pKeepAlivePacketData->Header.PacketType = PACKETTYPE_BUNNYSOCK_KEEPALIVE;
   pKeepAlivePacketData->Header.SourceId = m_DeviceId;
   pKeepAlivePacketData->SoftwareVersion =

   // Initialize and start the node's worker thread
         m_WorkerThread.Initialise(BunnySockTcpThreadLauncher, this);
   //m_WorkerThread.Start();
}

//=============================================================================
BunnySockTcpNode::~BunnySockTcpNode()
{
   m_WorkerThread.Stop();	// Close peer connection and stop the worker thread
}

//=============================================================================
void BunnySockTcpNode::ResetConnection(void)
{
   if (m_IsConnected)
   {
      // Restart the worker thread.  This will disconnect the node then
      // start it up again in the assigned connection mode
      m_WorkerThread.Stop();
      m_WorkerThread.Start();
   }
}

//=============================================================================
bool BunnySockTcpNode::SendPacket(BunnySockPacket& PacketToSend)
{
   bool PacketWasSent = true;

   // If the node is not connected, do nothing
   if (m_IsConnected)
   {
      // Lock the socket for writing to prevent re-entrancy of this function
      if (m_TxLock.Lock(100))
      {
         try
         {
            m_pSocket->iSendMessage(PacketToSend.GetRawBytes(),
                                    BUNNYSOCK_PACKET_SIZE);
         }
         catch (XPCException& e)
         {
            BSOCK_VERBOSE1(
                  MOOSTrace(
                        sBunnySockTcpNode + string(" failed to send packet: ")
                              + e.sGetException() + "\n"));
            PacketWasSent = false;
         }

         m_TxLock.UnLock();	// Release the Tx lock
      }
      else
      {
         BSOCK_VERBOSE1(
               MOOSTrace(
                     sBunnySockTcpNode
                           + " Timed out on m_TxLock while attempting to "
                                 "send a packet!\n"));
         PacketWasSent = false;
      }
   }
   else
   {
      PacketWasSent = false;
   }

   // On failure to send the packet, trigger a connection reset
   if (!PacketWasSent)
   {
      m_IsConnected = false;
   }
   else
   {
      m_LastTxTime.GetSystemTime(true);	// Register last Tx time
   }

   return PacketWasSent;
}

//=============================================================================
int BunnySockTcpNode::GetConnectionMode(void) const
{
   return m_ConnectionMode;
}

//=============================================================================
std::string BunnySockTcpNode::GetRemoteHostName(void) const
{
   return (m_IsConnected) ? m_sPeerHostname : string("");
}

//=============================================================================
int BunnySockTcpNode::GetPort(void) const
{
   return m_Port;
}

//=============================================================================
uint16_t BunnySockTcpNode::GetPeerDeviceId(void) const
{
   return (m_IsConnected) ? m_PeerDeviceId : 0;
}

//=============================================================================
void BunnySockTcpNode::Start(void)
{
   m_WorkerThread.Start();
}

//=============================================================================
void BunnySockTcpNode::SetConnectionTimeoutMs(uint32_t TimeoutMs)
{
   m_ConnectionTimeout_ms = TimeoutMs;
}

//=============================================================================
uint32_t BunnySockTcpNode::GetConnectionTimeoutMs(void) const
{
   return m_ConnectionTimeout_ms;
}

//=============================================================================
bool BunnySockTcpNode::OpenClientConnection(void)
{
   double TimeRef, ElapsedSeconds;		// Used to time connection retries
   bool Connected = false;

   // Spawn a new TCP socket
   m_pSocket = new XPTcpSocketEx(m_sPeerHostname, m_Port);

   while (!m_WorkerThread.IsQuitRequested())
   {
      //-------------------------------------------------------
      // Try to open a connection to the target peer
      TimeRef = MOOSTime(false);	// Grab the current time
      Connected = true;
      try
      {
         m_pSocket->vConnect(m_sPeerHostname.c_str());
      }
      catch (XPCException& e)
      {
         // On failure to connect to peer, print an error message and return false
         BSOCK_VERBOSE3(
               MOOSTrace(
                     sBunnySockTcpNode + " Failed to connect to "
                           + m_sPeerHostname + ":\n  " + e.sGetException()
                           + "\n"));

         Connected = false;
      }
      catch (std::exception& se)
      {
         // On failure to connect to peer, print an error message and return false
         BSOCK_VERBOSE3(
               MOOSTrace(
                     sBunnySockTcpNode + " Failed to connect to "
                           + m_sPeerHostname + ".  " + string(se.what())
                           + "\n"));
         Connected = false;
      }

      //---------------------------------------------------
      // If a connection was opened, break out of the loop
      // and return a pointer to the connected socket
      if (Connected)
      {
         break;		// Break out of the connection loop
      }

      //---------------------------------------------------
      // STATUS: We failed to open a connection
      else
      {
         // Pause before attempting another connection to
         // enforce the node's specified retry time. Check
         // whether the thread has been asked to quit
         while (!m_WorkerThread.IsQuitRequested())
         {
            ElapsedSeconds = MOOSTime(false) - TimeRef;

            //TODO: Need to sleep here - not spin on time comparison!

            if (ElapsedSeconds > m_RetryPeriodSec)
            {
               break;
            }
         }
      }

   }

   return Connected;
}

//=============================================================================
bool BunnySockTcpNode::OpenServerConnection(void)
{
   bool Connected;
   string sError;
   char szConnectedHostName[16] = "               ";
   XPTcpSocketEx* pBreederSocket = NULL;

   // Bind the socket to the specified remote host and port
   // and start listening for incoming connections
   // DB Note: the MOOS XPCSocket is *really* dumb, and can fail to
   // generate a TCP socket under Windows when getprotobyname()
   // fails under certain (weird) circumstances.
   try
   {
      //pBreederSocket = new XPTcpSocketEx(m_sPeerHostname, m_Port);
      pBreederSocket = new XPTcpSocketEx("0.0.0.0", m_Port);
      pBreederSocket->vSetReuseAddr(1);
      pBreederSocket->vBindSocket();
      pBreederSocket->vListen(1);
   }
   catch (XPCException& e)
   {
      MOOSTrace(sBunnySockTcpNode + " " + e.sGetException() + "\n");
      if (pBreederSocket != NULL)
      {
         pBreederSocket->vCloseSocket();
      }
      return false;
   }

   //--------------------------------------------------
   // Wait for an incoming connection
   while (!m_WorkerThread.IsQuitRequested())
   {
      Connected = true;

      try
      {
         m_pSocket = pBreederSocket->Accept(szConnectedHostName);
      }
      catch (XPCException& e)
      {
         // If Accept() fails, print an error and return NULL
         BSOCK_VERBOSE1(
               sError = sBunnySockTcpNode + " Error listening for connection: "
                     + e.sGetException() + "\n");

         Connected = false;

      }
      catch (std::exception& se)
      {
         // On failure to connect to peer, print an error message and return false
         BSOCK_VERBOSE1(
               sError = sBunnySockTcpNode + " Error listening for connection: "
                     + string(se.what()) + "\n");
         Connected = false;
      }

      // If an incoming connection was opened, notify registered listeners
      // and return a pointer to the connected socket.
      if (Connected)
      {
         m_sPeerHostname = szConnectedHostName;
         break;	// Break out of the connection loop
      }
      else
      {
         MOOSTrace(sError);
      }

   }

   // Close and release the parent socket
   if (pBreederSocket != NULL)
   {
      pBreederSocket->vCloseSocket();
      delete pBreederSocket;
   }
   return Connected;
}

//=============================================================================
void BunnySockTcpNode::WorkerThreadBody(void)
{
   FileSelector Selector;	// Used to receive data
   BunnySockPacket RxPacket;		// Buffer for receiving packets
   char* pRxPacketBytes;			// Pointer to Rx Buffer
   uint32_t NumBytesSoFar, NumBytesThisTime;
   bsockKeepAlivePacket_t KeepAlivePacketData(m_DeviceId);
   BunnySockPacket KeepAlivePacket(&KeepAlivePacketData, BUNNYSOCK_PACKET_SIZE);
   PrecisionTime RefTime;	// Current time (seconds)
   PrecisionTimeInterval TimeoutDelta(m_ConnectionTimeout_ms,
         PrecisionTimeInterval::MILLISECONDS);
   uint32_t Elapsed_ms;
   bool PacketWasSent;
   IgnoreSigPipe();	// Ignore broken pipe signals under UNIX

   // Initialize keep-alive packet data
   KeepAlivePacketData.Header.SourceId = m_DeviceId;
   memcpy(KeepAlivePacket.GetRawBytes(), &KeepAlivePacketData,
         BUNNYSOCK_PACKET_SIZE);

   // Outer loop:
   // This executes as long as the BunnySockTcpNode object is running
   while (!m_WorkerThread.IsQuitRequested())
   {
      //--------------------------------------------
      // Wait for a connection with a remote peer
      switch (m_ConnectionMode)
      {
         case BunnySockTcpNode::CLIENT:
            m_IsConnected = OpenClientConnection();
            break;

         case BunnySockTcpNode::SERVER:
            m_IsConnected = OpenServerConnection();
            break;
      }

      //--------------------------------------------
      // Verify and signal that a connection was opened
      if (!m_IsConnected)
      {
         MOOSPause(m_RetryPeriodSec * 1000, false);
         continue;
      }
      else
      {
         ReportConnectionEvent(BunnySockListener::CONNECTED);
      }

      //--------------------------------------------
      // Reset connection variables and packet times.
      m_LastRxTime.GetSystemTime(true);   // Use CLOCK_MONOTONIC
      m_LastTxTime.Zero();	// Trigger sending an initial keep-alive packet
      RxPacket.Clear();	// Clear the Rx buffer
      pRxPacketBytes = (char*) RxPacket.GetRawBytes();
      NumBytesSoFar = 0;	// Reset the Rx Byte count

      // Set up a file selector to block for received packets
      Selector.SetFile(m_pSocket->iGetSocketFd(), FileSelector::Readable);

      //------------------------------------------------------------------
      // Inner loop - we do these things as long as the node is connected
      // and the thread has not been asked to quit
      //------------------------------------------------------------------
      while (m_IsConnected && !m_WorkerThread.IsQuitRequested())
      {
         try
         {
            RefTime.GetSystemTime(true);
         }    // Grab current time
         catch (std::exception& e)
         {
            MOOSTrace("%s %s\n", sBunnySockTcpNode.c_str(), e.what());
            break;
         }

         //---------------------------------------------------
         // Has the node's connection timed out?
         Elapsed_ms = m_LastRxTime.ElapsedTime().As(
               PrecisionTimeInterval::MILLISECONDS);
         if (Elapsed_ms > m_ConnectionTimeout_ms)
         {
            m_IsConnected = false;
            ReportConnectionEvent(BunnySockListener::CONNECTION_TIMEOUT);
            BSOCK_VERBOSE4(MOOSTrace("%s Connection timed out after %u"
                  " ms\n", sBunnySockTcpNode.c_str(), Elapsed_ms));
            break;
         }

         //---------------------------------------------------
         // Send a keep-alive packet if 50% of the connection
         // timeout elapses without sending a packet
         //---------------------------------------------------
         uint32_t SendThreshold_ms = (m_ConnectionTimeout_ms >> 1);
         Elapsed_ms = m_LastTxTime.ElapsedTime().As(
               PrecisionTimeInterval::MILLISECONDS);
         if (Elapsed_ms > SendThreshold_ms)
         {
            PacketWasSent = SendPacket(KeepAlivePacket);

            try
            {
               RefTime.GetSystemTime(true);
            }
            catch (std::exception& e)
            {
               MOOSTrace("%s %s\n", sBunnySockTcpNode.c_str(), e.what());
               break;
            }

            if (m_Verbosity >= 4)
            {
               const char* szStatus = (
                     (PacketWasSent) ? "Sent" : "Failed to send");

               MOOSTrace("%s %s a keep-alive packet after %u ms\n",
                     sBunnySockTcpNode.c_str(), szStatus, Elapsed_ms);
            }
         }

         //--------------------------------------------------
         // Block until data is received or until it is time
         // to send a keep-alive packet.
         //--------------------------------------------------
         Elapsed_ms = m_LastTxTime.ElapsedTime().As(
               PrecisionTimeInterval::MILLISECONDS);
         uint32_t MsToBlock =
               (Elapsed_ms < SendThreshold_ms) ? (SendThreshold_ms - Elapsed_ms) :
                     0;
         int Flags = Selector.WaitForReadiness(FileSelector::Readable,
               static_cast<long>(MsToBlock));
         if (Flags < 0)
         {
            // If select() fails, we assume the socket is invalid, and exit
            // the inner loop to drop the peer connection
            BSOCK_VERBOSE2(
                  MOOSTrace(
                        sBunnySockTcpNode + "Failed to select() on socket!\n"));
            break;
         }

         //--------------------------------------------------
         // Read data from the socket
         //--------------------------------------------------
         if (Flags & FileSelector::Readable)
         {
            try
            {
               // Read data from the socket.  If the connection is reset, zero is returned.
               NumBytesThisTime = m_pSocket->iRecieveMessage(pRxPacketBytes,
                     (BUNNYSOCK_PACKET_SIZE - NumBytesSoFar), 0);
            }
            catch (XPCException& e)
            {
               BSOCK_VERBOSE1(
                     MOOSTrace(
                           sBunnySockTcpNode + " Error reading from socket: "
                                 + e.sGetException() + "\n"));
               NumBytesThisTime = 0;
            }

            // Handle a connection reset or error
            if (NumBytesThisTime == 0)
            {
               ReportConnectionEvent(BunnySockListener::CONNECTION_ERROR);
               m_IsConnected = false;
               break;	// Break out of the inner loop to restart the
               // connection
            }

            //---------------------------------------------
            // STATUS:
            //	data was read from the socket successfully
            //---------------------------------------------
            pRxPacketBytes += NumBytesThisTime;
            NumBytesSoFar += NumBytesThisTime;

            // If a full packet has been received, process it
            if (NumBytesSoFar == BUNNYSOCK_PACKET_SIZE)
            {
               m_LastRxTime.GetSystemTime(true);	// Update Last Rx Time

               // If the packet is a keep-alive packet, just register the
               // peer's device ID
               if (RxPacket.GetHeader()->PacketType
                     == PACKETTYPE_BUNNYSOCK_KEEPALIVE)
               {
                  m_PeerDeviceId = RxPacket.GetHeader()->SourceId;
                  BSOCK_VERBOSE4(
                        MOOSTrace(
                              sBunnySockTcpNode
                                    + MOOSFormat(
                                          " Received a keep-alive packet from "
                                                "device %d\n", m_PeerDeviceId)));
               }
               else
               {
                  BSOCK_VERBOSE2(
                        MOOSTrace(sBunnySockTcpNode + " Received a packet\n"));
                  // Report the received packet to listeners
                  BunnySockListener* pListener;
                  for (list<BunnySockListener*>::iterator iter =
                        m_ListenerList.begin(); iter != m_ListenerList.end();
                        iter++)
                  {
                     pListener = *iter;
                     pListener->OnPacketReceived(RxPacket, *this,
                           m_LastRxTime.AsDouble());
                  }
               }

               // Reset pointer and counter
               pRxPacketBytes = (char*) RxPacket.GetRawBytes();
               NumBytesSoFar = 0;
            }
         }	// END if (Selector.FileIsReadable())

      }

      //------------------------------------------------
      // STATUS:
      //	The node is disconnecting and/or the worker thread has
      //	been asked to exit.
      //------------------------------------------------

      // Notify all listeners that the node is disconnected
      m_IsConnected = false;
      ReportConnectionEvent(BunnySockListener::DISCONNECTED);

      // Close and delete the socket
      if (m_pSocket != NULL)
      {
         //&
         BSOCK_VERBOSE2(MOOSTrace(sBunnySockTcpNode + " Closing TCP socket\n"));
         m_pSocket->vCloseSocket();
         delete m_pSocket;
         m_pSocket = NULL;
      }

   }	// END while ( !m_WorkerThread.IsQuitRequested() )

   BSOCK_VERBOSE1(MOOSTrace(sBunnySockTcpNode + " Worker thread exiting.\n"));
}

}
;
// END namespace BunnySock
