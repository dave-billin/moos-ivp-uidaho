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
/** @file BunnySockUdpNode.cpp
 *
 * @brief
 *  Implementation of the BunnySockUdpNode class
 *
 * @author Dave Billin
 */
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "FileSelector.h"
#include "BunnySockUdpNode.h"
#include "LibBunnySock.h"

using namespace::std;
using namespace::BunnySock;
using YellowSubUtils::TimedLock;


/** @def UDP_RX_POLL_PERIOD_MS
 * @brief
 *    Sets the maximum time in milliseconds for which UDP sockets will block
 *    when receiving data.
 */
#define UDP_RX_POLL_PERIOD_MS   100

const string BunnySockUdpNode::sBunnySockUdpNode("<BunnySockUdpNode>");


//=============================================================================
/** A file scope function used to start a BunnySock TCP node object's worker
 * thread.
 *
 * @param pObject
 *   Pointer to the target TCP node object
 */
static bool BunnySockUdpThreadLauncher( void* pObject )
{
   BunnySockUdpNode* pObjectInstance = (BunnySockUdpNode*)pObject;
   pObjectInstance->WorkerThreadBody();
   return true;
}







//=============================================================================
BunnySockUdpNode::BunnySockUdpNode( void )
: BunnySockNode(NULL, 0, 0),
  m_Port(0),
  m_TxLock(false),
  m_pSocket(NULL),
  m_WorkerThread()
{
   m_IsConnected = false;
}



//=============================================================================
BunnySockUdpNode::BunnySockUdpNode( uint16_t Port, uint16_t DeviceId,
                           uint16_t Verbosity )
: BunnySockNode(NULL, DeviceId, Verbosity),
  m_Port(Port),
  m_TxLock(false),
  m_pSocket(NULL),
  m_WorkerThread()
{
   m_IsConnected = false;

   // Initialize and start the node's worker thread
   m_WorkerThread.Initialise(BunnySockUdpThreadLauncher, this);
}




//=============================================================================
BunnySockUdpNode::~BunnySockUdpNode()
{
   m_WorkerThread.Stop();   // Close peer connection and stop the worker thread
}




//=============================================================================
void BunnySockUdpNode::ResetConnection( void )
{
}




//=============================================================================
bool BunnySockUdpNode::SendPacket( BunnySockPacket& PacketToSend )
{
   bool PacketWasSent = true;

   // If the node is not connected, do nothing
   if (m_IsConnected)
   {
      // Lock the socket for writing to prevent re-entrancy of this function
      if ( m_TxLock.Lock(100) )
      {
         try
         {
            m_pSocket->iBroadCastMessage(PacketToSend.GetRawBytes(), BUNNYSOCK_PACKET_SIZE, m_Port);
            BSOCK_VERBOSE2( MOOSTrace( sBunnySockUdpNode + string(" Sent a packet via UDP broadcast\n") ) );
            BSOCK_VERBOSE3( MOOSTrace( sBunnySockUdpNode + PacketToSend.ToHexString() + "\n") );
         }
         catch (XPCException& e)
         {
            BSOCK_VERBOSE1( MOOSTrace( sBunnySockUdpNode + string(" failed to send packet: ") +
                            e.sGetException() + "\n") );
            PacketWasSent = false;
         }

         m_TxLock.UnLock();   // Release the Tx lock
      }
      else
      {
         BSOCK_VERBOSE1( MOOSTrace(sBunnySockUdpNode + " Timed out on m_TxLock while attempting to "
                             "send a packet!\n") );
         PacketWasSent = false;
      }
   }
   else
   {
      PacketWasSent = false;
   }

   return PacketWasSent;
}



//=============================================================================
int BunnySockUdpNode::GetPort( void ) const
{
   return m_Port;
}


//=============================================================================
void BunnySockUdpNode::Start( void )
{
   m_WorkerThread.Start();
}





//=============================================================================
void BunnySockUdpNode::WorkerThreadBody( void )
{
   FileSelector Selector;   // Used to receive data
   BunnySockPacket RxPacket;      // Buffer for receiving packets
   char* pRxPacketBytes;         // Pointer to Rx Buffer
   uint32_t NumBytesSoFar, NumBytesThisTime;
   int Flags;         // Used to calculate blocking time
   double LastRxTime;

   IgnoreSigPipe();   // Ignore broken pipe signals under UNIX

   // Outer loop:
   // This executes as long as the BunnySockUdpNode object is running
   while ( !m_WorkerThread.IsQuitRequested() )
   {
      //---------------------------
      // Create a UDP socket
      //---------------------------
      m_pSocket = new XPCUdpSocket(m_Port);
      m_pSocket->vSetBroadcast(1);    // Enable UDP broadcast packets
      if ( 0 == m_pSocket->iGetBroadcast() )
      {
         MOOSTrace("XPCUdpSocket::vSetBroadcast() failed to set the broadcast option");
      }
      m_IsConnected = true;

      //--------------------------------------------
      // Reset connection variables and packet times.
      RxPacket.Clear();   // Clear the Rx buffer
      pRxPacketBytes = (char*)RxPacket.GetRawBytes();
      NumBytesSoFar = 0;   // Reset the Rx Byte count

      // Set up a file selector to block for received packets
      Selector.SetFile(m_pSocket->iGetSocketFd(), FileSelector::Readable);

      //------------------------------------------------------------------
      // Inner loop - we do these things as long as the node is connected
      // and the thread has not been asked to quit
      //------------------------------------------------------------------
      while ( m_IsConnected && !m_WorkerThread.IsQuitRequested() )
      {
         //--------------------------------------------------
         // Block until data is received.
         // Wake up every 100 ms to check for an exit request
         //--------------------------------------------------
         Flags = Selector.WaitForReadiness( FileSelector::Readable,
                                            UDP_RX_POLL_PERIOD_MS);
         if (Flags < 0)
         {
            // If select() fails, we assume the socket is invalid, and exit the
            // inner loop to drop the peer connection
            BSOCK_VERBOSE2( MOOSTrace(sBunnySockUdpNode +
                        "Failed to select() on socket!\n"));
            break;
         }

         //--------------------------------------------------
         // Read data from the socket
         //--------------------------------------------------
         if (Flags & FileSelector::Readable)
         {
            try
            {
               // Read data from the socket.
               NumBytesThisTime = m_pSocket->iRecieveMessage( pRxPacketBytes,
                                                   (BUNNYSOCK_PACKET_SIZE - NumBytesSoFar),
                                                   0 );
            }
            catch (XPCException& e)
            {
               BSOCK_VERBOSE1( MOOSTrace( sBunnySockUdpNode +
                                    " Error reading from socket: " +
                                    e.sGetException() + "\n") );
               NumBytesThisTime = 0;
            }

            // Handle a read error
            if (NumBytesThisTime == 0)
            {
               ReportConnectionEvent(BunnySockListener::CONNECTION_ERROR);
               m_IsConnected = false;
               break;   // Exit out of the inner loop to get a new socket
            }


            //---------------------------------------------
            // STATUS:
            //   data was read from the socket successfully
            //---------------------------------------------
            pRxPacketBytes += NumBytesThisTime;
            NumBytesSoFar += NumBytesThisTime;

            // If a full packet has been received, process it
            if (NumBytesSoFar == BUNNYSOCK_PACKET_SIZE)
            {
               LastRxTime = HPMOOSTime(false);   // Capture Rx time

               BSOCK_VERBOSE2( MOOSTrace( sBunnySockUdpNode +
                                    " Received a packet\n"));
               // Report the received packet to listeners
               BunnySockListener* pListener;
               for (list<BunnySockListener*>::iterator iter = m_ListenerList.begin();
                   iter != m_ListenerList.end(); iter++)
               {
                  pListener = *iter;
                  pListener->OnPacketReceived(RxPacket, *this, LastRxTime);
               }

               // Reset pointer and counter
               pRxPacketBytes = (char*)RxPacket.GetRawBytes();
               NumBytesSoFar = 0;
            }
         }   // END if (Selector.FileIsReadable())

      }

      //------------------------------------------------
      // STATUS:
      //   The node is disconnecting and/or the worker
      //   thread has been asked to exit.
      //------------------------------------------------

      // Notify all listeners that the node is disconnected
      m_IsConnected = false;
      ReportConnectionEvent(BunnySockListener::DISCONNECTED);

      // Close and delete the socket
      if (m_pSocket != NULL)
      {
         m_pSocket->vCloseSocket();
         delete m_pSocket;
         m_pSocket = NULL;
      }

   }   // END while ( !m_WorkerThread.IsQuitRequested() )

   BSOCK_VERBOSE1( MOOSTrace( sBunnySockUdpNode + " Worker thread exiting.\n") );
}

