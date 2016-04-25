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
#include "BunnySockUdpNode.h"
#include "LibBunnySock.h"
#include "FileSelector.h"
#include "PrecisionTime.h"
#include <MOOS/libMOOS/Utils/MOOSUtilityFunctions.h>

#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <sstream>
#include <cassert>

using namespace::std;
using namespace::BunnySock;
using namespace BunnySock::Sockets;
using YellowSubUtils::TimedLock;
using YellowSubUtils::PrecisionTime;


// Maximum time (milliseconds) that UDP sockets will block when receiving data
static int32_t const UDP_RX_POLL_PERIOD_MS = 100;

static const std::string BUNNYSOCK_UDP_NODE_STRING("<BunnySockUdpNode>");
static int const INVALID_SOCKET_FD = -1;


//=============================================================================
/** A file scope function used to start a BunnySock TCP node object's worker
 * thread.
 *
 * @param pObject
 *   Pointer to the target TCP node object
 */
static bool BunnySockUdpThreadLauncher( void* pObject )
{
   reinterpret_cast<BunnySockUdpNode*>(pObject)->WorkerThreadBody();
   return true;
}

//=============================================================================
BunnySockUdpNode::BunnySockUdpNode( uint16_t rx_port, uint16_t tx_port,
                                    uint16_t device_id,
                                    uint16_t verbosity )
: BunnySockNode(),
  m_rx_port(rx_port),
  m_tx_port(tx_port),
  m_TxLock(false),
  m_WorkerThread()
{
   // Create a UDP socket that will send broadcast datagrams and receive
   // from any IP address
   Sockets::Socket_address address( Socket_address::IPv4_ANY_ADDRESS, rx_port );
   m_udp_socket.reset( new UDP_broadcast_socket(address) );
   m_IsConnected = true;

   // Initialize the node's worker thread
   m_WorkerThread.Initialise(BunnySockUdpThreadLauncher, this);

   m_Verbosity = verbosity;
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
            m_udp_socket->send( m_tx_port,
                        reinterpret_cast<uint8_t*>(PacketToSend.GetRawBytes()),
                        BUNNYSOCK_PACKET_SIZE );
         }
         catch (Socket_exception& e)
         {
            BSOCK_VERBOSE1( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                                      " failed to send data on UDP socket\n") );
            PacketWasSent = false;
         }

         m_TxLock.UnLock();   // Release the Tx lock
      }
      else
      {
         BSOCK_VERBOSE1( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                                   " Timed out on m_TxLock while attempting to "
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
void BunnySockUdpNode::Start( void )
{
   m_WorkerThread.Start();
}





//=============================================================================
void BunnySockUdpNode::WorkerThreadBody( void )
{
   FileSelector Selector;   // Used to receive data
   BunnySockPacket RxPacket;      // Buffer for receiving packets
   int Flags;         // Used to calculate blocking time
   Socket_address sender_address(Socket_address::IPv4_ANY_ADDRESS,
                                 Socket_address::ANY_PORT);

   IgnoreSigPipe();   // Ignore broken pipe signals under UNIX

   // Outer loop:
   // This executes as long as the BunnySockUdpNode object is running
   while ( !m_WorkerThread.IsQuitRequested() )
   {

      //--------------------------------------------
      // Reset connection variables and packet times.
      RxPacket.Clear();   // Clear the Rx buffer

      // Set up a file selector to block for received packets
      Selector.SetFile(m_udp_socket->file_descriptor(), FileSelector::Readable);

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
            BSOCK_VERBOSE2( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                            "Failed to select() on socket!\n"));
            break;
         }

         //--------------------------------------------------
         // Read data from the socket
         //--------------------------------------------------
         if (Flags & FileSelector::Readable)
         {
            int numbytes = 0;
            try
            {
               numbytes = m_udp_socket->recvfrom(
                           reinterpret_cast<uint8_t*>( RxPacket.GetRawBytes() ),
                           BUNNYSOCK_PACKET_SIZE, sender_address );
            }
            catch (Socket_exception& e)
            {
               BSOCK_VERBOSE2( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                               "recvfrom() failed on UDP socket: " +
                               e.what() ));
                break;
            }

            // If a full packet has been received, process it
            assert(numbytes == BUNNYSOCK_PACKET_SIZE);

            PrecisionTime LastRxTime = PrecisionTime::Now(true);   // Get Rx time

            if ( m_Verbosity >= 2 )
            {
               MOOSTrace( BUNNYSOCK_UDP_NODE_STRING +
                     MOOSFormat( " Received a UDP packet from %s\n",
                              sender_address.as_string().c_str() ) );

            }

            // Report the received packet to listeners
            BunnySockListener* pListener;
            for (list<BunnySockListener*>::iterator iter = m_ListenerList.begin();
                iter != m_ListenerList.end(); iter++)
            {
               pListener = *iter;
               pListener->OnPacketReceived(RxPacket, *this,
                                           LastRxTime.AsDouble());
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

   }   // END while ( !m_WorkerThread.IsQuitRequested() )

   BSOCK_VERBOSE1( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                             " Worker thread exiting.\n") );
}

