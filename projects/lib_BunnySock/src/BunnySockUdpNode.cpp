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
using YellowSubUtils::TimedLock;


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
// get sockaddr, IPv4 or IPv6:
static void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


//=============================================================================
/** Helper function to open a UDP socket on a specified port
 *  @param port   Port to open a UDP socket on
 *  @return A UDP socket file descriptor
 */
static int initialize_udp_socket( uint16_t port )
{
   int sockfd = INVALID_SOCKET_FD;
   struct addrinfo hints, *servinfo, *p;
   std::ostringstream oss;
   oss << port;

   memset(&hints, 0, sizeof hints);
   hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
   hints.ai_socktype = SOCK_DGRAM;
   hints.ai_flags = AI_PASSIVE; // use my IP

   int result = getaddrinfo(NULL, oss.str().c_str(), &hints, &servinfo);
   if (0 != result)
   {
      MOOSTrace( BUNNYSOCK_UDP_NODE_STRING +
                 MOOSFormat(" getaddrinfo error: %s\n", gai_strerror(result))
               );

       return 1;
   }

   // loop through all the results and bind to the first we can
   for(p = servinfo; p != NULL; p = p->ai_next)
   {
      sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (INVALID_SOCKET_FD == sockfd)
      {
         MOOSTrace("Failed to create UDP socket");
         continue;
      }

      if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1)
      {
         close(sockfd);
         sockfd = INVALID_SOCKET_FD;
         MOOSTrace("Failed to bind UDP socket");
         continue;
      }

      break;
   }

   freeaddrinfo(servinfo);
   return sockfd;
}


//=============================================================================
BunnySockUdpNode::BunnySockUdpNode( uint16_t Port, uint16_t DeviceId,
                                    uint16_t Verbosity )
: BunnySockNode(),
  m_Port(Port),
  m_TxLock(false),
  m_sockfd(INVALID_SOCKET_FD),
  m_WorkerThread()
{
   m_IsConnected = false;

   // Initialize the node's worker thread
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
         struct sockaddr_in their_addr; // connector's address information
         their_addr.sin_family = AF_INET;     // host byte order
         their_addr.sin_port = htons(m_Port); // short, network byte order
         their_addr.sin_addr.s_addr = 0xffffffff;    // broadcast address
         memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

         int numbytes = sendto(m_sockfd,
                               PacketToSend.GetRawBytes(),
                               BUNNYSOCK_PACKET_SIZE, 0,
                               (struct sockaddr *)&their_addr,
                               sizeof their_addr);

         if (-1 == numbytes)
         {
            BSOCK_VERBOSE1( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                                      " sendto() failed on UDP socket\n") );
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
      m_sockfd = initialize_udp_socket(m_Port);

      // allow broadcast packets to be sent:
      {
         int broadcast = 1;
         if ( -1 == setsockopt(m_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
                               sizeof broadcast) )
         {
            BSOCK_VERBOSE2( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                            "setsockopt (SO_BROADCAST) failed for UDP socket!\n")
                          );
             break;
         }
      }
      m_IsConnected = true;

      //--------------------------------------------
      // Reset connection variables and packet times.
      RxPacket.Clear();   // Clear the Rx buffer
      pRxPacketBytes = (char*)RxPacket.GetRawBytes();

      // Set up a file selector to block for received packets
      Selector.SetFile(m_sockfd, FileSelector::Readable);

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
            struct sockaddr_storage their_addr;
            socklen_t addr_len = sizeof their_addr;
            int numbytes = recvfrom( m_sockfd, pRxPacketBytes,
                                     BUNNYSOCK_PACKET_SIZE , 0,
                                     (struct sockaddr *)&their_addr, &addr_len );

            if ( -1 == numbytes )
            {
               BSOCK_VERBOSE2( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                               "recvfrom() failed on UDP socket!\n"));
                break;
            }

            // If a full packet has been received, process it
            assert(numbytes == BUNNYSOCK_PACKET_SIZE);

            LastRxTime = HPMOOSTime(false);   // Capture Rx time

            {
               char s[INET6_ADDRSTRLEN];
               BSOCK_VERBOSE2( MOOSTrace( BUNNYSOCK_UDP_NODE_STRING +
                     MOOSFormat( " Received a packet from %s\n",
                                 inet_ntop(their_addr.ss_family,
                                           get_in_addr((struct sockaddr *)&their_addr),
                                           s, sizeof s)) ));
            }

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
      if (INVALID_SOCKET_FD != m_sockfd)
      {
         close(m_sockfd);
         m_sockfd = INVALID_SOCKET_FD;
      }

   }   // END while ( !m_WorkerThread.IsQuitRequested() )

   BSOCK_VERBOSE1( MOOSTrace(BUNNYSOCK_UDP_NODE_STRING +
                             " Worker thread exiting.\n") );
}

