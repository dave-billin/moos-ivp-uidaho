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
/** @file BunnySockUdpNode.h
 *
 * @brief
 *   Definition of the BunnySockUdpNode class used to represent a single
 *   BunnySock TCP connection
 *
 * @author   Dave Billin
 */
//=============================================================================

#ifndef BUNNYSOCKUDPNODE_H_
#define BUNNYSOCKUDPNODE_H_

// Currently, the platform isn't getting defined in an OpenEmbedded CMake build
// To fix this, we'll add this hack...
#ifndef UNIX
   #ifndef _WIN32
      #define UNIX
   #endif
#endif

#include "MOOS/libMOOS/Utils/MOOSThread.h"
#include "sockets.h"
#include "YellowSubUtils.h"         // Needed for timed lock object
#include "BunnySockNode.h"

#include <memory>
#include <stdint.h>


using YellowSubUtils::TimedLock;

namespace BunnySock
{

//=============================================================================
/** A class that implements a BunnySock node that communicates using UDP
 *  broadcast datagrams
 *
 * @details
 *   BunnySock UDP nodes are provided as an adjunct to the normal (TCP) method
 *   of network communication.  They are <i>specifically intended</i> for the
 *   situation where a time-critical packet is being sent to <i>multiple</i>
 *   devices at once.  Due to the nature of the connectionless UDP protocol,
 *   there is no guarantee that data sent from the node will actually reach its
 *   destination.  Consequently, <b><i>TCP nodes should be favored over UDP
 *   nodes whenever possible</i></b>.
 */
class BunnySockUdpNode : public BunnySockNode
{
public:

   //=========================================================================
   /** Creates a (useless) BunnySock UDP node with a device ID and verbosity
    *   of zero */
   BunnySockUdpNode( void );


   //=========================================================================
   /** Creates a BunnySock Udp Node that communicates over a specified network
    *  port
    *
    * @param rx_port
    *    Network port the node will receive on
    *
    * @param tx_port
    *    Remote network port the node will send to
    *
    * @param DeviceId
    *    Device ID to report in the SourceID field of packets
    */
   BunnySockUdpNode( uint16_t rx_port, uint16_t tx_port, uint16_t DeviceId,
                     uint16_t Verbosity = 0);


   /** Called when the node goes out of scope */
   virtual ~BunnySockUdpNode();


   ////////////////////////////////////////////////////////////////////////////
   /// @addtogroup impl Implementation of BunnySockNode methods
   /// @{

   /** Inherited from the BunnySockNode class.  Does nothing, since UDP is
    *  connectionless.
    */
   void ResetConnection( void );


   /** Sends a BunnySock packet to the connected peer.
    *
    * @param [in] PacketToSend
    *    Reference to a BunnySockPacket to be sent
    *
    * @param [out] OUT_pPacketWasSent
    *   Pointer to a variable that will be populated with the return value of
    *   the send operation, or NULL if a return value is not required.
    *      - true if the packet was sent successfully
    *      - false if the packet could not be sent
    */
   bool SendPacket( BunnySockPacket& PacketToSend );

   /// @}
   ////////////////////////////////////////////////////////////////////////////


   //=========================================================================
   /** Starts a worker thread that receives data from the node.  This must be
    *  called in order for the node to receive any data.
    */
   void Start( void );

   //=========================================================================
   /** Execution body of the internal worker thread used to send and receive
      packets.  Not intended to be called directly.  */
   void WorkerThreadBody( void );


private:
   uint16_t m_rx_port;  /**< Port to receive data on */
   uint16_t m_tx_port;  /**< Remote port to send data to */
   TimedLock m_TxLock;  /**< Mutex used to synchronize access to the socket */

   typedef std::auto_ptr<Sockets::UDP_broadcast_socket> UDP_socket_ptr;
   UDP_socket_ptr m_udp_socket;

   CMOOSThread m_WorkerThread;   /**< Worker thread used to receive packets */

   //-------------------------------------------------------------------------
   // Disallow copy construction and assignment
   BunnySockUdpNode(const BunnySockUdpNode&);
   const BunnySockUdpNode& operator= (const BunnySockUdpNode&);
};

}

#endif /* BUNNYSOCKUDPNODE_H_ */
