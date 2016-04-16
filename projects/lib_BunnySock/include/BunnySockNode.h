//=============================================================================
/*    Copyright (c) 2012, 2016  Dave Billin

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
/** @file BunnySockNode.h
 *
 * @brief
 *  Declaration of the BunnySockNode base class
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef BUNNYSOCKNODE_H_
#define BUNNYSOCKNODE_H_

#include <list>
#include <stdint.h>

#include "MOOS/libMOOS/Utils/MOOSLock.h"			// MOOS lock class
#include "BunnySockListener.h"

//------------------------------------------------------
/** @def BUNNYSOCK_DEFAULT_CONNECTION_TIMEOUT_MS
 * @brief
 * 	Default TCP connection timeout (milliseconds) after which a BunnySock
 * 	node's connection will be reset.
 */
#define BUNNYSOCK_DEFAULT_CONNECTION_TIMEOUT_MS		3000

#define BSOCK_VERBOSE1(_expr_)	if (m_Verbosity >= 1) {_expr_;}
#define BSOCK_VERBOSE2(_expr_)	if (m_Verbosity >= 2) {_expr_;}
#define BSOCK_VERBOSE3(_expr_)	if (m_Verbosity >= 3) {_expr_;}
#define BSOCK_VERBOSE4(_expr_)	if (m_Verbosity >= 4) {_expr_;}
#define BSOCK_VERBOSE5(_expr_)	if (m_Verbosity >= 5) {_expr_;}
#define BSOCK_VERBOSE6(_expr_)	if (m_Verbosity >= 6) {_expr_;}
#define BSOCK_VERBOSE7(_expr_)	if (m_Verbosity >= 7) {_expr_;}
#define BSOCK_VERBOSE8(_expr_)	if (m_Verbosity >= 8) {_expr_;}


namespace BunnySock
{

//=============================================================================
/** Base class from which all BunnySock connections inherit.  This class
 *  establishes interface functions common to all nodes and provides the
 *  mechanisms needed to register and notify listener objects for received
 *  packets and events.
 */
class BunnySockNode
{
public:

   //=========================================================================
   /** Creates a node with no listeners */
   BunnySockNode(void);

   //=========================================================================
   /** Creates a node and registers a listener that it will notify of
    *  connection events and packets
    */
   BunnySockNode(BunnySockListener& listener, uint16_t DeviceId = 0,
                 uint16_t Verbosity = 0);

   //=========================================================================
   /** Called when the object goes out of scope */
   virtual ~BunnySockNode();

   //=========================================================================
   /** Registers a listener to be notified of received packets and connection
    * events
    *
    * @param listener   Listener object to register
    */
   void AddListener( BunnySockListener& listener );

   //=========================================================================
   /** Un-registers a listener
    *
    * @param listener   Listener object to unregister
    */
   void RemoveListener(BunnySockListener& pListener);

   //=========================================================================
   /** Returns true if the node is currently connected with a remote peer
    * Derived classes must implement this function
    *
    * @param NodeIsConnected
    *	Reference to a variable to be populated with the return value of this
    *	function.
    */
   bool IsConnected(void) const
   { return m_IsConnected; }

   //=========================================================================
   /** Sets the node's device ID */
   void SetDeviceId(uint16_t DeviceId)
   { m_DeviceId = DeviceId; }

   //=========================================================================
   /** Returns the node's device ID */
   uint16_t GetDeviceId(void)
   { return m_DeviceId; }

   //======================================
   // BUNNYSOCK NODE INTERFACE FUNCTIONS
   //======================================

   //=========================================================================
   /** Resets the node's connection with a remote peer.  Derived classes must
    * implement this function */
   virtual void ResetConnection(void) = 0;

   //=========================================================================
   /** Sends a BunnySock packet to a connected peer.  Derived classes must
    * implement this function
    *
    * @param [in] PacketToSend
    * 	Reference to the BunnySockPacket being sent
    *
    * @param [out] OUT_pPacketWasSent
    *	Pointer to a variable that will be populated with the return value of
    *	the send operation: true if the packet was sent successfully, false if
    *	the packet could not be sent, NULL if a return value is not required.
    */
   virtual bool SendPacket(BunnySockPacket& PacketToSend) = 0;


   unsigned long m_ConnectionTimeout_ms; /**< Maximum number of milliseconds
    that may elapse without
    receiving a packet from a
    connected BunnySock peer
    before the connection is
    reset */

protected:
   bool m_IsConnected; /**< True if the node is connected to a peer, or if
    a connectionless protocol (such as UDP) is being
    used */

   CMOOSLock m_ListenerListLock; /**< Mutex for synchronizing the node's
    listener list */

   std::list<BunnySockListener*> m_ListenerList; /**< A list of objects to
    be notified of
    received packets and
    connection events */

   uint16_t m_Verbosity; /**< Verbosity of debugging messages printed to
    stdio */

   uint16_t m_DeviceId; /**< Device ID registered as the node's
    source ID for keep-alive packets */

   //=========================================================================
   /** Helper function for reporting connection events to listeners */
   void ReportConnectionEvent( BunnySockListener::ConnectionEventId EventId);

   //=========================================================================
   /** A Helper function that configures the current thread to ignore UNIX
    * SIGPIPE (broken pipe) signals that may occur during normal operation.
    * This function does nothing under Windows */
   static void IgnoreSigPipe(void);

private:

   //-------------------------------------------------------------------------
   // Prevent automatic generation of copy constructor and assignment operator
   BunnySockNode(const BunnySockNode&);
   const BunnySockNode& operator=(const BunnySockNode&);
};

}	// END namespace BunnySock

#endif /* BUNNYSOCKNODE_H_ */
