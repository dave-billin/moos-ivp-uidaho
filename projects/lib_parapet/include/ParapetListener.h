//=============================================================================
/** @file ParapetListener.h
 *
 * @brief
 *	Declaration of a callback interface class used to receive parapet packets
 *	from a parapet_Master or parapet_Slave object
 *
 * @author dave
 */
//=============================================================================

#ifndef PARAPET_LISTENER_H_
#define PARAPET_LISTENER_H_

#include "parapet.h"
#include "ParapetEventBroadcaster.h"

namespace parapet
{

class ParapetEventBroadcaster;  // Forward declaration

//=============================================================================
/** Abstract base class used to define an interface for objects that need to
 *  receive packets from a parapet_Master or parapet_Slave object
 */
class ParapetListener
{
public:
    ParapetListener( void ) {}     /**< Default constructor */

    virtual ~ParapetListener( void ) {}    /**< Destructor */

    //=========================================================================
    /** This function will be called when a parapet packet has been received
     *
     * @details
     *  This function gets called when a parapet packet is received by an
     *  object implementing the ParapetEventBroadcaster interface.
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object  that received the
     *  packet.
     *
     * @param [in] Opcode
     *  The value of the OPCODE field in the received packet's header
     *
     * @param [in] SequenceID
     *  Sequence ID of the received packet
     *
     * @param [in] Packet
     *  A reference to the packet that was received.
     */
    virtual void OnPacketReceived( ParapetEventBroadcaster const& Source,
                                   parapet_opcode_t Opcode,
                                   uint8_t SequenceID,
                                   parapet_packet_t const& Packet ) = 0;


    //=========================================================================
    /** This function will be called when a notable event occurs on a
     *  ParapetEventBroadcaster object
     *
     * @details
     *  This function gets called when an error condition is detected by an
     *  object implementing the ParapetEventBroadcaster interface.
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object signaling the event
     *
     * @param [in] EventID
     *  ID of the event
     */
    virtual void OnPacketEvent( ParapetEventBroadcaster const& Source,
                                int EventID ) = 0;

};

}   // END namespace parapet

#endif /* PARAPET_LISTENER_H_ */
