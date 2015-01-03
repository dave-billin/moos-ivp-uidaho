
//=============================================================================
/** @file ParapetPacketFIFO.h
 *
 * @brief
 *  Declaration of a class that receives and enqueues packets from a parapet
 *  Master or Slave device
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#ifndef RXPACKETFIFO_H
#define RXPACKETFIFO_H

#include <stdint.h>
#include <cstring>
#include <queue>

#include "parapet.h"
#include "ParapetListener.h"
#include "ParapetEventBroadcaster.h"
#include "PrecisionTime.h"


namespace parapet
{

class ParapetPacketBroadcaster;     // Forward-declaration

//=============================================================================
/** @class ParapetPacketFIFO
 * @brief
 *   A ParapetListener object that manages a queue of received parapet packets
 */
//=============================================================================
class ParapetPacketFIFO : public parapet::ParapetListener
{
public:
    // Default constructor
    ParapetPacketFIFO( void ) {}

    // Destructor
    virtual ~ParapetPacketFIFO() {}


    //---------------------------
    // ParapetListener methods
    //---------------------------

    //=========================================================================
    void OnPacketReceived( ParapetEventBroadcaster const& Source,
                           parapet_opcode_t Opcode,
                           uint8_t SequenceID,
                           parapet_packet_t const& Packet );


    //=========================================================================
    void OnPacketEvent( ParapetEventBroadcaster const* Source,
                        int EventID )
    {}


    //=========================================================================
    // Inner class used to store info from a received event
    //=========================================================================
    struct RxPacketData
    {
        //=====================================================================        
        // Parameterized constructor - Stores a local copy of the
        // source packet if one was associated with the event
        RxPacketData( parapet_packet_t const& SrcPacket,
                      ParapetEventBroadcaster const* Source,
                      YellowSubUtils::PrecisionTime& RxTimestamp )
          : ReceiverObject( Source ),
            Timestamp( RxTimestamp )
        {
            // Const cast of packet reference is needed because the C
            // function parapet_PacketSize is not const-qualified
            parapet_packet_t& SrcPacketRef =
                    const_cast<parapet_packet_t&>( SrcPacket );

            // Copy packet data
            uint16_t PacketLength = parapet_PacketSize( &SrcPacketRef );
            memcpy( &Packet, &SrcPacket, PacketLength );
        }

        /// Pointer to the object the packet was received from
        parapet_packet_t Packet;
        parapet::ParapetEventBroadcaster const* ReceiverObject;
        YellowSubUtils::PrecisionTime Timestamp;
    };


    //=========================================================================
    /** Returns a a data structure containing the oldest received packet in
     *  the FIFO
     *
     * @param [out] PacketData
     *   Reference to an RxPacketData structure to be populated with received
     *   packet data
     *
     * @return
     *   true if PacketData was populated; else false if the FIFO was empty
     */
    bool Peek( RxPacketData& PacketData );


    //=========================================================================
    /** Returns a data structure containing the oldest received packet in the
     *  FIFO and removes the packet from the FIFO
     *
     * @param [out] PacketData
     *   Reference to an RxPacketData structure to be populated with received
     *   packet data
     *
     * @return
     *   true if PacketData was populated; else false if the FIFO was empty
     */
    bool Pop( RxPacketData& PacketData );


    //=========================================================================
    /** @return The number of received packets in the FIFO */
    uint32_t NumPacketsInFIFO( void ) const { return m_FIFO.size(); }


private:
    std::queue<RxPacketData> m_FIFO;   ///< Queue of Received packets

    //-----------------------------------------------------------------
    // Disallow default copy constructor and assignment operator
    ParapetPacketFIFO( ParapetPacketFIFO const& );
    ParapetPacketFIFO& operator=( ParapetPacketFIFO const& );
    //-----------------------------------------------------------------
};

}   // END namespace parapet

#endif  // END #ifndef FIFOLISTENER_H
