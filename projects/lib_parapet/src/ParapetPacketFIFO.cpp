//=============================================================================
/** @file ParapetPacketFIFO.cpp
 *
 * @brief
 *  Implementation of the ParapetPacketFIFO class
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <cassert>
#include <cstddef>
#include "ParapetPacketFIFO.h"

using namespace std;

namespace parapet
{

//=============================================================================
void ParapetPacketFIFO::OnPacketReceived(ParapetEventBroadcaster const& Source,
                                         parapet_opcode_t Opcode,
                                         uint8_t SequenceID,
                                         parapet_packet_t const& Packet )
{
    YellowSubUtils::PrecisionTime RxTimeStamp =
            YellowSubUtils::PrecisionTime::Now( true );
    m_FIFO.push( RxPacketData( Packet, &Source, RxTimeStamp ) );
}



//=============================================================================
bool ParapetPacketFIFO::Peek( RxPacketData& PacketData )
{
    bool retval = false;

    if ( m_FIFO.empty() == false )
    {
        PacketData = m_FIFO.front();
        retval = true;
    }

    return retval;
}


//=============================================================================
bool ParapetPacketFIFO::Pop( RxPacketData& PacketData )
{
    bool retval = false;

    if ( m_FIFO.empty() == false )
    {
        PacketData = m_FIFO.front();
        m_FIFO.pop();
        retval = true;
    }

    return retval;
}


}   // END namespace parapet
