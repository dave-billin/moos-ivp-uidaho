//=============================================================================
/** @file ParapetEventBroadcaster.cpp
 *
 * @brief
 *	Implementation of the ParapetEventBroadcaster class
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#include <time.h>
#include <unistd.h>

#include "parapet.h"
#include "ParapetEventBroadcaster.h"


namespace parapet
{


//=============================================================================
void ParapetEventBroadcaster::PostReceivedPacket(
                                            parapet_packet_t const& Packet )
{
    // Do nothing if no listeners are registered
    if ( m_Listeners.empty() == true )
    {
        return;
    }

    parapet_header_t const& Header = Packet.Header;
    parapet_opcode_t Opcode = PARAPET_HEADER_GET_OPCODE( Header );
    uint8_t SequenceID = PARAPET_HEADER_GET_SEQUENCE_ID( Header );

    for ( ListenerSet::iterator iter = m_Listeners.begin();
          iter != m_Listeners.end(); iter++ )
    {
        ParapetListener* Listener = *iter;
        Listener->OnPacketReceived( *this, Opcode, SequenceID, Packet );
    }
}



//=============================================================================
void ParapetEventBroadcaster::PostEvent( int EventID )
{
    // Do nothing if no listeners are registered
    if ( m_Listeners.empty() == true )
    {
        return;
    }

    for ( ListenerSet::iterator iter = m_Listeners.begin();
          iter != m_Listeners.end(); iter++ )
    {
        ParapetListener* Listener = *iter;
        Listener->OnPacketEvent( *this, EventID );
    }
}



//=============================================================================
double ParapetEventBroadcaster::MakeTimestamp( void )
{
    struct timespec TimeNow;
    clock_gettime( CLOCK_REALTIME, &TimeNow );
    return TimeNow.tv_sec + (TimeNow.tv_nsec * 1e-9);
}



//=============================================================================
ParapetEventBroadcaster::ListenerSet::iterator
ParapetEventBroadcaster::FindListener( ParapetListener* Target )
{
    ListenerSet::iterator iter;
    for ( iter = m_Listeners.begin();
          iter != m_Listeners.end(); iter++ )
    {
        // Make sure the listener is not already registered to
        // prevent listeners from getting multiple callbacks
        if ( *iter == Target )
        {
            break;
        }
    }

    return iter;
}


} // END namespace parapet
