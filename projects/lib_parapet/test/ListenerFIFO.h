//=============================================================================
// A ParapetListener object used for unit tests
//=============================================================================

#ifndef LISTENERFIFO_H
#define LISTENERFIFO_H

#include <queue>
#include "parapet.h"
#include "ParapetListener.h"
#include "ParapetEventBroadcaster.h"

class ListenerFIFO : public parapet::ParapetListener
{
public:
    // Default constructor
    ListenerFIFO( void ) {}

    // Destructor
    ~ListenerFIFO() {}

    //---------------------------
    // ParapetListener methods
    //---------------------------

    //=========================================================================
    void OnPacketReceived( parapet::ParapetEventBroadcaster const* Source,
                           parapet::parapet_packet_t const& Packet,
                           double Timestamp_sec )
    {
        m_RxPackets.push(Packet);
    }


    //=========================================================================
    void OnPacketEvent( parapet::ParapetEventBroadcaster const* Source,
                        int EventID,
                        parapet::parapet_packet_t const* Packet,
                        double Timestamp_sec )
    {
        RxEvent EventData( Source, EventID, Packet );
        m_RxEvents.push(EventData);
    }


    //=========================================================================
    bool GetRxPacket( parapet::parapet_packet_t& Target )
    {
        bool retval = false;

        if ( m_RxPackets.empty() == false )
        {
            Target = m_RxPackets.front();
            m_RxPackets.pop();
            retval = true;
        }

        return retval;
    }


    //=========================================================================
    // Inner class used to store info from a received event
    //=========================================================================
    class RxEvent
    {
    public:
        
        //=====================================================================        
        // Parameterized constructor - Stores a local copy of the
        // source packet if one was associated with the event
        RxEvent( parapet::ParapetEventBroadcaster const* Source, int EventID,
                 parapet::parapet_packet_t const* Packet )
            : m_Source(Source),
              m_EventID(EventID),
              m_HasPacket(false)
        {
            if ( Packet != NULL )
            {
                m_HasPacket = true;
                memcpy( &m_Packet, Packet, sizeof(parapet::parapet_packet_t) );
            }
        }


        //=====================================================================        
        // Destructor - deletes local copy of the event packet
        ~RxEvent()
        {}


        //=====================================================================        
        // Returns the source of the packet
        parapet::ParapetEventBroadcaster const* Source( void ) const 
        { return m_Source; }

        //=====================================================================        
        // Returns the event ID
        int EventID( void ) const { return m_EventID; }

        //=====================================================================        
        // Returns a pointer to a copy of the packet associated with the event
        // or NULL if no packet was associated
        parapet::parapet_packet_t const* Packet( void ) const 
        { return (m_HasPacket == true) ? &m_Packet : NULL; }

    private:
        parapet::ParapetEventBroadcaster const* m_Source;
        int m_EventID;
        parapet::parapet_packet_t m_Packet;
        bool m_HasPacket;       // True if event has an associated packet
    };


    std::queue<parapet::parapet_packet_t> m_RxPackets;    // Received packets
    std::queue<RxEvent> m_RxEvents;              // Events
};

#endif  // END #ifndef FIFOLISTENER_H
