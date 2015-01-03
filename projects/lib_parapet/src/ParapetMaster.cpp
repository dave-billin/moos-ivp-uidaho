//=============================================================================
/** @file ParapetMaster.cpp
 *
 * @brief
 *  Implementation of the ParapetMaster class
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================
#include <cstring>
#include <cassert>

#include "ParapetMaster.h"

namespace parapet
{


//=============================================================================
ParapetMaster::ParapetMaster()
 : m_RxState(RX_SYNC),
   m_NumBytesReceived(0),
   m_NumRxBytesNeeded(1),
   m_RequestID(0)
{
    // Initialize the Rx packet buffer
    memset( &m_RxPacket, 0, sizeof(parapet_packet_t) );

}


//=============================================================================
ParapetMaster::~ParapetMaster()
{

}



//=============================================================================
bool ParapetMaster::ProcessRxData( char const* RxData,
                                   uint32_t NumBytesToProcess )
{
    rx_state_t NextState = RX_SYNC;   // Used to implement FSM transitions
    parapet_header_t& Header = m_RxPacket.ContinueRequest.Header;
    int NumBytesToCopy;
    bool retval = false;

    while (NumBytesToProcess > 0)
    {
        // Copy the required number of Bytes into the Rx packet buffer
        NumBytesToCopy = (m_NumRxBytesNeeded < NumBytesToProcess) ?
                                    m_NumRxBytesNeeded : NumBytesToProcess;
        memcpy(&m_RxPacket.Bytes[m_NumBytesReceived], RxData, NumBytesToCopy);
        RxData += NumBytesToCopy;
        m_NumBytesReceived += NumBytesToCopy;
        m_NumRxBytesNeeded -= NumBytesToCopy;
        NumBytesToProcess -= NumBytesToCopy;
        
        if (m_NumRxBytesNeeded == 0)
        {

            // FSM state transition logic
            switch (m_RxState)
            {
                case RX_SYNC:
                {
                    if ( Header.fields.Sync == parapet_header_t::SYNC_BYTE )
                    {
                        NextState = RX_HEADER;
                    }
                    else
                    {
                        m_NumRxBytesNeeded = 1;
                        m_NumBytesReceived = 0;
                    }
                    break;
                }


                case RX_HEADER:
                {
                    // STATUS:
                    //  A complete packet header has been received
                    //  Now validate header fields

                    //-----------------------------------------
                    // Detect non-critical protocol errors
                    //-----------------------------------------

                    // Slaves must not reply to requests addressed to Device 0
                    if ( Header.fields.DeviceID == 0 )
                    {
                        PostEvent( DEVICEID_ZERO_REPLY, &m_RxPacket,
                                   ParapetEventBroadcaster::MakeTimestamp() );
                    }

                    // Slaves should not reply with the M (master) bit set
                    if ( Header.fields.M == 1 )
                    {
                        PostEvent( SLAVE_M_BIT_SET, &m_RxPacket,
                                   ParapetEventBroadcaster::MakeTimestamp() );
                    }
                    

                    // Determine whether the packet has payload data
                    if ( parapet::HasPayload(m_RxPacket) )
                    {
                        // Validate NumBytesToFollow
                        uint16_t NumBytesToFollow =
                                    Header.fields.NumBytesToFollow + 1;

                        if ( NumBytesToFollow <= MAX_PAYLOAD_BYTES )
                        {
                            // Set up to receive payload data
                            NextState = RX_PAYLOAD;
                            break;
                        }
                        else
                        {
                            // NumBytesToFollow value is invalid!
                            // This is a critical error, since we can't know
                            // how to receive the remainder of the packet!
                            PostEvent( INVALID_NUMBYTESTOFOLLOW, &m_RxPacket,
                                    ParapetEventBroadcaster::MakeTimestamp() );

                            NextState = RX_SYNC;
                            continue;
                        }

                    }

                    // If the packet does not have payload data, fall
                    // through to process it immediately
                }
                /* FALLTHROUGH */


                case RX_PAYLOAD:
                {
                    // STATUS: All packet payload Bytes have been received

                    // Post the packet to registered listeners
                    PostReceivedPacket( m_RxPacket,
                                    ParapetEventBroadcaster::MakeTimestamp() );

                    retval = true;
                    NextState = RX_SYNC;
                    break;
                }

                default:
                {
                    assert(false);  // Invalid state should never happen!
                    NextState = RX_SYNC;
                    break;
                }
            }   // END select


            // Implement FSM state outputs
            if ( NextState != m_RxState )
            {
                switch (NextState)
                {
                    case RX_SYNC:
                    {
                        m_NumBytesReceived = 0;
                        m_NumRxBytesNeeded = 1;
                        break;
                    }

                    case RX_HEADER:
                    {
                        m_NumRxBytesNeeded = sizeof(parapet_header_t) - 1;
                        break;
                    }

                    case RX_PAYLOAD:
                    {
                        m_NumRxBytesNeeded =
                                    Header.fields.NumBytesToFollow + 1;
                       break; 
                    }

                    default:
                    {
                        assert(false);  // Invalid states should never happen!
                        NextState = RX_SYNC;
                        break;
                    }
                }   // END select

                m_RxState = NextState;
            }

        }   // END if (m_NumRxBytesNeeded == 0)

    }   // END while (NumBytesToProcess > 0)

    return retval;
}





}   // END namespace parapet
