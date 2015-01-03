//=============================================================================
/** @file BSDuCStartupFSM.cpp
 *
 * @brief
 *	Brief description of BSDuCStartupFSM.cpp
 *
 * @author dave
 */
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "BSDuC_parapet_packets.h"
#include "BSDuCStartupFSM.h"

using namespace YellowSubUtils;
using namespace parapet;

static const char szDebugString[] = "<BSD Session Startup> ";
static const char* szStartupPacketTable[] = { "firmware version",
                                              "sensor",
                                              "battery monitor" };

//=============================================================================
BSDuC_StartupFSM::BSDuC_StartupFSM( ParapetSerialDeviceNode& ParapetNode )
  : m_ParapetNode( ParapetNode )
{
    Reset();
}


//=============================================================================
BSDuC_StartupFSM::~BSDuC_StartupFSM()
{
    m_ParapetNode.RemoveListener( this );
}


//=============================================================================
bool BSDuC_StartupFSM::Process(void)
{
    // Return true immediately if startup sequence is already finished
    if ( SESSION_ONLINE == m_State )
    {
        return true;
    }
    else
    {

        //-----------------------------------
        // Reset FSM if a request times out
        //-----------------------------------
        eBSDuC_FSM_States NextState = m_State;
        PrecisionTime t = PrecisionTime::Now();

        if ( ( ENTRY != m_State ) &&
             ( (t - m_ReferenceTime) >= m_Timeout ) )
        {
            MOOSTrace( "%s Timed out waiting for %s packet from BSD "
                       "microcontroller! Restarting startup sequence.\n",
                       szDebugString, szStartupPacketTable[m_State] );

            m_State = ENTRY;
        }
        else if ( m_ExpectedPacketWasReceived )
        {
            // If the expected packet was received, set up the next state
            NextState = static_cast<eBSDuC_FSM_States>( NextState + 1 );
        }

        // Send packets when state changes (FSM transition outputs)
        if ( NextState != m_State )
        {
            bool ShouldSendRequest = true;
            parapet_opcode_t Opcode;
            uint16_t DescriptorID;

            switch ( NextState )
            {
                case READ_FIRMWARE_VERSION:
                {
                    Opcode = OPCODE_EXECUTE;
                    DescriptorID = BSDuC_parapet_packets::START_SESSION;
                    break;
                }

                case READ_INITIAL_SENSORS:
                {
                    Opcode = OPCODE_READ_PARAM;
                    DescriptorID = BSDuC_parapet_packets::SENSORS;
                    break;
                }

                case READ_INITIAL_BATTERY_MONITOR:
                {
                    Opcode = OPCODE_READ_PARAM;
                    DescriptorID = BSDuC_parapet_packets::BATTERY_MONITOR;
                    break;
                }

                default:
                {
                    ShouldSendRequest = false;
                    break;
                }
            }

            if ( ShouldSendRequest )
            {
                MOOSTrace( "%s Sending %s request packet\n",
                           szDebugString, szStartupPacketTable[ NextState ] );

                parapet_short_request_t Request;
                parapet_packet_t* RequestPacket =
                        reinterpret_cast<parapet_packet_t*>( &Request );

                parapet_BuildRequest( &RequestPacket->Request, Opcode,
                              0,
                              BSDuC_parapet_packets::BSDuC_PARAPET_DEVICE_ID,
                              0, DescriptorID );

                m_ExpectedPacketWasReceived = false;
                int8_t SequenceID = m_ParapetNode.Transmit( *RequestPacket );

                if ( SequenceID >= 0 )
                {
                    m_Timeout.Set( 0.5 );   // 500 ms timeout
                    m_ReferenceTime = t;
                    m_ExpectedSequenceID = SequenceID;
                    m_ExpectedOpcode = Opcode;
                }

                m_State = NextState;
            }
        }
    }

    return (SESSION_ONLINE == m_State);
}


//=============================================================================
void BSDuC_StartupFSM::Reset( void )
{
    m_ParapetNode.RemoveListener( this );
    m_ReferenceTime.Zero();
    m_Timeout.Zero();
    m_ExpectedOpcode = parapet::OPCODE_EXECUTE;
    m_ExpectedPacketWasReceived = false;
    m_State = OPEN_SESSION;
}


//=============================================================================
void BSDuC_StartupFSM::OnPacketReceived(
        const parapet::ParapetEventBroadcaster& Source,
        parapet::parapet_opcode_t Opcode, uint8_t SequenceID,
        const parapet::parapet_packet_t& Packet)
{
}


//=============================================================================
void BSDuC_StartupFSM::OnPacketEvent(
        const parapet::ParapetEventBroadcaster& Source, int EventID)
{
}
