//=============================================================================
// Unit testing of the ParapetSlave module
//=============================================================================

#include <cstring>
#include <queue>
#include <memory>
#include "gtest/gtest.h"
#include "parapet.h"
#include "parapet_node.h"


using namespace std;
using namespace parapet;

//=============================================================================
// Test fixture for unit tests of the parapet_node module
//=============================================================================
class ParapetNodeTestFixture : public ::testing::Test
{

protected:

    //=========================================================================
    // Constructor
    ParapetNodeTestFixture( void )
        : DEVICE_ID(7)
    {
    }

    //=========================================================================
    // Destructor
    ~ParapetNodeTestFixture( void )
    {
    }

    //=========================================================================
    // Function called immediately after the constructor and before each test.
    virtual void SetUp()
    {
        // Initialize the device node
        parapet_node_init( &Node, DEVICE_ID );

        // Register callback functions
        for ( int i = 0; i < 7; ++i )
        {
            parapet_SetRxHandler( &Node, (parapet_opcode_t)i, OnPacketRx );
        }

        parapet_SetEventHandler( &Node, ParapetNodeTestFixture::OnEvent );
    }



    //=========================================================================
    // Packet receive callback used to receive packets from Node
    static void OnPacketRx( void*,
                            parapet_packet_t* Packet )
    {
        ParapetNodeTestFixture::RxPacketQueue.push( *Packet );
    }

    //=========================================================================
    // Event callback used to receive events from Node
    static void OnEvent( void*,
                         parapet_event_id_t EventID )
    {
        ParapetNodeTestFixture::EventQueue.push( EventID );
    }


    parapet_node_t Node;    // ParapetSlave unit under test
    const uint8_t DEVICE_ID; // Device ID of the node

    static std::queue<parapet_packet_t> RxPacketQueue;
    static std::queue<parapet_event_id_t> EventQueue;

};



std::queue<parapet_packet_t> ParapetNodeTestFixture::RxPacketQueue;
std::queue<parapet_event_id_t> ParapetNodeTestFixture::EventQueue;




//=============================================================================
TEST_F( ParapetNodeTestFixture, Receive_Packets )
{
    const uint8_t REQUEST_SOURCE_ID = 5;
    const uint8_t REQUEST_SEQUENCE_ID = 1;
    const uint16_t REQUEST_DESCRIPTOR_ID = 0x5555;

    const uint32_t PAYLOAD_DATA[ 16 ] =
                { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    enum { PAYLOAD_NUM_BYTES = sizeof(uint32_t) * 16 };


    parapet_packet_t Request;
    parapet_packet_t RequestWithPayload;  // payload = 16 32-bit integers
    uint16_t Request_NumBytes = 0;
    uint16_t RequestWithPayload_NumBytes = 0;
    // Initialize the first request
    parapet_BuildRequest( &Request.Request, OPCODE_EXECUTE,
                          REQUEST_SOURCE_ID, DEVICE_ID, REQUEST_SEQUENCE_ID,
                          REQUEST_DESCRIPTOR_ID );

    // Initialize the second request and give it payload data
    memcpy( &RequestWithPayload, &Request,
            parapet_PacketSize( &Request ) );

    parapet_AddPayloadData( &RequestWithPayload, (void*)PAYLOAD_DATA,
                            PAYLOAD_NUM_BYTES );

    Request_NumBytes = parapet_PacketSize( &Request );
    RequestWithPayload_NumBytes = parapet_PacketSize(&RequestWithPayload);

    //---------------------------------------------
    // Add the following data to the Rx buffer:
    //  - Five bogus 'noise' Bytes
    //  - Packet data of the first request
    //  - One bogus 'noise' Byte
    //  - Packet data of the second request
    //---------------------------------------------
    const uint32_t RxBufferSize =   5
                                + Request_NumBytes
                                + 1
                                + RequestWithPayload_NumBytes;

    std::auto_ptr<char> RxBuffer( new char[ RxBufferSize ] );

    char* pWrite = RxBuffer.get();

    for ( int i = 0; i < 5; ++i )
    {
        *pWrite++ = 0xff;       // Seven 'bogus' noise Bytes
    }

    memcpy( pWrite, &Request, Request_NumBytes );
    pWrite += Request_NumBytes;

    *pWrite++ = 0xff;

    memcpy( pWrite, &RequestWithPayload, RequestWithPayload_NumBytes );
    pWrite += RequestWithPayload_NumBytes;




    //---------------------------------------------
    // Process the buffer to receive the requests
    //---------------------------------------------
    parapet_process_node( &Node, RxBuffer.get(), RxBufferSize );


    //------------------------------------------------
    // Verify that six EVENT_HEADER_CHECKSUM_FAILURE
    // events were generated as a result of the
    // 'noise' Bytes interspersed with packet data
    //------------------------------------------------
    EXPECT_EQ( 6, EventQueue.size() );
    while( EventQueue.empty() == false )
    {
        parapet_event_id_t QueuedEvent = EventQueue.front();
        EventQueue.pop();
        EXPECT_EQ( EVENT_HEADER_CHECKSUM_FAILURE, QueuedEvent );
    }

    //------------------------------------------------
    // Verify the two packets that were received
    //------------------------------------------------
    EXPECT_EQ( 2, RxPacketQueue.size() );

    // Verify Request without payload
    parapet_packet_t RxPacket = RxPacketQueue.front();
    RxPacketQueue.pop();
    EXPECT_EQ( 0, memcmp( &Request, &RxPacket, Request_NumBytes ) );

    // Verify Request with payload
    RxPacket = RxPacketQueue.front();
    RxPacketQueue.pop();
    EXPECT_EQ( 0, memcmp( &RequestWithPayload, &RxPacket,
                          RequestWithPayload_NumBytes ) );
}

