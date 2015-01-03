//=============================================================================
// Unit tests of functions defined in parapet.cpp
//=============================================================================
#include <cstring>
#include <stdint.h>
#include "gtest/gtest.h"
#include "parapet.h"

using namespace parapet;


//=============================================================================
TEST( Test_parapet, test_BuildRequestPacket )
{
    parapet_packet_t Packet;

    const parapet_opcode_t Opcode = OPCODE_RESERVED_5;
    const uint8_t SourceID = 0x0a;
    const uint8_t DestID = 0x05;
    const uint8_t SequenceID = 0;
    const uint16_t DescriptorID = 0xa5a5;

    // Build a request packet
    parapet_BuildRequest( &Packet.Request, Opcode, SourceID, DestID, SequenceID,
                        DescriptorID );

    EXPECT_EQ( Opcode, PARAPET_HEADER_GET_OPCODE( Packet.Header ) );
    EXPECT_EQ( 1, PARAPET_HEADER_Q_FLAG_IS_SET( Packet.Header ) );
    EXPECT_EQ( SourceID, PARAPET_HEADER_GET_SOURCE_ID( Packet.Header ) );
    EXPECT_EQ( DestID, PARAPET_HEADER_GET_DEST_ID( Packet.Header ) );
    EXPECT_EQ( SequenceID, PARAPET_HEADER_GET_SEQUENCE_ID( Packet.Header ) );
    EXPECT_EQ( 1, PARAPET_HEADER_A_FLAG_IS_CLEARED( Packet.Header ) );
    EXPECT_EQ( sizeof(parapet_descriptor_t),
               PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( Packet.Header ) );

    // Verify header checksum
    uint8_t Checksum = PARAPET_HEADER_GET_CHECKSUM( Packet.Header );
    uint8_t CalculatedChecksum = CalculateHeaderChecksum( &Packet.Header );
    EXPECT_EQ( CalculatedChecksum, Checksum );

    EXPECT_NE( 0, PARAPET_HEADER_CHECKSUM_IS_VALID( Packet.Header ) );

    // Verify descriptor contents
    EXPECT_EQ( DescriptorID,
               PARAPET_DESCRIPTOR_GET_ID( Packet.Request.Descriptor ) );
    EXPECT_EQ( 0,
               PARAPET_DESCRIPTOR_GET_USER_BITS( Packet.Request.Descriptor ) );
}


//=============================================================================
TEST( Test_parapet, test_BuildResponsePacket )
{
    parapet_packet_t Packet;
    parapet_header_t RequestHeader;
    const parapet_opcode_t Opcode = OPCODE_RESERVED_5;
    const uint8_t SourceID = 0x0a;
    const uint8_t DestID = 0x05;
    const uint8_t SequenceID = 0;
    const uint8_t IsAck = 1;

    // Set up the request header
    PARAPET_HEADER_SET_OPCODE( RequestHeader, Opcode );
    PARAPET_HEADER_SET_SOURCE_ID( RequestHeader, DestID );
    PARAPET_HEADER_SET_DEST_ID( RequestHeader, SourceID );
    PARAPET_HEADER_SET_SEQUENCE_ID( RequestHeader, SequenceID );

    parapet_BuildResponse( &Packet.Response, &RequestHeader, SourceID, IsAck );

    // Verify the response packet
    EXPECT_EQ( Opcode, PARAPET_HEADER_GET_OPCODE( Packet.Header ) );
    EXPECT_EQ( 1, PARAPET_HEADER_Q_FLAG_IS_CLEARED( Packet.Header ) );
    EXPECT_EQ( SourceID, PARAPET_HEADER_GET_SOURCE_ID( Packet.Header ) );
    EXPECT_EQ( DestID, PARAPET_HEADER_GET_DEST_ID( Packet.Header ) );
    EXPECT_EQ( SequenceID, PARAPET_HEADER_GET_SEQUENCE_ID( Packet.Header ) );
    EXPECT_EQ( 0, PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( Packet.Header ) );
    EXPECT_NE( 0, PARAPET_HEADER_A_FLAG_IS_SET( Packet.Header ) );

    // Verify header checksum
    uint8_t Checksum = PARAPET_HEADER_GET_CHECKSUM( Packet.Header );
    uint8_t CalculatedChecksum = CalculateHeaderChecksum( &Packet.Header );
    EXPECT_EQ( CalculatedChecksum, Checksum );

    EXPECT_NE( 0, PARAPET_HEADER_CHECKSUM_IS_VALID( Packet.Header ) );
}



//=============================================================================
// Tests the parapet_AddPayloadData function with a Request packet
TEST( Test_parapet, test_AddPayloadData_Request )
{
    enum { PAYLOAD_SIZE =
            PARAPET_MAX_NUM_PAYLOAD_BYTES - sizeof(parapet_descriptor_t)
    };

    parapet_packet_t Packet;

    const parapet_opcode_t Opcode = OPCODE_RESERVED_5;
    const uint8_t SourceID = 0x0a;
    const uint8_t DestID = 0x05;
    const uint8_t SequenceID = 0;
    const uint16_t DescriptorID = 0xa5a5;

    // Build a request packet
    parapet_BuildRequest( &Packet.Request, Opcode, SourceID, DestID, SequenceID,
                        DescriptorID );

    // Initialize payload data content
    uint8_t PayloadData[ PAYLOAD_SIZE ];
    for ( int i = 0; i < PAYLOAD_SIZE; i++ )
    {
        PayloadData[i] = static_cast<uint8_t>( i );
    }

    //--------------------------------
    // TEST ERROR CASES:
    //--------------------------------
    #ifndef PARAPET_ENABLE_ASSERTS
    // Verify parapet_AddPayloadData with a NULL parameter
    EXPECT_EQ( 0, parapet_AddPayloadData( &Packet, NULL, 100 ) );
    #endif

    // Attempt to add too much payload data to the packet (packet is a request
    // so it already contains 4 Bytes of payload)
    EXPECT_EQ( 0, parapet_AddPayloadData( &Packet, PayloadData,
                                  PARAPET_MAX_NUM_PAYLOAD_BYTES ) );


    //--------------------------------
    // TEST VALID CASES:
    //--------------------------------
    uint16_t ExpectedPacketSize =
            sizeof(parapet_header_t) + sizeof(parapet_descriptor_t);

    // Add zero payload Bytes
    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, PayloadData, 0 ) );

    // Add a single payload data Byte to the packet
    uint8_t* p_Read = PayloadData;
    ExpectedPacketSize += 1;
    EXPECT_EQ( ExpectedPacketSize, parapet_AddPayloadData( &Packet, p_Read, 1 ) );
    p_Read += 1;

    // Add slightly more payload data to the packet
    ExpectedPacketSize += 4;
    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, p_Read, 4 ) );
    p_Read += 4;

    // Add the remainder of payload data to the packet
    ExpectedPacketSize = PARAPET_MAX_BYTES_PER_PACKET;
    uint16_t NumBytesToAdd = PAYLOAD_SIZE - (p_Read - PayloadData);

    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, p_Read, NumBytesToAdd ) );

    // Verify packet payload contents
    EXPECT_EQ( 0,
               memcmp( Packet.Request.Payload, PayloadData, PAYLOAD_SIZE ) );
}



//=============================================================================
// Tests the parapet_AddPayloadData function with a Response packet
TEST( Test_parapet, test_AddPayloadData_Response )
{
    enum { PAYLOAD_SIZE = PARAPET_MAX_NUM_PAYLOAD_BYTES };

    parapet_packet_t Packet;
    parapet_packet_t RequestPacket;

    const parapet_opcode_t Opcode = OPCODE_RESERVED_5;
    const uint8_t SourceID = 0x0a;
    const uint8_t DestID = 0x05;
    const uint8_t SequenceID = 0;
    const uint16_t DescriptorID = 0xa5a5;

    // Build a request packet
    parapet_BuildRequest( &RequestPacket.Request, Opcode, DestID, SourceID,
                        SequenceID, DescriptorID );

    // Build a response packet
    parapet_BuildResponse( &Packet.Response, &RequestPacket.Header,
                         SourceID, 1 );

    // Initialize payload data content
    uint8_t PayloadData[ PAYLOAD_SIZE ];
    for ( int i = 0; i < PAYLOAD_SIZE; i++ )
    {
        PayloadData[i] = static_cast<uint8_t>( i );
    }

    //--------------------------------
    // TEST ERROR CASES:
    //--------------------------------
    #ifndef PARAPET_ENABLE_ASSERTS
    // Verify parapet_AddPayloadData with a NULL parameter
    EXPECT_EQ( 0, parapet_AddPayloadData( &Packet, NULL, 100 ) );
    #endif

    // Attempt to add too much payload data to the packet
    EXPECT_EQ( 0, parapet_AddPayloadData( &Packet, PayloadData, PAYLOAD_SIZE + 1 ) );


    //--------------------------------
    // TEST VALID CASES:
    //--------------------------------
    uint16_t ExpectedPacketSize = sizeof(parapet_header_t);

    // Add zero payload Bytes
    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, PayloadData, 0 ) );

    // Add a single payload data Byte to the packet
    uint8_t* p_Read = PayloadData;
    ExpectedPacketSize += 1;
    EXPECT_EQ( ExpectedPacketSize, parapet_AddPayloadData( &Packet, p_Read, 1 ) );
    p_Read += 1;

    // Add slightly more payload data to the packet
    ExpectedPacketSize += 4;
    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, p_Read, 4 ) );
    p_Read += 4;

    // Add the remainder of payload data to the packet
    ExpectedPacketSize = PARAPET_MAX_BYTES_PER_PACKET;
    uint16_t NumBytesToAdd = PAYLOAD_SIZE - (p_Read - PayloadData);

    EXPECT_EQ( ExpectedPacketSize,
               parapet_AddPayloadData( &Packet, p_Read, NumBytesToAdd ) );

    // Verify packet payload contents
    EXPECT_EQ( 0,
               memcmp( Packet.Response.Payload, PayloadData, PAYLOAD_SIZE ) );
}



//=============================================================================
TEST( Test_parapet, test_PacketSize )
{
    parapet_packet_t Packet;
    uint16_t ExpectedSize = 0;

    memset( &Packet, 0, sizeof(parapet_packet_t) );

    // Verify with no payload
    PARAPET_HEADER_SET_PAYLOAD_COUNT( Packet.Header, 0 );
    ExpectedSize = sizeof(parapet_header_t);
    EXPECT_EQ( ExpectedSize, parapet_PacketSize( &Packet ) );
    EXPECT_EQ( 0, PARAPET_PACKET_HAS_PAYLOAD( Packet ) );

    // Verify with some payload
    PARAPET_HEADER_SET_PAYLOAD_COUNT( Packet.Header, 100 );
    ExpectedSize = sizeof(parapet_header_t) + 100;
    EXPECT_EQ( ExpectedSize, parapet_PacketSize( &Packet ) );
    EXPECT_EQ( 1, PARAPET_PACKET_HAS_PAYLOAD( Packet ) );

    // Verify with full payload
    PARAPET_HEADER_SET_PAYLOAD_COUNT( Packet.Header,
                                      PARAPET_MAX_NUM_PAYLOAD_BYTES );
    EXPECT_EQ( sizeof(parapet_header_t) + PARAPET_MAX_NUM_PAYLOAD_BYTES,
               parapet_PacketSize( &Packet ) );
    EXPECT_EQ( 1, PARAPET_PACKET_HAS_PAYLOAD( Packet ) );
}
