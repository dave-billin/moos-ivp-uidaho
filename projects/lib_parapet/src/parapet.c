//=============================================================================
/** @file parapet.c
 *
 * @brief
 *	Implementation of functions declared in parapet.h
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#include <assert.h>
#include <stddef.h>

#include "parapet.h"


//=============================================================================
void parapet_BuildRequest( parapet_request_t* Packet, uint8_t Opcode,
                         uint8_t SourceID, uint8_t DestID, uint8_t SequenceID,
                         uint16_t DescriptorID )
{
    #ifdef PARAPET_ENABLE_ASSERTS
    assert( Packet != NULL );
    #endif

    // Copy in header fields
    Packet->Header.u32 =
       ( (uint32_t)(Opcode & 0x07) ) |
       ( (uint32_t)(SourceID & 0x0f) << HEADER_OFFSET_SOURCE_ID ) |
       ( (uint32_t)(DestID & 0x0f) << HEADER_OFFSET_DEST_ID ) |
       ( (uint32_t)(SequenceID & 0xf) << HEADER_OFFSET_SEQUENCE_ID );

    // Set up payload count and set Q (query) flag
    Packet->Header.u32 |=
       ( ( sizeof(parapet_descriptor_t) << HEADER_OFFSET_PAYLOAD_COUNT )
               & HEADER_MASK_PAYLOAD_COUNT ) | HEADER_MASK_Q;

    #ifdef PARAPET_ENABLE_HEADER_CHECKSUM
    POPULATE_HEADER_CHECKSUM( Packet->Header );
    #endif

    Packet->Descriptor.u32 = (uint32_t)( DescriptorID & 0xffff );
}



//=============================================================================
void parapet_BuildResponse( parapet_response_t* Packet,
                          parapet_header_t* RequestHeader,
                          uint8_t SourceID,
                          uint8_t IsAck )
{
    #ifdef PARAPET_ENABLE_ASSERTS
    assert( Packet != NULL );
    assert( RequestHeader != NULL );
    parapet_opcode_t Opcode = PARAPET_HEADER_GET_OPCODE( *RequestHeader );
    #endif

    parapet_header_t* Header = &Packet->Header;

    // Copy opcode and sequence ID from the request packet
    Header->u32 = RequestHeader->u32 &
                    ( HEADER_MASK_OPCODE | HEADER_MASK_SEQUENCE_ID );

    // Copy destination ID of the request to source ID of the response
    Header->u32 |= ( ( RequestHeader->u32 & HEADER_MASK_SOURCE_ID )
                        << (HEADER_OFFSET_DEST_ID - HEADER_OFFSET_SOURCE_ID) );

    // Set the Source ID of the response packet
    Header->u32 |= SourceID << HEADER_OFFSET_SOURCE_ID;

    // Set ACK flag as indicated
    if ( IsAck != 0 )
    {
        Header->u32 |= HEADER_MASK_A;
    }

    // Calculate header checksum
    #ifdef PARAPET_ENABLE_HEADER_CHECKSUM
    Header->u32 |=
           (uint32_t)CalculateHeaderChecksum(Header) << HEADER_OFFSET_CHECKSUM;
    #endif
}


//=============================================================================
uint16_t parapet_AddPayloadData( parapet_packet_t* Packet, void* PayloadData,
                         uint16_t NumPayloadBytes )
{
    #ifdef PARAPET_ENABLE_ASSERTS
    assert( Packet != NULL );
    assert( PayloadData != NULL );
    #endif

    parapet_header_t* Header = &Packet->Header;
    uint16_t PayloadBytesNow = PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( *Header );

    uint16_t ret_val = 0;
    uint16_t PayloadSizeAfterAppend = PayloadBytesNow + NumPayloadBytes;
    if ( PayloadSizeAfterAppend <= PARAPET_MAX_NUM_PAYLOAD_BYTES )
    {
        // Set up write pointer
        uint8_t* SourceByte = (uint8_t*)PayloadData;
        uint8_t* PacketByte = (uint8_t*)Header + sizeof(parapet_header_t)
                                               + PayloadBytesNow;

        // Copy payload data into the packet
        while ( NumPayloadBytes-- > 0 )
        {
            *PacketByte++ = *SourceByte++;
        }

        // Update payload Byte count in the header
        PARAPET_HEADER_SET_PAYLOAD_COUNT( *Header, PayloadSizeAfterAppend );

        // Recalculate header checksum
        POPULATE_HEADER_CHECKSUM( *Header );

        // Return the total length of the packet in Bytes
        ret_val = PayloadSizeAfterAppend + sizeof(parapet_header_t);
    }

    return ret_val;
}


//=============================================================================
uint16_t parapet_PacketSize( parapet_packet_t* Packet )
{
    return   sizeof(parapet_header_t)
           + PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( Packet->Header );
}
