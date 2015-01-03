//=============================================================================
/** @file parapet.h
 *
 * @brief
 *	Declaration of common functions for operating on parapet packets
 *
 * @author Dave Billin
 */
//=============================================================================

#ifndef PARAPET_H_
#define PARAPET_H_

#include <stdint.h>
#include "parapet_config.h"
#include "parapet_header.h"     // parapet_header_t and header constants/macros
#include "parapet_descriptor.h" // Descriptor word functionality
//#include "parapet_block_data.h" // ASCII/binary block data

#ifdef __cplusplus
namespace parapet
{
#endif

//-----------------------------------------------------------------------------
/** @struct parapet_request_t
 *  @brief
 *     Data structure describing a parapet request packet
 */
typedef struct
{
    parapet_header_t Header;            // Packet header
    parapet_descriptor_t Descriptor;    // Descriptor word
    uint8_t Payload[ PARAPET_MAX_NUM_PAYLOAD_BYTES - 4 ];  // Request Payload

} parapet_request_t;


//-----------------------------------------------------------------------------
/** @struct parapet_short_request_t
 *  @brief
 *     Data structure describing a parapet request packet that contains only
 *     a descriptor word and no payload.
 *
 *  @remarks
 *     In order to prevent accidentally adding payload data to
 *     parapet_short_request_t structures, it is necessary to explicitly cast
 *     them to parapet_packet_t structures in order to use them with parapet
 *     API functions.
 */
typedef struct
{
    parapet_header_t Header;            // Packet header
    parapet_descriptor_t Descriptor;    // Descriptor word
    uint8_t Payload[ PARAPET_MAX_NUM_PAYLOAD_BYTES - 4 ];  // Request Payload

} parapet_short_request_t;


//-----------------------------------------------------------------------------
/** @struct parapet_response_t
 *  @brief
 *     Data structure describing a parapet response packet
 */
typedef struct
{
    parapet_header_t Header;
    uint8_t Payload[ PARAPET_MAX_NUM_PAYLOAD_BYTES ];  // Response payload

} parapet_response_t;


//-----------------------------------------------------------------------------
/** @struct parapet_short_response_t
 *  @brief
 *     Data structure describing a parapet response packet that contains no
 *     payload data
 *
  *  @remarks
 *     In order to prevent accidentally trying to read payload data from
 *     parapet_short_request_t structures, it is necessary to explicitly cast
 *     them to parapet_packet_t structures in order to use them with parapet
 *     API functions.
 */
typedef struct
{
    parapet_header_t Header;
    uint8_t Payload[ PARAPET_MAX_NUM_PAYLOAD_BYTES ];  // Response payload

} parapet_short_response_t;


//-----------------------------------------------------------------------------
/** @union parapet_packet_t
 *  @brief
 *     A union of parapet packet types for ease of pointer passing
 */
typedef union
{
    parapet_header_t   Header;
    parapet_request_t  Request;
    parapet_short_request_t ShortRequest;
    parapet_response_t Response;
    parapet_short_response_t ShortResponse;
    uint8_t u8[ PARAPET_MAX_BYTES_PER_PACKET ];

} parapet_packet_t;



//=============================================================================
/** @return non-zero if a specified packet contains payload data; else zero
 *  @param _packet_    Packet to examine */
#define PARAPET_PACKET_HAS_PAYLOAD( _packet_ ) \
    ( ( (_packet_).Header.u32 & HEADER_MASK_PAYLOAD_COUNT ) != 0 )



//=============================================================================
/** @addtogroup parapet_packet_functions Packet Functions
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
/** Builds a parapet request packet
 *
 * @details
 *  This function populates the header and Descriptor of a parapet request
 *  packet with a specified opcode, source/destination ID's, sequence ID, and
 *  Descriptor ID/address value.  The generated request packet is initialized
 *  to contain no payload data beyond the Descriptor word.  Additional payload
 *  data may be added to the end of the packet using the parapet_AddPayloadData()
 *  function.
 *
 * @param [out] Packet      Pointer to the packet to operate on
 * @param [in] Opcode       Packet opcode
 * @param [in] SourceID     Packet SOURCE_ID field value
 * @param [in] DestID       Packet DEST_ID field value
 * @param [in] SequenceID   Packet SEQUENCE_ID field value
 * @param [in] DescriptorID Value of the ID field in the packet Descriptor word
 *
 * @see parapet_AddPayloadData
 */
void parapet_BuildRequest( parapet_request_t* Packet, uint8_t Opcode,
                           uint8_t SourceID, uint8_t DestID,
                           uint8_t SequenceID, uint16_t DescriptorID );


//=============================================================================
/** Builds a parapet response packet
 *
 * @details
 *  This function populates the header of a parapet response packet based on
 *  the header of a corresponding request packet.  The generated response
 *  packet contains no payload data.  Additional payload data may be added
 *  using the parapet_AddPayloadData() function.
 *
 * @param [out] Packet
 *   Pointer to the packet to operate on
 *
 * @param [in] RequestHeader
 *   Pointer to the header of the request packet being responded to
 *
 * @param [in] SourceID
 *   Value to place in the response packet's SOURCE_ID field
 *
 * @see parapet_AddPayloadData
 */
void parapet_BuildResponse( parapet_response_t* Packet,
                            parapet_header_t* RequestHeader,
                            uint8_t SourceID, uint8_t IsAck );


//=============================================================================
/** Adds payload data to the end of a packet
 *
 * @param [in] Packet
 *   Pointer to the packet to operate on
 *
 * @param [in] PayloadData
 *   Pointer to payload data that should be added to the end of the packet
 *
 * @param [in] NumPayloadBytes
 *   Number of payload data Bytes pointed to by PayloadData
 *
 * @return
 *   Non-zero to indicate the total packet size in Bytes after payload data was
 *   added; else zero if payload data was not added (possibly because
 *   PayloadData is NULL or because the requested operation would cause the
 *   packet to exceed the maximum allowed packet length)
 */
uint16_t parapet_AddPayloadData( parapet_packet_t* Packet, void* PayloadData,
                                 uint16_t NumPayloadBytes );


//=============================================================================
/** @return the total length of a parapet packet in Bytes
 *
 * @param [in] Packet
 *   Pointer to the packet whose size is to be obtained
 */
uint16_t parapet_PacketSize( parapet_packet_t* Packet );


#ifdef __cplusplus
}   // END extern "C" {
}   // END namespace parapet
#endif


/// @} */




#endif /* PARAPET_H_ */
