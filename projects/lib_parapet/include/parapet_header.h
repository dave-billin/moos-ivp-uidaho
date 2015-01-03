//=============================================================================
/** @file parapet_header.h
 *
 * @brief
 *	Declaration of a parapet packet header data structure along with associated
 *	constants, macros, and functions
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef PARAPET_HEADER_H_
#define PARAPET_HEADER_H_

#include <stdint.h>
#include "parapet_config.h"


#ifdef __cplusplus
namespace parapet
{
#endif

//=============================================================================
/// Header data structure that appears at the beginning of all parapet packets
typedef union
{
    uint32_t u32;
    uint16_t u16[ sizeof(uint32_t) / sizeof(uint16_t) ];
    uint8_t  u8[ sizeof(uint32_t) / sizeof(uint8_t) ];

} parapet_header_t;


enum
{
    PARAPET_MAX_NUM_PAYLOAD_BYTES =
            PARAPET_MAX_BYTES_PER_PACKET - sizeof(parapet_header_t)
};



//=============================================================================
/// Bit offsets of fields in a parapet packet header
enum
{
    HEADER_OFFSET_OPCODE = 0,       ///< Bit offset of OPCODE field
    HEADER_OFFSET_Q = 3,            ///< Bit offset of the Q (Request) flag
    HEADER_OFFSET_SOURCE_ID = 4,    ///< Bit offset of SOURCE_ID field
    HEADER_OFFSET_DEST_ID = 8,      ///< Bit offset of DEST_ID field
    HEADER_OFFSET_SEQUENCE_ID = 12, ///< Bit offset of SEQUENCE_ID field
    HEADER_OFFSET_A = 16,           ///< Bit offset of the A (Ack) flag
    HEADER_OFFSET_PAYLOAD_COUNT = 17,   ///< Bit offset of payload Byte count
    HEADER_OFFSET_CHECKSUM = 28         ///< Bit offset of CHECKSUM
};



//=============================================================================
/// Masks corresponding to parapet header fields
#define HEADER_MASK_OPCODE         0x00000007UL
#define HEADER_MASK_Q              0x00000008UL
#define HEADER_MASK_SOURCE_ID      0x000000f0UL
#define HEADER_MASK_DEST_ID        0x00000f00UL
#define HEADER_MASK_SEQUENCE_ID    0x0000f000UL
#define HEADER_MASK_A              0x00010000UL
#define HEADER_MASK_PAYLOAD_COUNT  0x0ffe0000UL
#define HEADER_MASK_CHECKSUM       0xf0000000UL



//=============================================================================
/// Values defined for the Opcode field of a parapet packet header
//=============================================================================
typedef enum
{
    OPCODE_EXECUTE = 0,     /**< Use dot requests that a device execute a
                                 specified function or operation */

    OPCODE_READ_PARAM = 1,  /**< Used to request that a device send the value
                                 of one or more parameters */

    OPCODE_WRITE_PARAM = 2, /**< Used to send or report the value of one or
                                 more parameters in a device */

    OPCODE_READ_BLOCK = 3,  /**< Used to request that a device send a specified
                                 block of ASCII or binary data */

    OPCODE_WRITE_BLOCK = 4, /**< Used to set or report a block of ASCII or
                                 binary data */

    OPCODE_RESERVED_5,      /**< Reserved for future use */
    OPCODE_RESERVED_6,      /**< Reserved for future use */
    OPCODE_RESERVED_7       /**< Reserved for future use */

} parapet_opcode_t;




//=============================================================================
/** @addtogroup parapet_header_read_macros Macros to read from a parapet Header
 * @brief
 *   A selection of macros that may be used to extract the value of fields in
 *   a parapet packet header
 *
 * @{
 */
//=============================================================================

//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_OPCODE
 * @brief Extracts the value of the Opcode from a parapet_header_t
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_OPCODE( _header_ ) \
    ( (parapet_opcode_t)( (_header_).u32 & HEADER_MASK_OPCODE ) )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_Q_FLAG_IS_SET
 *  @brief Evaluates to non-zero if the R flag in a parapet header is set
  * @param _header_   parapet_header_t struct to examine
 */
#define PARAPET_HEADER_Q_FLAG_IS_SET( _header_ ) \
    ( ( (_header_).u32 & HEADER_MASK_Q ) != 0 )

/** @def PARAPET_HEADER_Q_FLAG_IS_CLEARED
 *  @brief Evaluates to non-zero if the R flag in a parapet header is cleared
 * @param _header_   parapet_header_t struct to examine
 */
#define PARAPET_HEADER_Q_FLAG_IS_CLEARED( _header_ ) \
    ( (( (_header_).u32 & HEADER_MASK_Q ) == 0) )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_SOURCE_ID
 * @brief Extracts the SOURCE_ID field from a parapet header
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_SOURCE_ID( _header_ ) \
    ( (uint8_t)(( (_header_).u32 & HEADER_MASK_SOURCE_ID) \
                    >> HEADER_OFFSET_SOURCE_ID ) )


//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_DEST_ID
 * @brief Extracts the DEST_ID field from a parapet header
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_DEST_ID( _header_ ) \
    ( (uint8_t)(( (_header_).u32 & HEADER_MASK_DEST_ID) \
                    >> HEADER_OFFSET_DEST_ID ) )


//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_SEQUENCE_ID
 * @brief Extracts the DEST_ID field from a parapet header
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_SEQUENCE_ID( _header_ ) \
    ( (uint8_t)(( (_header_).u32 & HEADER_MASK_SEQUENCE_ID) \
                    >> HEADER_OFFSET_SEQUENCE_ID ) )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_A_FLAG_IS_SET
 *  @brief Evaluates to non-zero if the A flag in a parapet header is set
 * @param _header_   parapet_header_t struct to examine
 */
#define PARAPET_HEADER_A_FLAG_IS_SET( _header_ ) \
    ( (( (_header_).u32 & HEADER_MASK_A ) != 0) )

/** @def PARAPET_HEADER_A_FLAG_IS_CLEARED
 *  @brief Evaluates to non-zero if the A flag in a parapet header is cleared
 * @param _header_   parapet_header_t struct to examine
 */
#define PARAPET_HEADER_A_FLAG_IS_CLEARED( _header_ ) \
    ( (( (_header_).u32 & HEADER_MASK_A ) == 0) )




//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES
 * @brief Extracts the payload Byte count from a parapet_header_t
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( _header_ ) \
    ( (uint16_t)( \
        ( ( (_header_).u32 & HEADER_MASK_PAYLOAD_COUNT ) \
                    >> HEADER_OFFSET_PAYLOAD_COUNT ) ) )


//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_GET_CHECKSUM
 * @brief Extracts the header checksum from a parapet_header_t
 * @param _header_   parapet_header_t struct to extract from
 */
#define PARAPET_HEADER_GET_CHECKSUM( _header_ ) \
    ( (uint16_t)( \
        ( ( (_header_).u32 & HEADER_MASK_CHECKSUM ) \
                    >> HEADER_OFFSET_CHECKSUM ) ) )

/// @}
//=============================================================================




//=============================================================================
/** @addtogroup parapet_header_write_macros Macros to write header fields
 * @brief
 *   Macros used to write the value of fields in a parapet packet header
 * @{
 */
//=============================================================================



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_OPCODE
 * @brief Sets the Opcode field in a parapet_header_t struct
 * @param _header_   parapet_header_t struct to operate on
 * @param _opcode_   Opcode field value
 */
#define PARAPET_HEADER_SET_OPCODE( _header_, _opcode_ ) \
    ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_OPCODE) | ( (_opcode_) & 0x07 ) )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_ASSERT_Q_FLAG
 *  @brief Asserts the R flag in a parapet header, setting it to 1
 *  @param _header_   parapet_header_t struct to operate on
 */
#define PARAPET_HEADER_ASSERT_Q_FLAG( _header_ ) \
    ( (_header_).u32 |= HEADER_MASK_Q )

/** @def PARAPET_HEADER_CLEAR_Q_FLAG
 *  @brief Clears the R flag in a parapet header, setting it to 0
 *  @param _header_   parapet_header_t struct to operate on
 */
#define PARAPET_HEADER_CLEAR_Q_FLAG( _header_ )  \
    ( (_header_).u32 &= ~HEADER_MASK_Q )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_SOURCE_ID
 * @brief Sets the SOURCE_ID field in a parapet header
 * @param _header_      Struct to operate on
 * @param _device_id_   Value to apply to the SOURCE_ID field
 */
#define PARAPET_HEADER_SET_SOURCE_ID( _header_, _device_id_ ) \
    ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_SOURCE_ID) | \
            (( (_device_id_) & 0x0f) << HEADER_OFFSET_SOURCE_ID)  )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_DEST_ID
 * @brief Sets the DEST_ID field in a parapet header
 * @param _header_      Struct to operate on
 * @param _device_id_   Value to apply to the DEST_ID field
 */
#define PARAPET_HEADER_SET_DEST_ID( _header_, _device_id_ ) \
    ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_DEST_ID) | \
            (( (_device_id_) & 0x0f) << HEADER_OFFSET_DEST_ID)  )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_SEQUENCE_ID
 * @brief Sets the SEQUENCE_ID field in a parapet header
 * @param _header_      Struct to operate on
 * @param _seq_id_      Value to apply to the SEQUENCE_ID field
 */
#define PARAPET_HEADER_SET_SEQUENCE_ID( _header_, _seq_id_ ) \
    ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_SEQUENCE_ID) | \
            (( (_seq_id_) & 0x0f) << HEADER_OFFSET_SEQUENCE_ID)  )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_ASSERT_A_FLAG
 *  @brief Asserts the A flag in a parapet header, setting it to 1
 *  @param _header_   parapet_header_t struct to operate on
 */
#define PARAPET_HEADER_ASSERT_A_FLAG( _header_ ) \
    ( (_header_).u32 |= HEADER_MASK_A )

/** @def PARAPET_HEADER_CLEAR_A_FLAG
 *  @brief Clears the A flag in a parapet header, setting it to 0
 *  @param _header_   parapet_header_t struct to operate on
 */
#define PARAPET_HEADER_CLEAR_A_FLAG( _header_ )  \
    ( (_header_).u32 &= ~HEADER_MASK_A )



//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_PAYLOAD_COUNT
 * @brief Sets the payload Byte count field in a a parapet header
 * @param _header_      Struct to operate on
 * @param _seq_id_      Value to apply to the payload Byte count field
 */
#define PARAPET_HEADER_SET_PAYLOAD_COUNT( _header_, _num_bytes_ ) \
        ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_PAYLOAD_COUNT) | \
            (( (_num_bytes_) & 0x7ff) << HEADER_OFFSET_PAYLOAD_COUNT) )


//--------------------------------------------------------------------------
/** @def PARAPET_HEADER_SET_CHECKSUM
 * @brief Sets the header checksum value in a a parapet header
 * @param _header_      Struct to operate on
 * @param _seq_id_      Value to apply to the checksum field
 */
#define PARAPET_HEADER_SET_CHECKSUM( _header_, _num_bytes_ ) \
        ( (_header_).u32 = \
            ( (_header_).u32 & ~HEADER_MASK_CHECKSUM) | \
            (( (_num_bytes_) & 0x7ff) << HEADER_OFFSET_CHECKSUM ) )

/// @}
//=============================================================================





//=============================================================================
/** @def POPULATE_HEADER_CHECKSUM
 *  @brief
 *   Calculates the checksum of a parapet_header_t and stores it to the
 *   header's CHECKSUM field (evaluates to an empty expression if the symbol
 *   PARAPET_ENABLE_HEADER_CHECKSUM is not defined)
 *
 * @param _header_ [in]   Packet header to operate on
 */
#ifdef PARAPET_ENABLE_HEADER_CHECKSUM
    #define POPULATE_HEADER_CHECKSUM( _header_ ) \
        (_header_).u32 |= \
              ( (uint32_t)CalculateHeaderChecksum( &(_header_) ) \
                      << HEADER_OFFSET_CHECKSUM )
#else
    #define POPULATE_HEADER_CHECKSUM( _header_ )
#endif



//=============================================================================
/** @def PARAPET_HEADER_CHECKSUM_IS_VALID
 *  @brief
 *   A macro that validates a packet header's checksum
 *
 * @param Header [in]   Packet header to validate
 *
 * @return
 *   Non-zero if the packet header checksum was validated successfully; else
 *   zero if the header checksum failed (always returns 1 if the symbol
 *   PARAPET_ENABLE_HEADER_CHECKSUM is not defined)
 */
#ifdef PARAPET_ENABLE_HEADER_CHECKSUM
#define PARAPET_HEADER_CHECKSUM_IS_VALID( _header_ ) \
        ( ( (uint8_t)( (_header_).u32 >> HEADER_OFFSET_CHECKSUM ) \
                        == CalculateHeaderChecksum( &(_header_) ) ) )
#else
    #define PARAPET_HEADER_CHECKSUM_IS_VALID( _header_ )    1
#endif


#ifdef __cplusplus
extern "C" {
#endif



//=============================================================================
/** Calculates the value of a header's checksum field
 * @param Header    Header to calculate checksum from
 * @return          Calculated checksum in the lower nibble of an 8-bit value
 */
uint8_t CalculateHeaderChecksum( parapet_header_t* Header );


//=============================================================================
/** Validates a header's CHECKSUM field against its contents
 * @param Header    Header to calculate checksum from
 * @return          Calculated checksum in the lower nibble of an 8-bit value
 */
uint8_t HeaderChecksumIsValid( parapet_header_t* Header );


#ifdef __cplusplus
}   // END extern "C"
#endif

#ifdef __cplusplus
}   // END namespace parapet
#endif

#endif /* PARAPET_HEADER_H_ */
