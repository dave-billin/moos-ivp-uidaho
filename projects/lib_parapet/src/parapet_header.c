//=============================================================================
/** @file parapet_header.c
 *
 * @brief
 *	Implementation of functions declared in parapet_header.h
 *
 * @author Dave Billin
 */
//=============================================================================

#include "parapet_config.h"
#include "parapet_header.h"
#include "parapet_descriptor.h"

#ifdef PARAPET_ENABLE_ASSERTS
#include <assert.h>
#include "stddef.h"
#endif



//=============================================================================
uint8_t CalculateHeaderChecksum( parapet_header_t* Header )
{
#ifdef PARAPET_ENABLE_HEADER_CHECKSUM
    #ifdef PARAPET_ENABLE_ASSERTS
    assert( Header != NULL );
    #endif

    uint8_t Offset;
    uint8_t Accum = 0;

    // Sum nibbles in Bytes 0..2 of the header
    for ( Offset = 0; Offset < 3; ++Offset )
    {
        uint8_t Byte = Header->u8[Offset];
        Accum +=  Byte & 0x0f;
        Accum +=  (Byte >> 4) & 0x0f;
    }

    // Add lower nibble of Byte 3
    Accum += Header->u8[3] & 0x0f;

    // Return the 2's compliment of the sum
    Accum = ~Accum + 1;
    return Accum & 0x0f;
#else
    return 0;
#endif
}


//=============================================================================
uint8_t HeaderChecksumIsValid( parapet_header_t* Header )
{
    return PARAPET_HEADER_CHECKSUM_IS_VALID( *Header );
}
