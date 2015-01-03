//=============================================================================
/*  Copyright (C) 2013  Dave Billin

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-----------------------------------------------------------------------------
/** @file test_parapet_header.cpp
 *
 * @brief
 *   Unit tests of parapet_header.h/.c modules
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#include <stdint.h>
#include "gtest/gtest.h"
#include "parapet_header.h"

using namespace parapet;

//=============================================================================
TEST( test_parapet_header, test_getter_macros )
{
    parapet_header_t Header;
    Header.u32 =
            OPCODE_READ_PARAM |
            ( 0 << HEADER_OFFSET_Q ) |
            ( 0x05 << HEADER_OFFSET_SOURCE_ID ) |
            ( 0x0a << HEADER_OFFSET_DEST_ID ) |
            ( 0x0f << HEADER_OFFSET_SEQUENCE_ID ) |
            ( 0 << HEADER_OFFSET_A ) |
            ( 0x07ff << HEADER_OFFSET_PAYLOAD_COUNT ) |
            ( 0x05 << HEADER_OFFSET_CHECKSUM );

    parapet_opcode_t Opcode = PARAPET_HEADER_GET_OPCODE( Header );
    EXPECT_EQ( OPCODE_READ_PARAM, Opcode );

    EXPECT_FALSE( PARAPET_HEADER_Q_FLAG_IS_SET( Header ) );
    EXPECT_TRUE( PARAPET_HEADER_Q_FLAG_IS_CLEARED( Header ) );

    uint8_t SourceID = PARAPET_HEADER_GET_SOURCE_ID( Header );
    EXPECT_EQ( 0x05, SourceID );

    uint8_t DestID = PARAPET_HEADER_GET_DEST_ID( Header );
    EXPECT_EQ( 0x0a, DestID );

    uint8_t SeqID = Header.u32 & HEADER_MASK_SEQUENCE_ID;
    SeqID >>= HEADER_OFFSET_SEQUENCE_ID;
    SeqID = PARAPET_HEADER_GET_SEQUENCE_ID( Header );
    EXPECT_EQ( 0x0f, SeqID );

    EXPECT_FALSE( PARAPET_HEADER_A_FLAG_IS_SET( Header ) );
    EXPECT_TRUE( PARAPET_HEADER_A_FLAG_IS_CLEARED( Header ) );

    uint16_t PayloadCount = PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( Header );
    EXPECT_EQ( 0x07ff, PayloadCount );

    uint8_t Checksum = PARAPET_HEADER_GET_CHECKSUM( Header );
    EXPECT_EQ( 0x05, Checksum );
}


//=============================================================================
TEST( test_parapet_header, test_setter_macros )
{
    parapet_header_t Header;
    Header.u32 = 0;

    // Test opcode set macro
    PARAPET_HEADER_SET_OPCODE( Header, OPCODE_WRITE_PARAM );
    EXPECT_EQ( static_cast<uint32_t>( OPCODE_WRITE_PARAM ),
               Header.u32 );


    // Test R flag macros
    Header.u32 = 0;
    PARAPET_HEADER_ASSERT_Q_FLAG( Header );
    EXPECT_EQ( (1 << HEADER_OFFSET_Q), Header.u32 );

    Header.u32 = 0xffffffff;
    PARAPET_HEADER_CLEAR_Q_FLAG( Header );
    EXPECT_EQ( ( 0xffffffff & ~(1 << HEADER_OFFSET_Q) ), Header.u32 );


    // Test Source and Dest ID set macros
    Header.u32 = 0;
    PARAPET_HEADER_SET_SOURCE_ID( Header, 0x05 );
    EXPECT_EQ( (0x05 << HEADER_OFFSET_SOURCE_ID), Header.u32 );

    Header.u32 = 0;
    PARAPET_HEADER_SET_DEST_ID( Header, 0x05 );
    EXPECT_EQ( (0x05 << HEADER_OFFSET_DEST_ID), Header.u32 );

    Header.u32 = 0;
    PARAPET_HEADER_SET_SEQUENCE_ID( Header, 0x0f );
    EXPECT_EQ( (0x0f << HEADER_OFFSET_SEQUENCE_ID), Header.u32 );


    // Test A flag macros
    Header.u32 = 0;
    PARAPET_HEADER_ASSERT_A_FLAG( Header );
    EXPECT_EQ( (1 << HEADER_OFFSET_A), Header.u32 );

    Header.u32 = 0xffffffff;
    PARAPET_HEADER_CLEAR_A_FLAG( Header );
    EXPECT_EQ( ( 0xffffffff & ~(1 << HEADER_OFFSET_A) ), Header.u32 );


    Header.u32 = 0;
    PARAPET_HEADER_SET_PAYLOAD_COUNT( Header, 0x07ff );
    EXPECT_EQ( ( 0x07ff << HEADER_OFFSET_PAYLOAD_COUNT ), Header.u32 );

    Header.u32 = 0;
    PARAPET_HEADER_SET_CHECKSUM( Header, 0x0a );
    EXPECT_EQ( ( 0x0a << HEADER_OFFSET_CHECKSUM ), Header.u32 );

}


//=============================================================================
TEST( test_parapet_header, test_utility_functions )
{
    //-----------------------------------
    // Test CalculateHeaderChecksum
    //-----------------------------------
    parapet_header_t Header;

    #ifdef PARAPET_ENABLE_HEADER_CHECKSUM
        // CASE 0x02323232:
        // Header nibbles sum to 17 --> 17 mod 16 = 1
        // (4-bit) 2's compliment of 1 is 0xf  --> CHECKSUM = 0x0f
        Header.u32 = 0x02323232;
        uint8_t CalculatedChecksum = CalculateHeaderChecksum( &Header );
        EXPECT_EQ( 0x0f, CalculatedChecksum );


    #else
        // Verify CalculateHeaderChecksum when checksums are disabled
        Header.u32 = 0x0f323232;     // Header Bytes sum to 0xa5
        EXPECT_EQ( 0, CalculateHeaderChecksum( &Header ) );
    #endif

    //-------------------------------------------
    // Test HeaderChecksumIsValid macro
    //-------------------------------------------
    Header.u32 = 0xf2323232;
    EXPECT_NE( 0, PARAPET_HEADER_CHECKSUM_IS_VALID( Header ) );

}
