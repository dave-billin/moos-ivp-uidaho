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
/** @file test_parapet_descriptor.cpp
 *
 * @brief
 *   Unit tests of parapet_header.h/.c modules
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#include <stdint.h>
#include "gtest/gtest.h"
#include "parapet_descriptor.h"

using namespace parapet;

//=============================================================================
TEST( test_parapet_descriptor, test_getter_macros )
{
    parapet_descriptor_t Descriptor;
    Descriptor.u32 = ( 0xaaaaUL << DESCRIPTOR_OFFSET_USER_BITS ) | 0x5555UL;

    uint16_t id = PARAPET_DESCRIPTOR_GET_ID( Descriptor );
    EXPECT_EQ( 0x5555UL, id );

    uint16_t user_bits = PARAPET_DESCRIPTOR_GET_USER_BITS( Descriptor );
    EXPECT_EQ( 0xaaaaUL, user_bits );
}


//=============================================================================
TEST( test_parapet_descriptor, test_setter_macros )
{
    parapet_descriptor_t Descriptor;


    Descriptor.u32 = 0;
    PARAPET_DESCRIPTOR_SET_ID( Descriptor, 0xffff );
    EXPECT_EQ( 0xffff, Descriptor.u32 );

    Descriptor.u32 = 0;
    PARAPET_DESCRIPTOR_SET_USER_BITS( Descriptor, 0xffff );
    EXPECT_EQ( (0xffff << DESCRIPTOR_OFFSET_USER_BITS), Descriptor.u32 );
}



