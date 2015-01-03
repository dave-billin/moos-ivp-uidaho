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
/** @file UT_StringUtilities.cpp
 *
 * @brief
 *   Unit tests for the StringUtilities class
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include "gtest/gtest.h"
#include "StringUtilities.h"

using namespace std;
using namespace YellowSubUtils;


//=============================================================================
TEST( Test_StringUtilities, test_ExtractBool )
{
    const string ucTrue("TRUE");
    const string ucFalse("FALSE");
    const string lcTrue("true");
    const string lcFalse("false");
    const string Nonsense("NoNSenSe");

    // Test with case-sensitive comparison
    EXPECT_EQ( StringUtilities::STRING2BOOL_TRUE,
               StringUtilities::ExtractBoolean(ucTrue, true) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_FALSE,
               StringUtilities::ExtractBoolean(ucFalse, true) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
               StringUtilities::ExtractBoolean(lcTrue, true) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
               StringUtilities::ExtractBoolean(lcFalse, true) );

    // Test against nonsense string
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
                   StringUtilities::ExtractBoolean( Nonsense, true) );

    // Test against empty string
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
                   StringUtilities::ExtractBoolean( StringUtilities::empty,
                                                    true) );

    // Test with case-insensitive comparison
    EXPECT_EQ( StringUtilities::STRING2BOOL_TRUE,
               StringUtilities::ExtractBoolean(ucTrue, false) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_FALSE,
               StringUtilities::ExtractBoolean(ucFalse, false) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_TRUE,
               StringUtilities::ExtractBoolean(lcTrue, false) );
    EXPECT_EQ( StringUtilities::STRING2BOOL_FALSE,
               StringUtilities::ExtractBoolean(lcFalse, false) );

    // Test against nonsense string
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
                   StringUtilities::ExtractBoolean( Nonsense, false) );

    // Test against empty string
    EXPECT_EQ( StringUtilities::STRING2BOOL_NEITHER,
                   StringUtilities::ExtractBoolean( StringUtilities::empty,
                                                    false) );
}



//=============================================================================
TEST( Test_StringUtilities, test_IsBoolean )
{
    const string ucTrue("TRUE");
    const string ucFalse("FALSE");
    const string lcTrue("true");
    const string lcFalse("false");
    const string Nonsense("NoNSenSe");

    // Test case-sensitive comparison
    EXPECT_TRUE( StringUtilities::IsBoolean( ucTrue, true ) );
    EXPECT_TRUE( StringUtilities::IsBoolean( ucFalse, true ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( lcTrue, true ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( lcFalse, true ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( StringUtilities::empty, true ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( Nonsense, true ) );

    // Test case-insensitive comparison
    EXPECT_TRUE( StringUtilities::IsBoolean( ucTrue, false ) );
    EXPECT_TRUE( StringUtilities::IsBoolean( ucFalse, false ) );
    EXPECT_TRUE( StringUtilities::IsBoolean( lcTrue, false ) );
    EXPECT_TRUE( StringUtilities::IsBoolean( lcFalse, false ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( StringUtilities::empty, false ) );
    EXPECT_FALSE( StringUtilities::IsBoolean( Nonsense, false ) );
}


//=============================================================================
TEST( Test_StringUtilities, test_StringToBool )
{
    const string ucTrue("TRUE");
    const string ucFalse("FALSE");
    const string lcTrue("true");
    const string lcFalse("false");
    const string Nonsense("NoNSenSe");

    // Test case-sensitive comparison
    EXPECT_TRUE( StringUtilities::StringToBool( ucTrue, true ) );
    EXPECT_FALSE( StringUtilities::StringToBool( ucFalse, true ) );
    EXPECT_FALSE( StringUtilities::StringToBool( lcTrue, true ) );
    EXPECT_FALSE( StringUtilities::StringToBool( lcFalse, true ) );
    EXPECT_FALSE( StringUtilities::StringToBool( StringUtilities::empty, true ) );
    EXPECT_FALSE( StringUtilities::StringToBool( Nonsense, true ) );

    // Test case-insensitive comparison
    EXPECT_TRUE( StringUtilities::StringToBool( ucTrue, false ) );
    EXPECT_FALSE( StringUtilities::StringToBool( ucFalse, false ) );
    EXPECT_TRUE( StringUtilities::StringToBool( lcTrue, false ) );
    EXPECT_FALSE( StringUtilities::StringToBool( lcFalse, false ) );
    EXPECT_FALSE( StringUtilities::StringToBool( StringUtilities::empty, false ) );
    EXPECT_FALSE( StringUtilities::StringToBool( Nonsense, false ) );
}
