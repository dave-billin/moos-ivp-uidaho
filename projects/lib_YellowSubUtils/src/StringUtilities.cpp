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
/** @file StringUtilities.cpp
 *
 * @brief
 *   A short description of the file
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <cctype>
#include "StringUtilities.h"

using namespace std;


namespace YellowSubUtils
{


const string StringUtilities::empty;


//=============================================================================
StringUtilities::eStringToBoolResult StringUtilities::ExtractBoolean(
                                                   std::string const& Target,
                                                   bool CompareCaseSensitive )
{
    StringUtilities::eStringToBoolResult result = STRING2BOOL_NEITHER;

    string::size_type size = Target.size();
    if ( ( size == 4) || ( size == 5 ) )
    {
        string s = Target;
        if ( CompareCaseSensitive == false )
        {
            for ( string::iterator c = s.begin(); c != s.end(); c++ )
            {
                *c = toupper( *c );
            }
        }

        if ( s[0] == 'T' )
        {
            if ( s == "TRUE" )
            {
                result = STRING2BOOL_TRUE;
            }
        }
        else
        {
            if ( s == "FALSE" )
            {
                result = STRING2BOOL_FALSE;
            }
        }
    }

    return result;
}


}   // END namespace YellowSubUtils
