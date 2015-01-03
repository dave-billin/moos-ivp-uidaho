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
/** @file StringUtilities.h
 *
 * @brief
 *   Utility functions used to operate on std::string objects
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef STRINGUTILITIES_H_
#define STRINGUTILITIES_H_

#include <string>

namespace YellowSubUtils
{

//=============================================================================
/** @class StringUtilities
 * @brief
 *   Utility functions for working with std::string objects
 */
//=============================================================================
class StringUtilities
{
public:

    enum eStringToBoolResult
    {
        STRING2BOOL_FALSE = 0,
        STRING2BOOL_TRUE,
        STRING2BOOL_NEITHER
    };

    //=========================================================================
    /** Returns a value from eStringToBoolResult based on the contents of a
     *  specified std::string object
     *
     * @param [in] Target
     *   Reference to a std::string object to be checked
     *
     * @param [in] CompareCaseSensitive
     *   true to match only upper-case "TRUE" and "FALSE"; else false
     *   (default) to also match the strings "true" and "false"
     */
    static eStringToBoolResult ExtractBoolean( std::string const& Target,
                                           bool CompareCaseSensitive = false );


    //=========================================================================
    /** @return
     *   true if the string indicates a boolean value: either "TRUE" or "FALSE"
     *
     * @param [in] Target
     *   Reference to the std::string object to be tested
     *
     * @param [in] CompareCaseSensitive
     *   true to match only upper-case "TRUE" and "FALSE"; else false
     *   (default) to also match the strings "true" and "false"
     */
    static bool IsBoolean( std::string const& Target,
                           bool CompareCaseSensitive = false );


    //=========================================================================
    /** @return
     *   true if a specified std::string object contains only the word "TRUE",
     *   or false otherwise
     *
     * @param [in] CompareCaseSensitive
     *   true to match only upper-case "TRUE"; else false (default) to also
     *   match lower-case "true"
     */
    static bool StringToBool( std::string const& Target,
                              bool CompareCaseSensitive = false );


    //=========================================================================
    /** @return
     *   A std::string object containing the boolean representation of a
     *   specified bool value: either "TRUE" or "FALSE"
     *
     * @param [in] Value
     *   Boolean value to convert to a string
     */
    static std::string BoolToString( bool Value )
    { return (Value == true) ? "TRUE" : "FALSE"; }

    static const std::string empty;   /**< An empty string object */

private:


};



//=============================================================================
inline bool StringUtilities::IsBoolean( std::string const& Target,
                                 bool CompareCaseSensitive )
{
    return ExtractBoolean(Target, CompareCaseSensitive) != STRING2BOOL_NEITHER;
}


//=============================================================================
inline bool StringUtilities::StringToBool( std::string const& Target,
                                           bool CompareCaseSensitive )
{
    return
        ExtractBoolean(Target, CompareCaseSensitive) == STRING2BOOL_TRUE;
}



} /* namespace YellowSubUtils */
#endif /* STRINGUTILITIES_H_ */
