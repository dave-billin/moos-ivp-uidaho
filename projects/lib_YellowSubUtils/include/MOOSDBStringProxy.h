//=============================================================================
/*    Copyright (C) 2013  Dave Billin

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
/** @file MOOSDBStringProxy.h
 *
 * @brief
 *  Declaration of the MOOSDBStringProxy class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================
#ifndef MOOSDBSTRINGPROXY_H_
#define MOOSDBSTRINGPROXY_H_

#include "MOOSDBVariableProxy.h"
#include "StringUtilities.h"

namespace YellowSubUtils
{

//=============================================================================
/** @class MOOSDBStringProxy
 *
 * @brief
 *   A MOOSDBVariableProxy that can be used to publish or subscribe to a
 *   string in the MOOS database
 */
//=============================================================================
class MOOSDBStringProxy : public MOOSDBVariableProxy
{
public:

    //=========================================================================
    /// Default constructor: creates a MOOSDBStringProxy object with publishing
    /// and subscribing disabled, an empty string content, and an empty target
    /// variable name
    MOOSDBStringProxy();


    //=========================================================================
    /// Creates a MOOSDBStringProxy that targets a specified variable in the
    /// MOOS database, and assigns it an initial content and freshness
    ///
    /// @param [in] Mode
    ///   Mode the object will function in (PUBLISH | SUBSCRIBE)
    ///
    /// @param [in] MOOSDBVariableName
    ///   Name of the variable in the MOOS database the object will target
    ///
    /// @param [in] InitialContent
    ///   Initial value to assign to the object
    ///
    /// @param [in] IsFresh
    ///   true (default) to mark the variable as fresh immediately; else false
    ///   to leave it as unfresh
    MOOSDBStringProxy( e_ProxyModes Mode,
                       std::string const& MOOSDBVariableName,
                       std::string const& InitialContent,
                       bool IsFresh = true );


    //=========================================================================
    /// Copy constructor
    MOOSDBStringProxy( MOOSDBStringProxy const& other );

    //=========================================================================
    /// Destructor
    virtual ~MOOSDBStringProxy();




    //=========================================================================
    /// If the object is configured to publish and the object's data is marked
    /// as fresh, this method publishes its data to the MOOS database
    ///
    /// @return
    ///   true if a value was published to the MOOS database
    ///
    /// @see MOOSDBVariableProxy::Publish()
    virtual bool Publish( CMOOSCommClient& CommsClient );


    //=========================================================================
    /// If the object is configured to publish, this method updates the
    /// object's string contents using data received from the MOOS Database
    ///
    /// @param [in] MOOSDBMessage
    ///   Data received from the MOOS database variable the object targets
    ///
    /// @return
    ///   true if the object's value was updated from the contents of
    ///   MOOSDBMessage; else false on failure or if the object is not
    ///   configured to subscribe
    ///
    /// @see MOOSDBVariableProxy::Publish()
    virtual bool Update( CMOOSMsg const& MOOSDBMessage );



    //=========================================================================
    /// Returns the object's string contents
    std::string const& AsString( void ) const { return m_StringValue; }


    //=========================================================================
    /// @return
    ///   true if the string indicates a boolean value: either "TRUE" or
    ///   "FALSE"
    ///
    /// @param [in] CompareCaseSensitive
    ///   true to match only upper-case "TRUE" and "FALSE"; else false
    ///   (default) to also match "true" and "false"
    bool ContainsBoolean( void ) const
    { return YellowSubUtils::StringUtilities::IsBoolean( m_StringValue ); }


    //=========================================================================
    /// @return
    ///   true if the object's string content is equal to "TRUE", and false
    ///   if the content equals "FALSE"
    ///
    /// @param [in] CompareCaseSensitive
    ///   true to match only upper-case "TRUE" and "FALSE"; else false
    ///   (default) to also match "true" and "false"
    ///
    /// @throw
    ///   A CMOOSException if the content does not equal "TRUE" or "FALSE" or
    ///   is empty
    bool AsBoolean( void ) const;


    //=========================================================================
    /// Sets the object's string contents
    virtual void Set( std::string const& StringValue, double Time );

    //=========================================================================
    /// Sets the object's string contents to indicate a boolean value: either
    /// "TRUE" or "FALSE"
    ///
    /// @param [in] Value
    ///   Boolean value to represent in the string
    ///
    /// @param [in] Time
    ///   A double indicating the MOOS Time to associate with the value
    void Set( bool Value, double Time );

protected:
    std::string m_StringValue;  ///< Value that will be published to the MOOSDB
};



//=============================================================================
inline bool MOOSDBStringProxy::AsBoolean( void ) const
{
    return StringUtilities::StringToBool( m_StringValue );
}

//=============================================================================
inline void MOOSDBStringProxy::Set( std::string const& StringValue,
                                    double Time )
{
    m_StringValue = StringValue;
    m_TimeLastWritten = Time;
}

//=============================================================================
inline void MOOSDBStringProxy::Set( bool IsTrue, double Time )
{
    m_StringValue = StringUtilities::BoolToString( IsTrue );
    m_TimeLastWritten = Time;
}

} /* namespace YellowSubUtils */
#endif /* MOOSDBSTRINGPROXY_H_ */
