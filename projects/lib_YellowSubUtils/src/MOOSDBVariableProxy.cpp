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
/** @file MOOSDBVariableProxy.cpp
 *
 * @brief
 *  Implementation of the MOOSDBVariableProxy class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

#include "MOOSDBVariableProxy.h"


namespace YellowSubUtils
{


//=============================================================================
const char* MOOSDBVariableProxy::s_CannotAccessAsString =
                              "Proxy contents cannot be accessed as a string!";

const char* MOOSDBVariableProxy::s_CannotAccessAsDouble =
                              "Proxy contents cannot be accessed as a double!";

const char* MOOSDBVariableProxy::s_CannotAccessAsBinaryData =
                           "Proxy contents cannot be accessed as binary data!";


//=============================================================================
MOOSDBVariableProxy::MOOSDBVariableProxy( e_ProxyModes Mode,
                                        e_VariableDataTypes Type,
                                        std::string const& MOOSDBVariableName,
                                        bool ShouldMarkAsFresh )
: m_ProxyMode( Mode & (PUBLISHES | SUBSCRIBES) ),
  m_IsFresh( ShouldMarkAsFresh ), m_MOOSDataType( Type ),
  m_MOOSDBVariableName(MOOSDBVariableName),
  m_CanBeRemapped( true ), m_IsRemapped( false ), m_IsDisabled( false ),
  m_TimeLastWritten( 0.0 )
{
    if ( ShouldMarkAsFresh )
    {
        m_TimeLastWritten = -1.0;
    }
}


//=============================================================================
MOOSDBVariableProxy::MOOSDBVariableProxy( MOOSDBVariableProxy const& other )
  : m_ProxyMode( other.m_ProxyMode ),
    m_IsFresh( other.m_IsFresh ),
    m_MOOSDataType( other.m_MOOSDataType ),
    m_MOOSDBVariableName( other.m_MOOSDBVariableName ),
    m_OldMOOSDBVariableName( other.m_OldMOOSDBVariableName ),
    m_CanBeRemapped( true ),
    m_IsRemapped( other.m_IsRemapped ),
    m_IsDisabled( other.m_IsDisabled),
    m_TimeLastWritten( other.m_IsDisabled )
{
}



//=============================================================================
MOOSDBVariableProxy::~MOOSDBVariableProxy( void )
{
}


//=============================================================================
MOOSDBVariableProxy& MOOSDBVariableProxy::operator= (
                                             MOOSDBVariableProxy const& other )
{
    if ( &other != this )
    {
        m_ProxyMode = other.m_ProxyMode;
        m_IsFresh = other.m_IsFresh;
        m_MOOSDataType = other.m_MOOSDataType;
        m_MOOSDBVariableName = other.m_MOOSDBVariableName;
        m_OldMOOSDBVariableName = other.m_OldMOOSDBVariableName;
        m_CanBeRemapped = other.m_CanBeRemapped;
        m_IsRemapped = other.m_IsRemapped;
        m_IsDisabled = other.m_IsDisabled;
        m_TimeLastWritten = other.m_TimeLastWritten;
    }
    return *this;
}



//=============================================================================
bool MOOSDBVariableProxy::ApplyMissionFileMapping(
                                           CProcessConfigReader& ConfigReader )
{
    bool ProxyWasRemapped = false;

    if ( m_CanBeRemapped )
    {
        const char* suffix = ( this->Publishes() ) ?
                                    "_PUBLISHTO" : "_SUBSCRIBETO";

        std::string QueryString = m_MOOSDBVariableName + suffix;


        std::string ConfigString;
        bool ConfigWasFound = ConfigReader.GetConfigurationParam( QueryString,
                                                                ConfigString );

        // If a configuration was found for the configured variable name
        if ( ConfigWasFound )
        {
            m_IsRemapped = true;
            if ( ConfigString == "DISABLED" )
            {
                // Disable the object
                m_IsDisabled = true;
            }
            else
            {   // Re-map the object to a new variable name
                m_OldMOOSDBVariableName = m_MOOSDBVariableName;
                m_MOOSDBVariableName = ConfigString;
            }
        }

        ProxyWasRemapped = ConfigWasFound;
    }

    return ProxyWasRemapped;
}



//=============================================================================
std::string MOOSDBVariableProxy::Print( void ) const
{
    static const char* szModes[] = { "DISABLED", "PUBLISH", "SUBSCRIBE",
                                     "PUBLISH + SUBSCRIBE" };
    static const char* szTypes[] = { "double", "string", "binary" };

    std::ostringstream oss;
    std::streamsize DefaultWidth = oss.width();

    const char* Prefix = ( m_IsRemapped ) ? "* " : "  ";

    int Mode = ( m_IsDisabled ) ? DISABLED : m_ProxyMode;

    oss << Prefix << std::setw(32) << std::setiosflags( std::ios::left )
        << m_MOOSDBVariableName;

    oss << " : " << std::setw(10) << szModes[Mode]
        << " as <" << std::setw(DefaultWidth)
        << szTypes[m_MOOSDataType] << "> ";

    return oss.str();
}


}   // END namespace YellowSubUtils
