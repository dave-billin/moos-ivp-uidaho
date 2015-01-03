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
/** @file MOOSDBDoubleProxy.cpp
 *
 * @brief
 *  Implementation of the MOOSDBDoubleProxy class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================

#include "MOOSDBDoubleProxy.h"

using namespace std;


namespace YellowSubUtils
{

//=============================================================================
MOOSDBDoubleProxy::MOOSDBDoubleProxy()
 : MOOSDBVariableProxy( PUBLISHES, DOUBLE, std::string() ),
   m_DoubleValue(0.0)
{
}


//=============================================================================
MOOSDBDoubleProxy::MOOSDBDoubleProxy( e_ProxyModes Mode,
                                      string const& MOOSDBVariableName,
                                      double InitialValue,
                                      bool IsFresh )
 : MOOSDBVariableProxy( Mode, DOUBLE, MOOSDBVariableName, IsFresh ),
   m_DoubleValue(InitialValue)
{
}


//=============================================================================
MOOSDBDoubleProxy::MOOSDBDoubleProxy( MOOSDBDoubleProxy const& other )
 : MOOSDBVariableProxy( other ),
   m_DoubleValue( other.m_DoubleValue )
{
}


//=============================================================================
MOOSDBDoubleProxy::~MOOSDBDoubleProxy()
{
}



//=============================================================================
bool MOOSDBDoubleProxy::Publish( CMOOSCommClient& CommsClient )
{
    bool RetVal = false;

    if ( this->Publishes() )
    {
        RetVal = CommsClient.Notify( m_MOOSDBVariableName, m_DoubleValue,
                                     m_TimeLastWritten );
    }

    return RetVal;
}


//=============================================================================
bool MOOSDBDoubleProxy::Update( CMOOSMsg const& MOOSDBMessage )
{
    bool GotNewData = false;

    if ( this->Subscribes() && (MOOSDBMessage.m_cDataType == MOOS_DOUBLE) )
    {
        m_DoubleValue = MOOSDBMessage.m_dfVal;
        m_TimeLastWritten = MOOSDBMessage.m_dfTime;

        if ( m_MOOSDBVariableName.empty() )
        {
            m_MOOSDBVariableName = MOOSDBMessage.m_sKey;
        }

        m_IsFresh = true;
        GotNewData = true;
    }

    return GotNewData;
}


} /* namespace YellowSubUtils */
