/*
 * MOOSDBStringProxy.cpp
 *
 *  Created on: Mar 23, 2013
 *      Author: bill5510
 */

#include "MOOSDBStringProxy.h"

namespace YellowSubUtils
{


//=============================================================================
MOOSDBStringProxy::MOOSDBStringProxy()
 : MOOSDBVariableProxy( PUBLISHES, DOUBLE, std::string() )
{
}


//=============================================================================
MOOSDBStringProxy::MOOSDBStringProxy( e_ProxyModes Mode,
                                      std::string const& MOOSDBVariableName,
                                      std::string const& InitialContent,
                                      bool IsFresh )
 : MOOSDBVariableProxy( Mode, STRING, MOOSDBVariableName, IsFresh),
   m_StringValue(InitialContent)
{
}


//=============================================================================
MOOSDBStringProxy::MOOSDBStringProxy( MOOSDBStringProxy const& other )
 : MOOSDBVariableProxy( other ),
   m_StringValue( other.m_StringValue )
{
}


//=============================================================================
MOOSDBStringProxy::~MOOSDBStringProxy()
{
}



//=============================================================================
bool MOOSDBStringProxy::Publish( CMOOSCommClient& CommsClient )
{
    bool RetVal = false;

    if ( this->Publishes() )
    {
        RetVal = CommsClient.Notify( m_MOOSDBVariableName, m_StringValue,
                                     m_TimeLastWritten );
    }

    return RetVal;
}


//=============================================================================
bool MOOSDBStringProxy::Update( CMOOSMsg const& MOOSDBMessage )
{
    bool GotNewData = false;

    if ( this->Subscribes() && (MOOSDBMessage.m_cDataType == MOOS_STRING) )
    {
        m_StringValue = MOOSDBMessage.m_sVal;
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
