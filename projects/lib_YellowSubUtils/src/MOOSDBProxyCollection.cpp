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
/** @file MOOSDBProxyCollection.cpp
 *
 * @brief
 *	Implementation of the MOOSDBProxyCollection class
 *
 * @author Dave Billin
 */
//=============================================================================
#include <cassert>
#include <sstream>
#include "MOOSDBProxyCollection.h"

using std::string;
using std::set;

namespace YellowSubUtils
{


//=============================================================================
MOOSDBProxyCollection::MOOSDBProxyCollection()
{
}

//=============================================================================
MOOSDBProxyCollection::~MOOSDBProxyCollection()
{
}


//=============================================================================
void MOOSDBProxyCollection::RemoveProxy( MOOSDBVariableProxy& Proxy )
{
    m_Collection.erase( &Proxy );

    // Remove any timers associated with the proxy
    std::map<MOOSDBVariableProxy*, ProxyTimer>::iterator timer =
                                            m_ProxyTimerMap.find( &Proxy );
    if ( timer != m_ProxyTimerMap.end() )
    {
        m_ProxyTimerMap.erase( timer );
    }
}


//=============================================================================
void MOOSDBProxyCollection::SetMinimumUpdatePeriod(
                                            MOOSDBVariableProxy const& Proxy,
                                            double MinPeriod,
                                            double InitialTime )
{
    // If you hit this assert, it's because you need to add the Proxy
    // to the collection before assigning a minimum update period
    assert( m_Collection.find( const_cast<MOOSDBVariableProxy*>(&Proxy) )
                != m_Collection.end() );

    if ( Proxy.Subscribes() )
    {
        MOOSDBVariableProxy* proxy = const_cast<MOOSDBVariableProxy*>(&Proxy);
        m_ProxyTimerMap.insert(
            std::pair<MOOSDBVariableProxy*, ProxyTimer>( proxy,
                                      ProxyTimer( MinPeriod, InitialTime) ) );
    }
}



//=============================================================================
void MOOSDBProxyCollection::SetMaximumPublishPeriod(
                                            MOOSDBVariableProxy const& Proxy,
                                            double MaxPeriod,
                                            double InitialTime )
{
    // If you hit this assert, it's because you need to add the Proxy
    // to the collection before assigning a maximum update period
    assert( m_Collection.find( const_cast<MOOSDBVariableProxy*>(&Proxy) )
                != m_Collection.end() );

    if ( Proxy.Publishes() )
    {
        MOOSDBVariableProxy* proxy = const_cast<MOOSDBVariableProxy*>(&Proxy);
        m_ProxyTimerMap.insert(
            std::pair<MOOSDBVariableProxy*, ProxyTimer>( proxy,
                                      ProxyTimer( MaxPeriod, InitialTime) ) );
    }
}


//=============================================================================
bool MOOSDBProxyCollection::RegisterWithMOOSDB( CMOOSCommClient& CommClient )
{
    bool retval = false;

    for ( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
          iter != m_Collection.end(); iter++ )
    {
        MOOSDBVariableProxy* proxy = *iter;
        if ( proxy->Subscribes() )
        {
            std::map<MOOSDBVariableProxy*, ProxyTimer>::iterator Timer_item =
                                                m_ProxyTimerMap.find(proxy);

            double MinUpdatePeriod = ( Timer_item != m_ProxyTimerMap.end() ) ?
                                           Timer_item->second.Period_sec : 0.0;

            retval |= CommClient.Register( proxy->GetMOOSDBVariableName(),
                                           MinUpdatePeriod );
        }
    }

    return retval;
}


//=============================================================================
bool MOOSDBProxyCollection::UnregisterFromMOOSDB( CMOOSCommClient& CommClient )
{
    bool retval = false;

    for ( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
          iter != m_Collection.end(); iter++ )
    {
        MOOSDBVariableProxy* proxy = *iter;
        if ( proxy->Subscribes() )
        {
            retval |= CommClient.UnRegister( proxy->GetMOOSDBVariableName() );
        }
    }

    return retval;
}


//=============================================================================
bool MOOSDBProxyCollection::PublishFreshData( CMOOSCommClient& CommsClient )
{
    bool retval = false;
    double t = MOOSTime();

    for ( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
          iter != m_Collection.end(); iter++ )
    {
        MOOSDBVariableProxy* proxy = *iter;

        // Ignore subscriber proxies
        if ( proxy->Publishes() == false )
        {
            continue;
        }

        //-----------------------------------------
        // Determine whether the Proxy object's
        // value should be published
        //-----------------------------------------
        bool ShouldPublish = proxy->IsFresh();

        // If no timers are configured, bypass auto-publish timer lookup
        if ( (ShouldPublish == false ) &&
             ( m_ProxyTimerMap.empty() == false ) )
        {
            // If automatic publishing is configured for the Proxy,
            // determine whether the maximum publish period has elapsed
            std::map<MOOSDBVariableProxy*, ProxyTimer>::iterator
                    Timer_item = m_ProxyTimerMap.find(proxy);
            if ( Timer_item != m_ProxyTimerMap.end() )
            {
                ProxyTimer& Timer = Timer_item->second;
                double ElapsedTime = t - Timer.LastTime;
                if ( ElapsedTime >= Timer.Period_sec )
                {
                    ShouldPublish = true;
                    Timer.LastTime = t;
                }
            }
        }


        if ( ShouldPublish )
        {
            //----------------------------------------------------
            // Publish the object's value to the MOOS Database
            //----------------------------------------------------
            retval = proxy->Publish( CommsClient );
        }
    }

    return retval;
}



//=============================================================================
bool MOOSDBProxyCollection::UpdateFromMOOSMail( MOOSMSG_LIST &NewMail,
                                                CMOOSCommClient& CommClient )
{
    bool retval = false;
    double TimeNow = MOOSTime();

    //we only subscribe to things if we are in simulator mode
    for ( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
          iter != m_Collection.end(); iter++ )
    {
        MOOSDBVariableProxy* proxy = *iter;
        CMOOSMsg Msg;

        bool GotMailForProxy = CommClient.PeekMail( NewMail,
                                                proxy->GetMOOSDBVariableName(),
                                                Msg );
        if( GotMailForProxy )
        {
            if( !Msg.IsSkewed(TimeNow) )
            {
                retval |= proxy->Update( Msg );
            }
        }
    }

    return retval;
}



//=============================================================================
bool MOOSDBProxyCollection::DoMissionFileRemapping(
                                          CProcessConfigReader& ConfigReader )
{
    bool ProxiesWereRemapped = false;

    for( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
         iter != m_Collection.end();
         iter++ )
    {
        MOOSDBVariableProxy* proxy = *iter;
        ProxiesWereRemapped |= proxy->ApplyMissionFileMapping( ConfigReader );
    }

    return ProxiesWereRemapped;
}



//=============================================================================
double MOOSDBProxyCollection::GetConfiguredPeriod(
                                    MOOSDBVariableProxy const& Proxy ) const
{
    double Period_sec = -1.0;

    MOOSDBVariableProxy* proxy = const_cast<MOOSDBVariableProxy*>(&Proxy);
    std::map<MOOSDBVariableProxy*, ProxyTimer>::const_iterator Timer_item;
    Timer_item = m_ProxyTimerMap.find( proxy );

    if ( Timer_item != m_ProxyTimerMap.end() )
    {
        Period_sec = Timer_item->second.Period_sec;
    }

    return Period_sec;
}



//=============================================================================
std::string MOOSDBProxyCollection::Print( void ) const
{
    std::ostringstream oss;

    for( set<MOOSDBVariableProxy*>::iterator iter = m_Collection.begin();
             iter != m_Collection.end();
             iter++ )
    {
        oss << (*iter)->Print() << std::endl;
    }

    return oss.str();
}

} /* namespace YellowSubUtils */
