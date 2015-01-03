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
/** @file MOOSDBProxyCollection.h
 *
 * @brief
 *	Brief description of MOOSDBProxyCollection.h
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef MOOSDBPROXYCOLLECTION_H_
#define MOOSDBPROXYCOLLECTION_H_

#include <stdint.h>
#include <set>

#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/ProcessConfigReader.h"
#include "MOOSDBVariableProxy.h"

namespace YellowSubUtils
{


//=============================================================================
/// @class MOOSDBProxyCollection
/// @brief
///   A class that manages a set of MOOSDBVariableProxy objects, and supplies
///   methods for adding, removing, registering, publishing, and updating them
///   with data from the MOOS database
///
/// @remarks
///   This class is not a container, and does not create or delete proxy
///   objects.  It simply provides a convenient way to service multiple Proxy
///   objects at once.
//=============================================================================
class MOOSDBProxyCollection
{
public:

    //=========================================================================
    /// Creates an empty collection of proxy variables
    MOOSDBProxyCollection();

    //=========================================================================
    /// Destructor
    ///
    /// @note
    ///   The destructor does not destroy any objects registered with the
    ///   collection.
    virtual ~MOOSDBProxyCollection();



    //=========================================================================
    /// Registers a proxy variable with the collection
    ///
    /// @param [in] Proxy
    ///  A reference to the MOOSDBVariableProxy object to add to the object's
    ///  collection
    void AddProxy( MOOSDBVariableProxy& Proxy )
    { m_Collection.insert( &Proxy ); }


    //=========================================================================
    /// Un-registers a proxy variable, removing it from the collection
    ///
    /// @param [in] Proxy
    ///  Reference to the MOOSDBVariableProxy object to remove from the
    ///  object's collection
    ///
    /// @note
    ///  If the object specified by Proxy is not a member of the object's
    ///  collection, no action is taken.
    void RemoveProxy( MOOSDBVariableProxy& Proxy );



    //=========================================================================
    /// @return The number of MOOSDBVariableProxy objects in the collection
    uint32_t NumRegisteredProxies( void ) const
    { return m_Collection.size(); }


    //=========================================================================
    /// Sets the minimum number of seconds that must elapse between received
    /// updates to a specified Proxy object from the MOOS database
    ///
    /// @param [in] Proxy
    ///   Reference to the proxy whose minimum update period is being set
    ///
    /// @param [in] MinPeriod
    ///   Minimum number of seconds that must elapse between consecutive
    ///   updates from the object's target variable in the MOOS database
    ///
    /// @param [in] InitialTime
    ///   MOOS time to register as the last time the Proxy was updated
    ///
    /// @pre
    ///   It is assumed that Proxy has already been added to the collection
    void SetMinimumUpdatePeriod( MOOSDBVariableProxy const& Proxy,
                                 double MinPeriod, double InitialTime = 0.0 );

    //=========================================================================
    /// @return
    ///   The minimum number of seconds that must elapse between consecutive
    ///   updates to the specified Subscriber Proxy object from the MOOS
    ///   database; or -1.0 if no minimum update period has been assigned for
    ///   the specified object
    ///
    /// @param [in] Proxy
    ///   Reference to the proxy whose minimum update period is being set
    double GetMinimumUpdatePeriod( MOOSDBVariableProxy const& Proxy ) const;



    //=========================================================================
    /// Sets the maximum number of seconds that may elapse without publishing
    /// a specified Proxy object before the object's value is automatically
    /// published regardless of its freshness.
    ///
    /// @details
    ///   This method may be used to ensure that a Proxy object publishes its
    ///   value at least once every N seconds.  By default, auto-publishing
    ///   functionality is disabled for Proxy objects in the collection.  Use
    ///   this method to enable it for specific Proxy variables.
    ///
    /// @param [in] Proxy
    ///   Reference to the proxy whose maximum update period is being set
    ///
    /// @param [in] MaxPeriod
    ///   Maximum number of seconds that may elapse without the specified Proxy
    ///   object being published before its value is published automatically
    ///   regardless of the object's freshness
    ///
    /// @param [in] InitialTime
    ///   MOOS time to register as the last time the Proxy was published (set
    ///   to zero if the object should publish the next chance it gets)
    ///
    /// @pre
    ///   It is assumed that Proxy has already been added to the collection
    void SetMaximumPublishPeriod( MOOSDBVariableProxy const& Proxy,
                                 double MaxPeriod, double InitialTime = 0.0 );

    //=========================================================================
    /// @return
    ///   The maximum number of seconds that may elapse between consecutive
    ///   publishes of the specified Proxy object to the MOOS database; or
    ///   -1.0 if no maximum update period has been assigned for the specified
    ///   Proxy object
    ///
    /// @param [in] Proxy
    ///   Reference to the proxy whose maximum update period is being requested
    double GetMaximumPublishPeriod( MOOSDBVariableProxy const& Proxy ) const;



    //=========================================================================
    /// Registers with the MOOS database to receive updates from MOOSDB
    /// variables corresponding to members of the collection that are
    /// configured to subscribe
    ///
    /// @param [in] CommClient
    ///   A reference to the CMOOSCommClient object used to communicate with
    ///   the MOOS database
    bool RegisterWithMOOSDB( CMOOSCommClient& CommClient );

    //=========================================================================
    /// Un-registers all variables in the collection from the MOOS database
    ///
    /// @param [in] CommClient
    ///   A reference to the CMOOSCommClient object used to communicate with
    ///   the MOOS database
    bool UnregisterFromMOOSDB( CMOOSCommClient& CommClient );



    //=========================================================================
    /// Publishes the value of any MOOSDBVariableProxy objects in the
    /// collection marked as fresh to the MOOS database
    ///
    /// @param [in] CommClient
    ///   A reference to the CMOOSCommClient object used to communicate with
    ///   the MOOS database
    ///
    /// @return
    ///   true if any data was published to the MOOS database; else false if
    ///   no data was published
    bool PublishFreshData( CMOOSCommClient& CommClient );

    //=========================================================================
    /// Updates MOOSDBVariableProxy objects in the collection that are
    /// configured to subscribe
    ///
    /// @param [in] NewMail
    ///   A reference to mail received from the MOOS database that should be
    ///   used to update objects in the collection that are configured to
    ///   subscribe
    ///
    /// @param [in] CommClient
    ///   A reference to the CMOOSCommClient object used to detect skewed
    ///   incoming data
    ///
    /// @return
    ///   true if the value of any objects in the collection was updated
    bool UpdateFromMOOSMail( MOOSMSG_LIST &NewMail,
                             CMOOSCommClient& CommClient );


    //=========================================================================
    /// Applies re-mapping directives from a mission file to all variables in
    /// the collection
    ///
    /// @param [in] ConfigReader
    ///   Reference to a mission file reader used to search for re-mapping
    ///   directives
    ///
    /// @return
    ///   true if any proxy objects in the collection were re-mapped
    bool DoMissionFileRemapping( CProcessConfigReader& ConfigReader );


    //=========================================================================
    /// Prints the contents of each variable in the collection to a string
    ///
    /// @param [in] ShouldGroupByMode
    ///   true to group printed variables by their mode (publish/subscribe)
    std::string Print( void ) const;

protected:

    /// @return
    ///   The configured period for a specified Proxy object or -1 if the Proxy
    ///   object is not in the collection
    ///
    /// @param [in] Proxy
    ///   The Proxy object whose timer should be returned
    double GetConfiguredPeriod( MOOSDBVariableProxy const& Proxy ) const;


    /// @struct ProxyTimer
    /// @brief
    ///   Data structure used to implement minimum update period and maximum
    ///   publish period of registered proxy variables
    struct ProxyTimer
    {
        ProxyTimer( double Period_sec, double LastTime )
          : Period_sec( Period_sec ),
            LastTime( LastTime ) {}

        /// For subscriber Proxies, this holds the minimum number of seconds
        /// that must elapse between successive updates from the MOOS database;
        /// for publisher Proxies, it holds the maximum number of seconds that
        /// may elapse before forced publishing of the Proxy
        double Period_sec;

        /// For subscriber Proxies, this holds the last time the Proxy was
        /// updated from the MOOS database; for publisher Proxies, this holds
        /// the last time the Proxy published to the MOOS database
        double LastTime;
    };


    /// Pointers to MOOSDBVariableProxy objects that are managed by the
    /// collection
    std::set<MOOSDBVariableProxy*> m_Collection;

    /// Maps proxy variables to the MOOSTime they were last published
    std::map<MOOSDBVariableProxy*, ProxyTimer> m_ProxyTimerMap;

private:

    //-----------------------------------------------------------------
    // Copy constructor and assignment operator are disallowed
    MOOSDBProxyCollection( MOOSDBProxyCollection const& );
    MOOSDBProxyCollection& operator=( MOOSDBProxyCollection const& );
    //-----------------------------------------------------------------
};


//=============================================================================
inline double MOOSDBProxyCollection::GetMinimumUpdatePeriod(
                                    MOOSDBVariableProxy const& Proxy ) const
{
    return GetConfiguredPeriod( Proxy );
}

//=============================================================================
inline double MOOSDBProxyCollection::GetMaximumPublishPeriod(
                                    MOOSDBVariableProxy const& Proxy ) const
{
    return GetConfiguredPeriod( Proxy );
}


} /* namespace YellowSubUtils */
#endif /* MOOSDBPROXYCOLLECTION_H_ */
