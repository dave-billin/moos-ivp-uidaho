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
/** @file MOOSDBDoubleProxy.h
 *
 * @brief
 *  Declaration of the MOOSDBdoubleProxy class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================
#ifndef MOOSDBDOUBLEPROXY_H_
#define MOOSDBDOUBLEPROXY_H_

#include "MOOSDBVariableProxy.h"

namespace YellowSubUtils
{

//=============================================================================
/** @class MOOSDBDoubleProxy
 *
 * @brief
 *   A MOOSDBVariableProxy that can be used to publish or subscribe to a
 *   double in the MOOS database
 */
//=============================================================================
class MOOSDBDoubleProxy : public MOOSDBVariableProxy
{
public:

    //=========================================================================
    /// Default constructor: creates a MOOSDBdoubleProxy object with publishing
    /// and subscribing disabled, a value of zero, and an empty target variable
    /// name
    MOOSDBDoubleProxy();


    //=========================================================================
    /// Creates a MOOSDBdoubleProxy that targets a specified variable in the
    /// MOOS database, and assigns it an initial value and freshness
    ///
    /// @param [in] Mode
    ///   Mode the object will function in (PUBLISH | SUBSCRIBE)
    ///
    /// @param [in] MOOSDBVariableName
    ///   Name of the variable in the MOOS database the object will target
    ///
    /// @param [in] InitialValue
    ///   Initial value to assign to the object
    ///
    /// @param [in] IsFresh
    ///   true (default) to mark the variable as fresh immediately; else false
    ///   to leave it unfresh
    MOOSDBDoubleProxy( e_ProxyModes Mode,
                       std::string const& MOOSDBVariableName,
                       double InitialValue = 0.0, bool IsFresh = true );


    //=========================================================================
    /// Copy constructor
    MOOSDBDoubleProxy( MOOSDBDoubleProxy const& other );

    //=========================================================================
    /// Destructor
    virtual ~MOOSDBDoubleProxy();




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
    /// object's value using data received from the MOOS Database
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
    /// Returns the object's double contents
    double AsDouble( void ) const { return m_DoubleValue; }

    //=========================================================================
    /// Sets the value of the object's double contents
    virtual void Set( double DoubleValue, double Time );


protected:
    double m_DoubleValue;   ///< Value that will be published to the MOOSDB
};


//=============================================================================
inline void MOOSDBDoubleProxy::Set( double DoubleValue, double Time )
{
    m_DoubleValue = DoubleValue;
    m_TimeLastWritten = Time;
}


} /* namespace YellowSubUtils */
#endif /* MOOSDBDOUBLEPROXY_H_ */
