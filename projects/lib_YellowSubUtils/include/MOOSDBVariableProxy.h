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
/** @file MOOSDBVariableProxy.h
 *
 * @brief
 *  Declaration of the MOOSDBVariableProxy class
 *
 * @author
 *  Dave Billin
 */
//=============================================================================
#ifndef _MOOSDBVARIABLEPROXY_H_
#define _MOOSDBVARIABLEPROXY_H_

#include <stdint.h>
#include <string>

#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/ProcessConfigReader.h"


namespace YellowSubUtils
{


//=============================================================================
/** @class MOOSDBVariableProxy
 *
 * @brief
 *  Abstract base class for an object that acts as an application-side proxy
 *  used to publish or subscribe to a variable in the MOOS database.
 *
 * @details
 *  This is the base class for a container object where values to be sent to,
 *  or values arriving from the MOOS Database can be stored.
 *
 *  An application needing to publish values can create a MOOSDBVariableProxy
 *  object for each variable it publishes in the MOOSDB.  When the application
 *  calculates or receives new data to be published to the MOOSDB, it stores
 *  the data in its respective MOOSDBVariableProxy object.  At some point,
 *  perhaps in the application's Iterate() method, or when a sufficient number
 *  of new datum have accumulated, the application iterates through its
 *  collection of MOOSDBVariableProxy objects, invoking the Publish() method on
 *  each one.  If an object in the collection has fresh data to publish, it
 *  sends it to the MOOSDB.
 *
 *  Similarly, an application needing to receive values from the MOOSDB can
 *  create a MOOSDBVariableProxy object for each MOOSDB variable it needs to
 *  subscribe to.  Whenever new mail arrives from the MOOSDB, the application
 *  can simply iterate through its collection of MOOSDBVariableProxy objects
 *  and invoke each one's ApplySubscribedData() method, passing it the received
 *  MOOSMSG_LIST object.  If a MOOSDBVariableProxy object finds that the
 *  MOOSMSG_LIST contains new data from its subscribed variable, it stores it
 *  to its local contents and signals that its value is fresh.
 *
 * @remarks
 *  MOOSDBVariableProxy derived classes are provided which supply the same
 *  basic functionality as the CMOOSVariable class supplied with MOOS.
 *  However, the MOOSDBVariableProxy class is intended to serve as a basis for
 *  further enhancement.  For example: where CMOOSVariable uses a single object
 *  that can contain only the default MOOSDB value types (string, double),
 *  MOOSDBVariableProxy employs a class heirarchy that can be extended to
 *  accommodate serialized binary data or other types, or to supply useful
 *  functionality such as thread safety for asynchronous comms, mathematical
 *  operations on published or received data (e.g. averaging, statistics,
 *  filtering, limiting), etc.
 *
 *  In addition, objects derived from MOOSDBVariableProxy can be re-mapped from
 *  the MOOS mission file, making it easier to integrate new data or MOOS
 *  applications into an environment without re-coding to avoid overlapping
 *  MOOS variable names
 */
//=============================================================================
class MOOSDBVariableProxy
{
public:

    //----------------------------------------------------
    /** @enum e_ProxyModes
     * @brief
     *  Modes that a MOOSDBVariableProxy may take on
     */
    //----------------------------------------------------
    enum e_ProxyModes
    {
        DISABLED  = 0x00,   //!< The object neither publishes nor subscribes
        PUBLISHES = 0x01,   //!< The object publishes to the MOOSDB
        SUBSCRIBES = 0x02   //!< The object subscribes to the MOOSDB
    };

    enum e_VariableDataTypes
    {
        DOUBLE, ///! The object's value will be read/written as a double
        STRING, ///! The object's value will be read/written as a string
        BINARY  ///! The object's value will be read/written as binary data
    };

    //=========================================================================
    /// Default constructor: creates a proxy variable configured to contain
    /// a double, that will be neither published nor subscribed
    MOOSDBVariableProxy( void )
      : m_ProxyMode( DISABLED ), m_IsFresh( false ),
        m_MOOSDataType(DOUBLE),
        m_CanBeRemapped( true ), m_IsRemapped( false ), m_IsDisabled( false ),
        m_TimeLastWritten( -1.0 )
    {}


    //=========================================================================
    /// Creates a proxy variable and sets its mode and (optionally) freshness
    ///
    /// @param [in] Mode
    ///   Mode the proxy variable will carry out as a combination of entries
    ///   in e_ProxyModes
    ///
    /// @param [in] Type
    ///   Data type used to send the object's contents to the MOOSDB
    ///
    /// @param [in] MOOSDBVariableName
    ///   Name of the variable in the MOOS database the object will target
    ///
    /// @param [in] ShouldMarkAsFresh
    ///   true (default) to mark the variable as fresh initially; else false
    ///   to indicate the variable is not fresh
    MOOSDBVariableProxy( e_ProxyModes Mode, e_VariableDataTypes Type,
                         std::string const& MOOSDBVariableName,
                         bool ShouldMarkAsFresh = true );


    //=========================================================================
    /// Destructor
    virtual ~MOOSDBVariableProxy( void );


    //-------------------------------------------------------------------------
    /// @addtogroup MOOSDBVariableProxy_PureVirtuals Pure Virtual Methods
    /// @brief
    ///   These functions must be implemented by derived classes
    /// @{

    //=========================================================================
    /// If the object is configured to publish and it is marked as fresh, this
    /// method publishes the object's contents to the MOOS database
    ///
    /// @remarks
    ///   Derived classes must implement this method, and should set the value
    ///   of m_TimeLastPublished
    ///
    /// @param [in] CommsClient
    ///   CMOOSCommClient object used to publish data
    ///
    /// @return
    ///   true if the object's contents were published
    ///
    /// @post
    ///   If the object's contents were published to the MOOS database, the
    ///   object will be marked as not fresh
    virtual bool Publish( CMOOSCommClient& CommsClient ) = 0;


    //=========================================================================
    /// If the object is configured to subscribe, this function will scan the
    /// contents of a MOOSMSG_LIST and extract new data arriving from the MOOS
    /// variable the object has subscribed to
    ///
    /// @remarks
    ///   Derived classes must implement this method.  The caller must ensure
    ///   that MOOSDBMessage came from the variable the object targets.
    ///
    /// @param [in] MOOSDBMessage
    ///   A message received from the MOOS database containing new data from
    ///   the MOOSDB variable the object is associated with
    ///
    /// @return
    ///   true if data was read from MOOSDBMessage into the object; else false
    ///   if the contents of MOOSDBMessage is incompatible
    ///
    /// @post
    ///   If the object received data from NewMail, it will be flagged as fresh
    virtual bool Update( CMOOSMsg const& MOOSDBMessage ) = 0;

    /// @}
    //-------------------------------------------------------------------------




    //=========================================================================
    /// @return true if the object is configured to publish to the MOOSDB
    virtual bool Publishes( void ) const { return m_ProxyMode & PUBLISHES; }

    //=========================================================================
    /// @return true if the object is configured to subscribe to the MOOSDB
    virtual bool Subscribes( void ) const { return m_ProxyMode & SUBSCRIBES; }




    //=========================================================================
    /// @return true if the object's data is fresh
    virtual bool IsFresh( void ) const { return m_IsFresh; }

    //=========================================================================
    /// Marks the object's data as fresh or not fresh
    ///
    /// @param [in] ShouldMarkAsFresh
    ///   true to mark the object's data as fresh; else false to mark it as
    ///   not fresh
    virtual void SetFresh( bool ShouldMarkAsFresh = true )
    { m_IsFresh = ShouldMarkAsFresh; }




    //=========================================================================
    /// @return true if the object's contents should be accessed as a string
    virtual bool IsString( void ) const { return m_MOOSDataType == STRING; }

    //=========================================================================
    /// @return true if the object's contents should be accessed as a double
    virtual bool IsDouble( void ) const { return m_MOOSDataType == DOUBLE; }

    //=========================================================================
    /// @return true if the object's contents should be accessed as binary data
    virtual bool IsBinaryData( void ) const { return m_MOOSDataType == BINARY;}



    //=========================================================================
    /// @return
    ///  The time when the object's contents were last written
    virtual double TimeLastWritten( void ) const { return m_TimeLastWritten; }

    //=========================================================================
    /// @return
    ///  The number of seconds that have elapsed since the object's last write
    ///  time
    virtual double SecondsSinceLastWritten( void ) const
    { return HPMOOSTime() - m_TimeLastWritten; }



    //=========================================================================
    /// @return A reference to the object's string contents
    /// @throw
    ///   A CMOOSException if the object does not contain a string
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold string data should override this behavior
    virtual std::string const& AsString( void ) throw(CMOOSException)
    {
        throw CMOOSException( s_CannotAccessAsString );
    }

    //=========================================================================
    /// @return The object's contents as a double
    /// @throw
    ///   A CMOOSException if the object does not contain a double
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold double data should override this behavior
    virtual double AsDouble( void ) throw(CMOOSException)
    {
        throw CMOOSException( s_CannotAccessAsDouble );
    }

    //=========================================================================
    /// @return A pointer to the object's binary data contents
    ///
    /// @param [out] NumBytes
    ///   The number of Bytes in the object's binary data
    ///
    /// @throw
    ///   A CMOOSException if the object does not contain binary data
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold binary data should override this behavior
    virtual void const* AsBinaryData( uint32_t& NumBytes) throw(CMOOSException)
    {
        throw CMOOSException( s_CannotAccessAsBinaryData );
    }



    //=========================================================================
    /// Sets the contents of the object to a specified string
    ///
    /// @param [in] StringValue
    ///   String to assign to the object's contents
    ///
    /// @param [in] Time
    ///   Time when the new value was received or written
    ///
    /// @throw
    ///   A CMOOSException if the object does not contain a string
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold string data should override this behavior
    virtual void Set( std::string const& StringValue, double dfTime )
    {
        throw CMOOSException( s_CannotAccessAsString );
    }


    //=========================================================================
    /// Sets the contents of the object to a specified double value
    ///
    /// @param [in] DoubleValue
    ///   A double to assign to the object's contents
    ///
    /// @param [in] Time
    ///   Time when the new value was received or written
    ///
    /// @throw
    ///   A CMOOSException if the object does not contain a double
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold doubles should override this behavior
    virtual void Set( double DoubleValue, double dfTime )
    {
        throw CMOOSException( s_CannotAccessAsDouble );
    }


    //=========================================================================
    /// Sets the contents of the object to specified binary data
    ///
    /// @param [in] pData
    ///   A pointer to binary data to copy into the object
    ///
    /// @param [in] NumBytes
    ///   Number of Bytes to copy from pData
    ///
    /// @param [in] Time
    ///   Time when the new value was received or written
    ///
    /// @throw
    ///   A CMOOSException if the object does not contain a double
    ///
    /// @remarks
    ///   The default implementation of this method throws an exception.
    ///   Derived classes that hold doubles should override this behavior
    virtual void Set( void const* pData, uint32_t NumBytes,
                      double dfTime )
    {
        throw CMOOSException( s_CannotAccessAsBinaryData );
    }



    //=========================================================================
    /// Sets the name of the MOOSDB variable the object will target
    void SetMOOSDBVariableName( std::string const& MOOSDBVariableName )
    { m_MOOSDBVariableName = MOOSDBVariableName; }

    //=========================================================================
    /// @return
    ///   Returns a reference to the name of the MOOSDB variable the object
    ///   publishes or subscribes to
    std::string const& GetMOOSDBVariableName( void ) const
    { return m_MOOSDBVariableName; }



    //=========================================================================
    /// Scans a mission file configuration for directives that re-map or
    /// disable the object's operation
    ///
    /// @param [in] ConfigReader
    ///   Reference to the CProcessConfigReader containing directives from a
    ///   MOOS mission file to process
    ///
    /// @return
    ///   true if the variable was re-mapped or disabled by mission file
    ///   settings
    ///
    /// @details
    ///   In this method, the contents of a MOOS mission file are scanned for
    ///   settings that change the MOOSDB variable the object publishes or
    ///   subscribes to, or that disable publishing or subscribing.
    ///
    ///   If the object is configured to publish to a MOOSDB variable, a
    ///   CProcessConfigReader object containing mission file settings for the
    ///   host process is scanned for a line with the form:
    ///
    ///   \code "<VARIBLE_NAME>_PUBLISHTO = <NEW_VARIABLE_NAME>" \endcode
    ///
    ///   where "<VARIBLE_NAME>" is the name of the MOOSDB variable the object
    ///   is configured to publish to, and <NEW_VARIABLE_NAME> is the name of
    ///   a different MOOSDB variable the object should publish to instead.  If
    ///   such a line is found, the object will be configured to publish to the
    ///   MOOSDB variable specified by <NEW_VARIABLE_NAME>, and will be flagged
    ///   as being re-mapped.  If <NEW_VARIABLE_NAME> is the string 'DISABLED'
    ///   (case-sensitive, without quotes), the object will instead be
    ///   configured such that its data will never be published, and will be
    ///   flagged as disabled.
    ///
    ///   If the object is instead configured to subscribe to a variable in the
    ///   MOOS database, the CProcessConfigReader object will be used to scan
    ///   for a line in the form:
    ///
    ///   \code "<VARIBLE_NAME>_SUBSCRIBETO = <NEW_VARIABLE_NAME>" \endcode
    ///
    ///   If such a line is found, the object will be configured to subscribe
    ///   to the MOOSDB variable specified in <NEW_VARIABLE_NAME>, and will be
    ///   flagged as being re-mapped.  If <NEW_VARIABLE_NAME> is the string
    ///   'DISABLED' (case-sensitive, without quotes), the object will be
    ///   configured such that it will not subscribe, and will be flagged as
    ///   disabled.
    virtual bool ApplyMissionFileMapping( CProcessConfigReader& ConfigReader );

    //=========================================================================
    /// @return
    ///   A reference to the name of the MOOSDB variable the object was
    ///   configured to publish or subscribe to prior to being re-mapped from
    ///   a mission file parameter.
    ///
    /// @remarks
    ///   If the object has not been re-mapped, this method returns a reference
    ///   to the active MOOSDB variable name
    std::string const& GetUnmappedMOOSDBVariableName( void ) const;

    //=========================================================================
    /// @return
    /// true if the object has been re-mapped by a mission file parameter
    bool IsRemapped( void ) const { return m_IsRemapped; }

    //=========================================================================
    /// @return
    /// true if the object has been disabled by a mission file parameter
    bool IsDisabled( void ) const { return m_IsDisabled; }

    //=========================================================================
    /// Controls whether the proxy should allow its target variable in the
    /// MOOS database to be re-mapped by a mission file directive
    ///
    /// @param [in] ShouldAllowRemapping
    ///   true (default setting) if the proxy variable should allow re-mapping
    ///   from a mission file directive; else false to disable re-mapping
    void SetRemappingEnabled( bool ShouldBeRemappable )
    { m_CanBeRemapped = ShouldBeRemappable; }

    //=========================================================================
    /// @return
    ///   true if the proxy allows its target to be re-mapped by directives in
    ///   the mission file
    bool CanBeRemapped( void ) const { return m_CanBeRemapped; }


    //=========================================================================
    /// @return A std::string describing the object's content
    /// @remarks
    ///   Derived classes must implement this function.
    virtual std::string Print( void ) const;


protected:
    static const char* s_CannotAccessAsString;
    static const char* s_CannotAccessAsDouble;
    static const char* s_CannotAccessAsBinaryData;

    int m_ProxyMode;   ///< Mode the object will operate in
    bool m_IsFresh;             ///< true if the object's contents are fresh
    e_VariableDataTypes m_MOOSDataType; ///< Data type of the object's contents
    std::string m_MOOSDBVariableName;   ///< DB Variable the object operates on
    std::string m_OldMOOSDBVariableName;  ///< DB variable before re-mapping
    bool m_CanBeRemapped; ///< true if re-mapping via mission config is allowed
    bool m_IsRemapped;  ///< true if object is re-mapped in the mission config
    bool m_IsDisabled;  ///< true if object is disabled in the mission config
    double m_TimeLastWritten;   ///< MOOSTime the object's content was last set


    //=========================================================================
    /// Copy constructor for base class
    ///
    /// @remarks
    ///   Base class copy constructor is protected to discourage copy
    ///   construction using a base class handle.  Derived classes may call the
    ///   base class copy constructor from within their own copy constructor
    ///   implementation
    MOOSDBVariableProxy( MOOSDBVariableProxy const& other );

    //=========================================================================
    /// Assignment operator overload for base class
    ///
    /// @remarks
    ///   Base class assignment operator is protected to discourage assignment
    ///   between base class objects.  Derived classes should implement their
    ///   own assignment operators and call this base class method
    virtual MOOSDBVariableProxy& operator= (MOOSDBVariableProxy const& other);
};


//=============================================================================
inline
std::string const& MOOSDBVariableProxy::GetUnmappedMOOSDBVariableName( void )
const
{
    return ( m_IsRemapped ) ?
            m_OldMOOSDBVariableName : m_MOOSDBVariableName;
}



}   // END namespace YellowSubUtils

#endif // END #ifndef _MOOSDBVARIABLEPROXY_H_
