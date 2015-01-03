//=============================================================================
/*    Copyright (C) 2012  Dave Billin

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
/** @file iGPSd.h
 *
 * @brief
 *   MOOS application class for the iGPSd application
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 *
 * @par Created for
 * 	The University of Idaho in Moscow, Idaho, USA.
 */
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef _IGPSD_H_
#define _IGPSD_H_

#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "libgpsmm.h"       // C++ API used to access GPSd



//=============================================================================
/** @brief
 * 	Definition of the iGPSd MOOS application class
 *
 * @ingroup iGPSd
 */
//=============================================================================
class iGPSd : public CMOOSApp
{
public:
    /** An example mission file configuration for the application */
    static const char ExampleMissionFileConfiguration[];

    //=========================================================================
    /** Creates an instance of the object */
    iGPSd( void );

    //=========================================================================
    /** Called when the application is exiting */
    virtual ~iGPSd();


    //======================================
    // Methods inherited from CMOOSApp
    //======================================

    //=========================================================================
    /** Called when new mail arrives from the MOOS Database.
     *
     * @details
     *  This method will be called at a rate determined by the CommsTick 
     *  mission file parameter.  In it you'll probably want to interate over 
     *  the list of received MOOS variable messages received in NewMail, or  
     *  call m_Comms::PeekMail() to check for a specific variable.
     *
     * @param NewMail
     *  Reference to a list of MOOS variable messages received from the MOOS 
     *  Database
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool OnNewMail(MOOSMSG_LIST & NewMail);



    //=========================================================================
    /** This function is where the application can do most of its work.
     *
     * @details
     *  This method is called at a rate determined by the value of the AppTick
     *  mission file parameter loaded at startup.
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool Iterate( void );


    //=========================================================================
    /** Called when the application first starts up
     *
     * @details
     *  This method contains a call to LoadMissionFileParameters(), and runs
     *  before Iterate() is ever called.
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool OnStartUp( void );


    //=========================================================================
    /** Called when the application connects to the MOOS Database.
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool OnConnectToServer( void );


    //=========================================================================
    /** Called when the application disconnects from the MOOS Database.
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool OnDisconnectFromServer( void );


    //=========================================================================
    /** A helper function that opens a connection to the GPS daemon
     * @return
     *  true if a connection was opened; else false
     */
    bool DaemonConnect( void );


    //=========================================================================
    /** A helper function that closes the connection to the GPS daemon */
    void DaemonDisconnect( void );

    //=========================================================================
    inline bool DaemonIsConnected( void ) const {return m_gpsdclient != NULL;}


protected:

    enum e_PublishedVariable_Ids
    {
        FIX_IS_VALID = 0,   /**< Status of the GPS fix */
        FIX_MODE,           /**< Mode of the last fix (no fix, 2D, 3D) */
        LATITUDE_DEG,       /**< Latitude in degrees */
        LONGITUDE_DEG,      /**< Longitude in degrees */
        HEADING_DEG,        /**< Heading in degrees relative to true North */
        GROUND_SPEED_MPS,   /**< Speed over ground in meters per second */
        ALTITUDE_M,         /**< Altitude in meters */
        VERTICAL_SPEED_MPS, /**< Vertical speed (rate of climb) in meters per
                                 second */

        #ifdef USE_GOOGLE_PROTOCOL_BUFFERS
        PROTOBUF_REPORT,     /**< Serialized Google Protocol Buffer structure */
        #endif
        NUM_PUBLISHED_VARIABLES     /**< Used for bounds-checking only */
    };

    //=========================================================================
    /** Called from OnStartup() to load application parameters from the MOOS 
     * mission file
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool LoadMissionFileParameters( void );


    gpsmm* m_gpsdclient;    ///< GPS daemon client interface


private:
    int m_Verbosity;	      /**< Verbosity of debugging messages */
    bool m_OnStartupIsDone;   /**< Used to guard MOOSDB registration */

    std::string m_ServerHostname;   /**< Hostname of the GPSd server */
    uint16_t m_ServerPort;          /**< TCP port to connect to on server */


    /** @var m_PublishedVarNames
     * @brief
     *  An array of MOOS variable names published by iGPSd populated at runtime
     */
    std::string m_PublishedVarNames[NUM_PUBLISHED_VARIABLES];

    /** @var sm_DefaultPublishedVarNames
     * @brief
     *  Array of default names of MOOS variables published by iGPSd
     */
    static char const* sm_DefaultPublishedVarNames[NUM_PUBLISHED_VARIABLES];


    //=========================================================================
    /** Helper function that publishes info in a gps_data_t structure to the
     *  MOOS database
     *
     * @param GpsData
     *  A reference to the GPS fix info to be published
     */
    void PublishGpsData( struct gps_data_t& GpsData );


    //=========================================================================
    /** Helper template function that will publish a value to a named variable
     *  if the variable name is not an empty string.
     *
     * @param sTargetVariableName
     *  Reference to a string containing the name of the MOOS variable to
     *  publish to
     *
     * @param ValueToPublish
     *  The value to be published to the variable
     */
    template <typename DataType>
    void NotifyIfEnabled( const std::string& sTargetVariableName,
                          const DataType& ValueToPublish,
                          double Timestamp_sec )
    {
        if ( !sTargetVariableName.empty() )
        {
            m_Comms.Notify(sTargetVariableName, ValueToPublish, Timestamp_sec);
        }
    }

    // Disable automatic generation of copy constructor and assignment operator
    iGPSd (const iGPSd&);
    const iGPSd& operator= (const iGPSd&);
};


#endif	// END #ifdef _IARCHANGELIMU_H_
