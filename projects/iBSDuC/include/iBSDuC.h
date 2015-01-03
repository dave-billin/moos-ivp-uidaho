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
/** @file iBSDuC.h
 *
 * @brief
 *   Declaration of the MOOS application class used to implement the
 *   iBSDuC application
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 *
 * @par Created for
 * 	The University of Idaho in Moscow, Idaho, USA.
 */
//=============================================================================
#ifndef _IBSDUC_H_
#define _IBSDUC_H_

#include <string>

#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core libraries
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "MOOSDBDoubleProxy.h"
#include "MOOSDBStringProxy.h"
#include "MOOSDBProxyCollection.h"
#include "BSDuCModule.h"    // Wrapper class for BSD microcontroller interface


//=============================================================================
/** @brief
 * 	Definition of the iBSDuC MOOS application class
 *
 * @ingroup iBSDuC
 */
//=============================================================================
class iBSDuC : public CMOOSApp
{
public:
    /** An example mission file configuration for the application */
    static const char ExampleMissionFileConfiguration[];

    //=========================================================================
    /** Creates an instance of the object */
    iBSDuC( void );

    //=========================================================================
    /** Called when the application is exiting */
    virtual ~iBSDuC();


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
    bool OnNewMail( MOOSMSG_LIST& NewMail );

    //=========================================================================
    /** Called when a new value is received from the iBSDuC_CMD variable in
     *  the MOOS database
     *
     * @param Msg
     *   A copy of the received command message from the MOOS database
     *
     * @return
     *   false to signal error; else true
     *
     * @see CMOOSApp::EnableCommandMessageFiltering
     */
    bool OnCommandMsg( CMOOSMsg Msg );


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

protected:

    //=========================================================================
    /** Called from OnStartup() to load application parameters from the MOOS 
     * mission file
     *
     * @return
     *  true if all is well; else false to cause the application to exit.
     */
    bool LoadMissionFileParameters( void );


    //=========================================================================
    /** Opens and closes the BSD microcontroller interface in correspondence
     *  with the connection to the MOOS database
     *
     * @remarks
     *   If the MOOS database is connected, this will open a communications
     *   session with the BSD microcontroller.  If the MOOS database
     *   disconnects, this function will terminate any open communications
     *   session.
     */
    void Service_BSDuC_Connection( void );

private:
    int m_Verbosity;	        /**< Verbosity of debugging messages */
    bool m_StartupIsDone;       /**< Used to guard MOOSDB registration */

    std::string m_SerialPortDevice; /**< Serial port device connected to the
                                         BSD microcontroller */

    BSDuCModule m_BSDMicro;     /**< BSD microcontroller interface wrapper */


    YellowSubUtils::PrecisionTime m_FirmwareResetTimer; /** Used to implement
                                                            firmware reset
                                                            request safety */

    uint8_t m_FirmwareResetCounter; /**< Used to implement firmware reset
                                         request safety */

    //-------------------------------------------------------------
    // Proxy objects that implement published MOOS variables
    //-------------------------------------------------------------
    YellowSubUtils::MOOSDBStringProxy m_BSD_IsOnline_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_BSD_Depth_Proxy;
    YellowSubUtils::MOOSDBStringProxy m_BSD_DepthFail_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_Propeller_RPM_Proxy;
    YellowSubUtils::MOOSDBStringProxy m_WaterIsDetected_Proxy;
    YellowSubUtils::MOOSDBStringProxy m_BatteryMonitorSerialNum_Proxy;
    YellowSubUtils::MOOSDBStringProxy m_FirmwareVersion_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_RPMVelocityEstimate_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_BatteryVolts_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_BatteryAmps_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_BatteryDischargedAmps_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_BatteryTemp_Proxy;

    //-------------------------------------------------------------
    // Proxy objects that implement subscribed MOOS variables
    //-------------------------------------------------------------
    YellowSubUtils::MOOSDBDoubleProxy m_DesiredRudderAngle_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_DesiredElevatorAngle_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_DesiredAileronAngle_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_DesiredThrust_Proxy;

    YellowSubUtils::MOOSDBDoubleProxy m_LeftElevatorTrim_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_RightElevatorTrim_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_RudderTrim_Proxy;
    YellowSubUtils::MOOSDBDoubleProxy m_CouplingCoefficient_Proxy;

    YellowSubUtils::MOOSDBProxyCollection m_ProxyCollection;


    //--------------------------------------------------------
    // Disallow copy constructor and assignment operator
    //--------------------------------------------------------
    iBSDuC( iBSDuC const& );
    iBSDuC const& operator=( iBSDuC const& );
};

#endif	// END #ifdef _IBSDUC_H_
