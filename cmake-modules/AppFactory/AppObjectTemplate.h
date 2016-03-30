//=============================================================================
/*    Copyright (C) MOOSAPPFACTORY_YEAR  MOOSAPPFACTORY_AUTHOR

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
/** @file MOOSAPPFACTORY_NAME.h
 *
 * @brief
 *   Declaration of the MOOS application class used to implement the
 *   MOOSAPPFACTORY_NAME application
 *
 * @author MOOSAPPFACTORY_AUTHOR
 *         MOOSAPPFACTORY_EMAIL
 *
 * @par Created for
 * 	The University of Idaho in Moscow, Idaho, USA.
 */
//=============================================================================
#ifndef _MOOSAPPFACTORY_CAPITALNAME_H_
#define _MOOSAPPFACTORY_CAPITALNAME_H_


#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core libraries
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"


//=============================================================================
/** @brief
 * 	Definition of the MOOSAPPFACTORY_NAME MOOS application class
 *
 * @ingroup MOOSAPPFACTORY_NAME
 */
//=============================================================================
class MOOSAPPFACTORY_NAME : public CMOOSApp
{
public:
    /** An example mission file configuration for the application */
    static const char ExampleMissionFileConfiguration[];

    //=========================================================================
    /** Creates an instance of the object */
    MOOSAPPFACTORY_NAME( void );

    //=========================================================================
    /** Called when the application is exiting */
    virtual ~MOOSAPPFACTORY_NAME();


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


private:
    int m_Verbosity;	      /**< Verbosity of debugging messages */
    bool m_OnStartupIsDone;   /**< Used to guard MOOSDB registration */

    //--------------------------------------------------------
    // Disallow copy constructor and assignment operator
    //--------------------------------------------------------
    MOOSAPPFACTORY_NAME( MOOSAPPFACTORY_NAME const& );
    MOOSAPPFACTORY_NAME const& operator=( MOOSAPPFACTORY_NAME const& );
};

#endif	// END #ifdef _MOOSAPPFACTORY_CAPITALNAME_H_
