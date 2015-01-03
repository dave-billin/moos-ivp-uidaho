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
/** @file iRevolutionCompass.h
 *
 * @brief
 *   MOOS application class for the iRevolutionCompass application
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

#ifndef _IREVOLUTIONCOMPASS_H_
#define _IREVOLUTIONCOMPASS_H_

#include <string>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include "MOOS/libMOOS/MOOSLib.h"
#include "RevolutionCompassModule.h"


//=============================================================================
/** @brief
 * 	Definition of the iRevolutionCompass MOOS application class
 *
 * @ingroup iRevolutionCompass
 */
//=============================================================================
class iRevolutionCompass : public CMOOSApp
{
public:
    /** An example mission file configuration for the application */
    static const char ExampleMissionFileConfiguration[];

    //=========================================================================
    /** Creates an instance of the object */
    iRevolutionCompass( void );

    //=========================================================================
    /** Called when the application is exiting */
    virtual ~iRevolutionCompass();


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

    RevolutionCompassModule m_Compass;  /**< Compass object */

    float m_HeadingOffset_deg;  /**< Offset in degrees applied to the heading
                                     reported by the compass module */

    /** @enum e_PublishedVariableIDs
     * @brief
     *  Indices of published variables in m_PublishedVarNames
     */
    enum e_PublishedVariableIDs
    {
        Heading = 0,
        Dip,
        Pitch,
        Roll,
        MagX,
        MagY,
        MagZ,
        Status,
        NUM_PUBLISHED_VARIABLES
    };


    /** @var m_PublishedVarNames
     * @brief
     *  A table containing names of MOOSDB variables this application
     *  publishes to.  This table is meant to be indexed using elements of
     *  e_SubscribedVariables
     */
    std::string m_PublishedVarNames[NUM_PUBLISHED_VARIABLES];

    //-------------------------------------------------
    // Disable automatically-generated functions
    //-------------------------------------------------
    iRevolutionCompass (const iRevolutionCompass&);
    const iRevolutionCompass& operator= (const iRevolutionCompass&);
};


#endif	// END #ifdef _IARCHANGELIMU_H_
