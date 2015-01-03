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
/** @file pMissionMonitor.h
 *
 * @brief
 * 	A MOOS process to monitor critical variables and enforce safety conditions
 *
 * @author Dave Billin
 */
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _pMissionMonitor_H_
#define _pMissionMonitor_H_

#include <string>
#include "MOOS/libMOOS/MOOSLib.h" 
#include "MonitorTarget.h"



//=============================================================================
/** This is the main application object for the pMissionMonitor
	MOOS application.

	pMissionMonitor is a MOOS application with a singular task: monitor sets of
	logical conditions involving variables in the MOOS database.  If any of the
	conditions in a set evaluate to FALSE, post pre-configured value(s) to one
	or more MOOS variables in the database.

	This application is primarily intended to monitor critical vehicle
	systems, and initiate actions via the MOOSDB in the case of failures.
*/
//=============================================================================
class pMissionMonitor : public  CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the MOOS application object */
	pMissionMonitor( void );

	//=========================================================================
	/** Called when the application closes */
	~pMissionMonitor();


	//=========================================================================
	/** Called when new mail has arrived from the MOOS Database.
	@details
	    This method will be called at a rate determined by the setting of
	    CommsTick mission file parameter.  In this function you'll most likely
	    interate over the collection of mail messages received from the MOOSDB
	    or call a m_Comms::PeekMail() to look for a specific named message.

	@param NewMail
		A reference to the list of received mail from the MOOS Database

	@return
		true on success; false on error
	*/
	bool OnNewMail(MOOSMSG_LIST & NewMail);


	//=========================================================================
	/** If command message filtering is enabled for this application (via the
		EnableCommandMessageFiltering() function), then this function is called
		when a command message having the identifier
		pMissionMonitor_CMD is recieved from the MOOS Database.

	@param Msg
		A copy of the received command message

	@return
		true on success; false on error

	@see CMOOSApp::EnableCommandMessageFiltering
	*/
	bool OnCommandMsg(CMOOSMsg Msg);


	//=========================================================================
    /** This function is where the application can do most of its work.
	@details
		The rate at which Iterate() is called is determined by the value of the
		AppTick parameter specified in the (.moos) mission file.  The value of
		AppTick is loaded automatically when the application starts up.
	*/
    bool Iterate( void );


	//=========================================================================
    /** Called when the application first starts up
	@details
		This function is called as the application first starts up before any
		calls to Iterate() begin.
	*/
    bool OnStartUp( void );


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called when the application is disconnected from the MOOS Database. */
    bool OnDisconnectFromServer( void );


private:
    bool m_AppIsInitialized;	/**< Set to true at the end of OnStartup() */

    InfoBuffer m_InfoBuffer;			/**< Info buffer for MOOS variables
    										 used in time slice CONDITIONS */

    std::vector<MonitorTarget> m_MonitorTargets;    /**< A vector of monitored
												         logical conditions */

    std::set<std::string> m_ConditionVarNames;	/**< Names of MOOS variables
     	 	 	 	 	 	 	 	 	 	         used in time slice
     	 	 	 	 	 	 	 	 	 	         CONDITIONS */


    //=========================================================================
    /** Loads monitor targets from the monitor configuration file
     *
     * @param sFilePath
     *	Absolute path of the time slice configuration file to load from
     *
     * @return
     * 	true if the file was loaded successfully; else false if the file could
     * 	not be loaded or a parsing error occurs
     */
    bool LoadMonitorTargetsFromFile( std::string& sFilePath );



	// Prevent automatic generation of copy constructor and assignment operator
	pMissionMonitor (const pMissionMonitor&);
    const pMissionMonitor& operator= (const pMissionMonitor&);
};



#endif	// END #ifdef _pMissionMonitor_H_
