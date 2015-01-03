//=============================================================================
/*    Copyright (C) 2012  Brandt Pedrow

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
/** @file pVehicleID.h

@brief
	A MOOS process application
	
@author Brandt Pedrow

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _PVEHICLEID_H_
#define _PVEHICLEID_H_

#include <string>
#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core library
#include "MOOS/libMOOS/Utils/MOOSLock.h"



// The application's software revision
#define APPLICATION_SOFTWARE_VERSION 0.1



//=====================================
// CONSTANTS AND MACROS

/*------------------------------------------------
	<<< Add your #define statements here >>>
------------------------------------------------*/





//=============================================================================
/** This is the main application object for the pVehicleID
	MOOS application.

	<<< Add a description of your application here >>>
*/
//=============================================================================
class pVehicleID : public  CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the MOOS application object */
	pVehicleID( void );

	//=========================================================================
	/** Called when the application closes */
	~pVehicleID();


	//=========================================================================
	/** Called when new mail has arrived from the MOOS Database.
	@details
	    This method will be called at a rate of approximately 1/CommsTick Hz. 
		In this function you'll most likely interate over the collection of 
		mail messages received or call a m_Comms::PeekMail() to look for a 
		specific named message.

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
		PVEHICLEID_CMD is recieved from the MOOS Database.
    
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

    bool LoadMissionParameters( void );
	
private:
    unsigned int m_VehicleID;	//!< Address used for receiving
    std::string m_PublishedVariables;
	// Prevent automatic generation of copy constructor and assignment operator 
	pVehicleID (const pVehicleID&);
    const pVehicleID& operator= (const pVehicleID&);
};



#endif	// END #ifdef _PVEHICLEID_H_
