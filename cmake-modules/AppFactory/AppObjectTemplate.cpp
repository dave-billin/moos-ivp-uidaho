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
/** @file MOOSAPPFACTORY_NAME.cpp
 *
 * @brief
 *  Implementation of the MOOSAPPFACTORY_NAME application class
 *
 * @author MOOSAPPFACTORY_AUTHOR
 *         MOOSAPPFACTORY_EMAIL
 */
//=============================================================================

#include <string>
#include <stdint.h>

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"

#include "MOOSAPPFACTORY_NAME.h"
#include "AppVersion.h"

using namespace std;


// Character string containing an example mission file configuration
// block for the application
const char MOOSAPPFACTORY_NAME::ExampleMissionFileConfiguration[]=
    "ProcessConfig = MOOSAPPFACTORY_NAME\n"
    "{\n"
    "    // REQUIRED PARAMETERS\n"
    "\n"
    "    // OPTIONAL PARAMETERS\n"
    "    AppTick   = 100    // Frequency (Hz) at which Iterate() runs\n"
    "    CommsTick = 100    // Frequency (Hz) of MOOS database updates\n"
    "\n"
    "    VERBOSITY = 0      // Verbosity of debugging messages\n"
    "}\n"
    "\n";



//=============================================================================
/* 
    This function is called when the application object is created in main().
    This is a good place to initialize application variables to default
    values, and set up dynamic memory resources.  

    NOTE: This function executes as the application object is being created, 
          so it's best to save activities that might require error handling
          (e.g. opening files or I/O devices) for the OnStartup() method.
*/
MOOSAPPFACTORY_NAME::MOOSAPPFACTORY_NAME( void )
:   m_Verbosity(0),
    m_OnStartupIsDone(false)
{

}



//=============================================================================
/* 
    This function is called when the application object is exiting.  Make sure
    you release any resources your program allocated!

    NOTE: There is no need to explicitly delete Dynamic Variables.  They get
          cleaned up automatically.
*/
MOOSAPPFACTORY_NAME::~MOOSAPPFACTORY_NAME()
{
}




//=============================================================================
/* 
    This is where your application will probably do most of its work.  Some
    handy methods you might want to use include:

    Get the current system time:
        MOOSTime()

    Check whether the MOOS database is connected:
        m_Comms.IsConnected()

    Publish a value directly to the MOOS database:
        m_Comms.Notify()

    Set or get the value of a Dynamic Variable:
        SetMOOSVar() and GetMOOSVar()

    Publish Dynamic Variable values that have changed to the MOOS database:
        PublishFreshMOOSVariables()

    Publish a debugging message to the MOOS database variable MOOS_DEBUG:
        MOOSDebugWrite()

    Set the error state and message in the MOOSAPPFACTORY_NAME_STATUS variable:
        SetAppError()

    NOTE: returning false will cause the application to exit.
*/
bool MOOSAPPFACTORY_NAME::Iterate( void )
{
	return true;
}






//=============================================================================
/*
    Here is where the application should handle mail received from the MOOS 
    database.  This mail takes the form of CMOOSMsg objects - each of which
    contains the name and contents of a MOOS variable the application has
    subscribed to.

    NOTE: returning false will cause the application to exit.
*/
bool MOOSAPPFACTORY_NAME::OnNewMail(MOOSMSG_LIST & NewMail)
{
    //-------------------------------------------
    // Update dynamic variables with values 
    // received from the MOOS database
    //-------------------------------------------
    UpdateMOOSVariables(NewMail);


    // If your application isn't using dynamic variables, or if 
    // you would prefer to handle messages individually, un-comment
    // the following code to iterate through the received mail
    /*
    MOOSMSG_LIST::iterator iter;
    double t = MOOSTime();      // Current system time
    for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
    {
	    CMOOSMsg& Msg = *iter;

        // STATUS: Msg contains a mail message from the MOOS database.
        // Note that this message may be skewed (i.e. it may have been 
        // published to the database more than a few seconds ago)

	    // Here we process only messages that are not skewed
	    if( !RxMsg.IsSkewed(t) )
	    {
		    // <<< Process only un-skewed messages here >>>

		    continue;
	    }

        // <<< Process skewed messages here >>>

    }
    */

	return true;
}









//=============================================================================
/*
    This function is called once when the application starts running.  The 
    first thing it does is call to LoadMissionParameters() to read parameters
    from the (.moos) mission file.

    This function is a good spot to open files, connect to I/O devices, or 
    perform other actions that might result in an error condition that must be
    handled gracefully.

    This is also a good place to create Dynamic Variables (which can simplify 
    the process of publishing and subscribing to the MOOS database) using 
    AddMOOSVariable().

    NOTE: returning false will cause the application to exit.
*/
bool MOOSAPPFACTORY_NAME::OnStartUp( void )
{

    //---------------------------------
	// Print a startup banner
    //---------------------------------
	string sAppName = GetAppName();
	string sBar = string(40, '=') + "\n";
	MOOSTrace(sBar +
			  MOOSFormat("MOOSAPPFACTORY_NAME version %s\n", 
                         APP_VERSION_STRING) +
			  "Written by MOOSAPPFACTORY_AUTHOR\n" + sBar + "\n\n");


    //---------------------------------
	// Load mission file parameters.
    //---------------------------------
	if ( LoadMissionFileParameters() == false )
	{
		return false;
	}

    //------------------------------------------------------------
    // << Do your start-up work here: create Dynamic Variables, >>
    // << connect to I/O devices, etc.                          >>
    //------------------------------------------------------------


    // Register Dynamic Variables to receive 
    // updates from the MOOS database
    m_OnStartupIsDone = true;
    RegisterMOOSVariables();
    
    return true;
}







//=============================================================================
/*
    This function gets called from OnStartup() to load parameters from the
    (.moos) mission file.  Call: 

    m_MissionReader.GetConfigurationParam()
        to read the value of a mission file parameter occurring in the 
        application's "ProcessConfig = MOOSAPPFACTORY_NAME" block

    m_MissionReader.GetValue()
        to load a global scope parameter (i.e. one occuring outside a
        ProcessConfig block)

    NOTE: There is no need to load the AppTick and CommsTick parameters;  
          they are automatically loaded.

    Returning false from this function causes the application to exit.
*/
bool MOOSAPPFACTORY_NAME::LoadMissionFileParameters( void )
{
    
    //------------------------------------------
    // LOAD REQUIRED MISSION FILE PARAMETERS
    //------------------------------------------
    
    // << Load parameters required by your application here >>


    //------------------------------------------
    // LOAD OPTIONAL MISSION FILE PARAMETERS
    //------------------------------------------

	// Get the verbosity level used for debugging messages
	m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);

    // << Load other optional mission file parameters >>

	return true;
}






//=============================================================================
/*
    This function gets called when the application connects to the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool MOOSAPPFACTORY_NAME::OnConnectToServer( void )
{
    // If OnStartup has already run, (re-)register Dynamic Variables
    // to receive updates from the MOOS database
    if (m_OnStartupIsDone)
    {
        RegisterMOOSVariables();
    }

    // << Perform any tasks associated with MOOSDB connection >>

	return true;
}



//=============================================================================
/*
    This function gets called when the application disconnects from the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool MOOSAPPFACTORY_NAME::OnDisconnectFromServer( void )
{
    // << Perform any tasks associated with MOOSDB disconnection >>

	return true;
}

