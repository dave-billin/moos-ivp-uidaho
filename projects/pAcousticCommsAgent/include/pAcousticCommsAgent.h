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
/** @file pAcousticCommsAgent.h

@brief
	A MOOS process to drive the timing of acoustic network traffic and
	navigation pings.

@author Dave Billin

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _PACOUSTICCOMMSAGENT_H_
#define _PACOUSTICCOMMSAGENT_H_

#include <string>
#include <list>
#include <set>
#include "MOOS/libMOOS/MOOSLib.h"	// MOOS core library
#include "InfoBuffer.h"
#include "TimeSlice.h"

/*------------------------------------------------
	<<< Add other #include statements here >>>
------------------------------------------------*/


// The application's software revision
#define APPLICATION_SOFTWARE_VERSION 1.0



//=====================================
// CONSTANTS AND MACROS

/*------------------------------------------------
	<<< Add your #define statements here >>>
------------------------------------------------*/





//=============================================================================
/** This is the main application object for the pAcousticCommsAgent
	MOOS application.


	Currently, pAcousticCommsAgent simply sends navigation pings at a regular
	interval.

@todo
	Implement a multi-vehicle timing cycle consisting of slots and tasks
*/
//=============================================================================
class pAcousticCommsAgent : public  CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the MOOS application object */
	pAcousticCommsAgent( void );

	//=========================================================================
	/** Called when the application closes */
	~pAcousticCommsAgent();


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
		PACOUSTICCOMMSAGENT_CMD is recieved from the MOOS Database.

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


    //=========================================================================
    /** Returns the value of MOOSTime() the last midnight came around */
    double GetMoosTimeAtMidnight( void );


    //=========================================================================
    /** Returns a pointer to the time slice that is active at a specified
     *  number of seconds into the communications cycle.
     *
     * @param SecondsInCommsCycle
     * 	A positive value indicating a number of seconds in the communications
     * 	cycle.  If this value is greater than the total length of the comms
     * 	cycle, a modulus is taken with respect to the total comms cycle
     * 	duration.
     *
     * @return
     * 	A pointer to the TimeSlice active at the indicated time in the comms
     * 	cycle, or NULL if no TimeSlices are active.
     */
    TimeSlice* GetActiveTimeSlice( double SecondsInCommsCycle );

private:
    bool m_AppIsInitialized;			/**< Set to true at the end of
    										 OnStartup() */

    InfoBuffer m_InfoBuffer;			/**< Info buffer for MOOS variables
    										 used in time slice CONDITIONS */

    std::vector<double> m_SliceTimes;   /**< Each element contains the seconds
    										 in the comms cycle when the
    										 corresponding element of
    										 m_TimeSliceList becomes active */

    std::list<TimeSlice> m_TimeSliceList;	/**< Comms cycle time slices */

    std::set<std::string> m_ConditionVarNames;	/**< Names of MOOS variables
     	 	 	 	 	 	 	 	 	 	         used in time slice
     	 	 	 	 	 	 	 	 	 	         CONDITIONS */

    double m_CommsCycleDuration_sec;	/**< Total comms cycle duration as the
    										 sum of all time slices */

    double m_MoosTimeAtMidnight;		/**< Value of MOOSTime() value the last
    										 time midnight rolled around.  Used
    										 to calculate seconds elapsed since
    										 midnight */

    //=========================================================================
    /** Loads and configures the communications cycle from a specified time
     *  slice configuration (typically .cyc) file
     *
     * @param sFilePath
     *	Absolute path of the time slice configuration file to load from
     *
     * @return
     * 	true if the file was loaded successfully; else false if the file could
     * 	not be loaded or a parsing error occurs
     */
    bool LoadCommsCycleFromFile( std::string& sFilePath );


	// Prevent automatic generation of copy constructor and assignment operator
	pAcousticCommsAgent (const pAcousticCommsAgent&);
    const pAcousticCommsAgent& operator= (const pAcousticCommsAgent&);
};



#endif	// END #ifdef _PACOUSTICCOMMSAGENT_H_
