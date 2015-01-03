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
/** @file iYellowSubDAQ.h
 *
 * @brief
 *  A MOOS instrument for communicating with the DSP data aquisition module in 
 *  the University of Idaho 'YellowSub' AUV
 *  
 * @author Dave Billin
 */
//=============================================================================

#ifndef _IYELLOWSUBDAQ_H_
#define _IYELLOWSUBDAQ_H_

#include <stdint.h>

#include "MOOS/libMOOS/MOOSLib.h"
#include <string>
#include "YellowSubDAQModule.h"


/** The iYellowSubDAQ MOOS application object */
class iYellowSubDAQ : public CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	iYellowSubDAQ( void );

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~iYellowSubDAQ();


	//======================================
	// Methods inherited from CMOOSApp
	//======================================

	//=========================================================================
	/** Called when new mail has arrived from the MOOS Database.
	 * @details
     *  This method will be called at a rate of approximately 1/CommsTick Hz.
     *  In this function you'll most likely interate over the collection of
     *  mail messages received or call a m_Comms::PeekMail() to look for a
     *  specific named message.
     *  
     * @param NewMail
     *  A reference to the list of received mail from the MOOS Database
     *  
     * @returns
     *  true on success; false on error
	 */
	bool OnNewMail(MOOSMSG_LIST & NewMail);


	//=========================================================================
	/** If command message filtering is enabled for this application (via the
     *  EnableCommandMessageFiltering() function), then this function is called
     *  when MOOS mail posted to IYELLOWSUBDAQ_CMD is received.
     *  
     * @param Msg
     *  A copy of the received command message
     *  
     * @return
     *  true on success; false on error
     *  
     * @see CMOOSApp::EnableCommandMessageFiltering
	 */
	bool OnCommandMsg(CMOOSMsg Msg);


	//=========================================================================
    /** This function is where the application can do most of its work.
     * @details
     *  The rate at which Iterate() is called is determined by the value of the
     *  AppTick parameter specified in the (.moos) mission file.  The value of
     *  AppTick is loaded automatically when the application starts up.
	 */
    bool Iterate( void );


	//=========================================================================
    /** Called when the application first starts up
     * @details
     *  This function is called as the application first starts up before any
     *  calls to Iterate() begin.
	 */
    bool OnStartUp( void );


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called when the application is disconnected from the MOOS Database. */
    bool OnDisconnectFromServer( void );



protected:

    //=========================================================================
    /** Loads and applies mission file parameters */
    bool LoadMissionFileParameters( void );

private:
    int m_Verbosity;			/**< Verbosity level for debugging messages */
    int m_BunnySockVerbosity;	/**< Verbosity of BunnySock connection */
    bool m_AppIsOnline;			/**< Set to true at the end of OnStartup() */

    YellowSubDAQModule* m_pDAQ;	/**< Object used to access the DAQ module */

    std::string m_DAQHostname;	/**< Hostname or IP address of the DAQ module */
    int16_t m_DAQPort;		/**< Network port the DAQ module is listening on */
    uint32_t m_ConnectionTimeoutMs; /**< DAQ module connection timeout in ms */

	/** Indices of SUBSCRIBETO_nnn variable names */
	enum e_SubscribedVariables
	{
		NAV_X = 0,		/**< East component of local grid location */
		NAV_Y,			/**< North component of local grid location */
		NAV_HEADING,	/**< Vehicle heading in degrees */
		NAV_DEPTH,		/**< Vehicle depth in meters */
		NAV_PITCH,		/**< Vehicle pitch in radians bounded to +/- PI */
		NAV_ROLL,		/**< Vehicle roll in radians bounded to +/- PI */
		VEHICLE_ID,		/**< Vehicle ID (i.e. sub number) */
		MISSION_ID,		/**< ID of the current mission */
		RUN_ID,			/**< Run number in the current mission */
		NUM_SUBSCRIBED_VARIABLES	/** Internal use only */
	};

	/** Indices of PUBLISHTO_nnn variable names */
	enum e_PublishedVariables
	{
		DAQ_STATUS = 0,	/**< The current state of the DAQ module's
							 disk recording engine as a string */

		NUM_PUBLISHED_VARIABLES	/**< Internal use only */
	};


	/** @var m_SubscribedVarTable
	 * @brief
	 * 	A table of pointers to MOOS variables used for subscribing.  This table
	 * 	is meant to be indexed using elements of e_SubscribedVariables
	 */
	CMOOSVariable* m_SubscribedVarTable[NUM_SUBSCRIBED_VARIABLES];


	/** @var m_PublishedVarTable
	 * @brief
	 * 	A table of pointers to MOOS variables used for publishing.  This table
	 * 	is meant to be indexed using elements of e_PublishedVariables
	 */
	CMOOSVariable* m_PublishedVarTable[NUM_PUBLISHED_VARIABLES];


	/** @var m_SubscribedVarNames
	 * @brief
	 * 	A table containing names of MOOSDB variables this application
	 * 	subscribes to.  This table is meant to be indexed using elements of
	 * 	e_SubscribedVariables
	 */
	std::string m_SubscribedVarNames[NUM_SUBSCRIBED_VARIABLES];

	/** @var m_PublishedVarNames
	 * @brief
	 * 	A table containing names of MOOSDB variables this application
	 * 	publishes to.  This table is meant to be indexed using elements of
	 * 	e_SubscribedVariables
	 */
	std::string m_PublishedVarNames[NUM_PUBLISHED_VARIABLES];


	/** @var sm_DefaultSubscribedVarNames
	 * @brief	Names of default subscribed variables */
	static const char* sm_DefaultSubscribedVarNames[NUM_SUBSCRIBED_VARIABLES];

	/** @var sm_DefaultPublishedVarNames
	 * @brief	Names of default published variables */
	static const char* sm_DefaultPublishedVarNames[NUM_PUBLISHED_VARIABLES];
};

#endif	// END #ifdef _IYELLOWSUBDAQ_H_

