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
/** @file pVehicleEKF.h

@brief
	A MOOS process that continually tracks the vehicle's position

@author Dave Billin

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _PVEHICLEEKF_H_
#define _PVEHICLEEKF_H_

#include <string>
#include <vector>
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

#include "LibYellowSubNav.h"


// The application's software revision
#define APPLICATION_SOFTWARE_VERSION 	0.1




//=============================================================================
/** This is the main application object for the pVehicleEKF
	MOOS application.
*/
//=============================================================================
class pVehicleEKF : public  CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the MOOS application object */
	pVehicleEKF( void );

	//=========================================================================
	/** Called when the application closes */
	virtual ~pVehicleEKF();


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
		PPOSITIONAGENT_CMD is recieved from the MOOS Database.

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
    bool OnStartUp();


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called when the application is disconnected from the MOOS Database. */
    bool OnDisconnectFromServer( void );


    //=========================================================================
    /** Helper function to load mission file parameters */
    bool LoadMissionParameters( void );


private:
    bool m_AppIsInitialized;
	CMOOSGeodesy m_ReferenceGeodesy;	/**< Geodesy object used to map between
											 latitude/longitude and the local
											 coordinate grid */

	YellowSubNav::VehicleEKF m_EKF;	/**< The EKF object that this app wraps */

	double m_DepthThreshold;	/**< Depth threshold used for switching between
									 publishing EKF estimates and sensors */
	double m_GpsHPEThreshold;	/**< HPE threshold used for switching between
									 publishing EKF estimates and sensors */


	/** Indices of SUBSCRIBETO_nnn variable names */
	enum e_SubscribedVariables
	{
		GPS_LATITUDE = 0,
		GPS_LONGITUDE,
		GPS_HEADING,
		GPS_VELOCITY,
		GPS_HPE,
		DEPTH,
		COMPASS_HEADING,
		PITCH,
		ROLL,
		WZ,
		WY,
		LBL_2WAYTRAVELTIME,
		LBL_RANGEPERIOD,
		RPM_VELOCITY_ESTIMATE,
		NUM_SUBSCRIBED_VARIABLES
	};

	/** Indices of PUBLISHTO_nnn variable names */
	enum e_PublishedVariables
	{
		NAV_X = 0,		/**< Local grid East coordinate (meters) */
		NAV_Y,			/**< Local grid North coordinate (meters) */
		NAV_DEPTH,		/**< The vehicle's current depth (meters) */
		NAV_HEADING,	/**< The vehicle's current heading (degrees) */
		NAV_YAW,		/**< The vehicle's current yaw (radians) */
		NAV_SPEED,		/**< The vehicle's current speed (meters per second) */
		YAWBIAS,		/**< Estimated yaw bias (radians) in sensors */
		NAV_PITCH,		/**< Vehicle pitch */
		NAV_ROLL,		/**< Vehicle roll */
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
	string m_SubscribedVarNames[NUM_SUBSCRIBED_VARIABLES];

	/** @var m_PublishedVarNames
	 * @brief
	 * 	A table containing names of MOOSDB variables this application
	 * 	publishes to.  This table is meant to be indexed using elements of
	 * 	e_SubscribedVariables
	 */
	string m_PublishedVarNames[NUM_PUBLISHED_VARIABLES];


	/** @var sm_DefaultSubscribedVarNames
	 * @brief	Names of default subscribed variables */
	static const char* sm_DefaultSubscribedVarNames[];

	/** @var sm_DefaultPublishedVarNames
	 * @brief	Names of default published variables */
	static const char* sm_DefaultPublishedVarNames[];



	/** Updates published MOOS variables.  New values will be posted if the
	 * value of a published variable has changed, or if it has not been
	 * published for 1 second
	 *
	 * @param N
	 * 	Current vehicle North coordinate
	 *
	 * @param E
	 * 	Current vehicle East coordinate
	 *
	 * @param Speed
	 * 	Current vehicle speed
	 *
	 * @param Yaw
	 * 	Current vehicle yaw (also used to publish heading)
	 *
	 * @param Bias
	 * 	Sensor yaw bias
	 *
	 * @param Depth
	 * 	Current vehicle depth
	 *
	 * @param t
	 * 	MOOSTime to associate with the variable update
	 */
	void UpdatePublishedVariables( double N, double E, double Speed,
								   double Yaw, double Bias, double Depth,
								   double t );

	/** Initializes the VehicleEKF object with current values
	 * @param t The current MOOSTime */
	void InitEKFObject( double t );

	// Prevent automatic generation of copy constructor and assignment operator
	pVehicleEKF (const pVehicleEKF&);
    const pVehicleEKF& operator= (const pVehicleEKF&);

};



#endif	// END #ifdef _PVEHICLEEKF_H_
