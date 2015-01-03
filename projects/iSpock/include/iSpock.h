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
/** @file iSpock.h

@brief
	Declaration of a MOOS instrument class used to implement communication
	with the Rabbit3000 SPOCK module in the U of I UAV ('Yellow Sub')

@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the
	University of Idaho, USA.
*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef _ISPOCK_H_
#define _ISPOCK_H_

#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core library
#include <string>
#include "SpockModule.h"



#define ISPOCK_VERSION 2.0

//=============================================================================
/** An object encapsulating communication with the "SPOCK" Rabbit3000 sensor
	module in the U of I UAV (a.k.a. 'Yellow Sub')
*/
//=============================================================================
class iSpock : public CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	iSpock( void );

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~iSpock();


	//======================================
	// Methods inherited from CMOOSApp
	//======================================

	//=========================================================================
	/** Called when new mail has arrived from the MOOS Database.
	 * @details
	 * 	This method will be called at a rate of approximately 1/CommsTick Hz.
	 * 	In this function you'll most likely interate over the collection of
	 * 	mail messages received or call a m_Comms::PeekMail() to look for a
	 * 	specific named message.
	 *
	 * @param NewMail
	 * 	A reference to the list of received mail from the MOOS Database
	 *
	 * @return
	 * 	true on success; false on error
	 */
	bool OnNewMail(MOOSMSG_LIST & NewMail);


	//=========================================================================
	/** If command message filtering is enabled for this application (via the
	 * EnableCommandMessageFiltering() function), then this function is called
	 * when a command message having the identifier ISPOCK_CMD is received
	 * from the MOOS Database.
	 *
	 * @param Msg
	 * 	A copy of the received command message
	 *
	 * @return
	 * 	false to signal error else true
	 *
	 * @see CMOOSApp::EnableCommandMessageFiltering
	 */
	bool OnCommandMsg(CMOOSMsg Msg);


	//=========================================================================
    /** This function is where the application can do most of its work.
	 * @details
	 * 	The rate at which Iterate() is called is determined by the value of the
	 * 	AppTick parameter specified in the (.moos) mission file.  The value of
	 * 	AppTick is loaded automatically when the application starts up.
	 */
    bool Iterate( void );


	//=========================================================================
    /** Called when the application first starts up
	 * @details
	 * 	This function is called as the application first starts up before any
	 * 	calls to Iterate() begin.
	 */
    bool OnStartUp( void );


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called when the application is disconnected from the MOOS Database. */
    bool OnDisconnectFromServer( void );


private:
    bool m_AppIsOnline;		/**< Set to true at the end of OnStartup() */

	/** @enum SpockSensorIds
	@brief
		These constants are used to index into a table of MOOS variable names
		each of the various sensors are published to
	*/
	enum e_SpockSensorIds
	{
		SENSOR_COMPASS_HEADING=0,	/**< MOOS Heading (degrees from North) from digital compass */
		SENSOR_COMPASS_YAW,			/**< MOOS Yaw (negative heading in radians) from digital compass */
		SENSOR_COMPASS_PITCH,		/**< MOOS Pitch (radians about x-axis) from digital compass */
		SENSOR_COMPASS_ROLL,		/**< MOOS Roll (radians about y-axis) from digital compass */
		SENSOR_COMPASS_DIP,			/**< Magnetic inclination (dip) from digital compass */

		SENSOR_DEPTH,				/**< Depth (meters) derived from pressure sensor */

		SENSOR_ACCEL_PITCH,			/**< Acceleration about the Y-axis from onboard gyro */
		SENSOR_ACCEL_ROLL,			/**< Acceleration about the X-axis from onboard gyro */

		SENSOR_GPS_LONGITUDE,		/**< Longitude reported by Garmin GPS 18x */
		SENSOR_GPS_LATITUDE,		/**< Latitude reported by Garmin GPS 18x */
		SENSOR_GPS_VELOCITY,		/**< Velocity reported by Garmin GPS 18x */
		SENSOR_GPS_HPE,				/**< Horizontal position error reported by GPS */
		SENSOR_GPS_HEADING,			/**< MOOS Heading (degrees from North) from GPS receiver */
		SENSOR_GPS_YAW,				/**< MOOS Yaw (negative heading in radians) from GPS receiver */
		SENSOR_GPS_HOURS,			/**< Time (hours) from GPS receiver */
		SENSOR_GPS_MINUTES,			/**< Time (minutes) from GPS receiver */
		SENSOR_GPS_SECONDS,			/**< Time (seconds) from GPS receiver */

		SENSOR_WATERLEAKISDETECTED,	/**< Water leak detector (TRUE if leak is detected; else FALSE) */
		SENSOR_TEMPERATURE,			/**< Temperature (Celsius) on the SPOCK module */
		SENSOR_BATTERYVOLTS,		/**< Battery voltage (Volts DC) */
		NUM_SPOCK_SENSOR_IDS		/**< Used internally.  Not a valid ID */
	};

	SpockModule* m_pSpock;	/**< Object encapsulating communication and
    							 interaction with SPOCK */

	std::string m_SpockHostname;	/**< Hostname of the SPOCK module */
	uint16_t m_SpockPort;	/**< Network port to connect to on the SPOCK
								 module */

	bool m_OnlyPublishChanges;	/**< Set to true if sensor variables should
									 only be published when they change; else
									 false (default) to publish values every
									 time Iterate() is called */

	int m_Verbosity;	/**< Verbosity level used for debugging messages */


	/** @var m_SensorVarNames
	 * @brief
	 * 	An array of names of MOOS variables to publish SPOCK sensors to.  The
	 * 	elements of this array correspond to the elements of e_SpockSensorIds.
	 */
	std::string m_PublishedVarNames[NUM_SPOCK_SENSOR_IDS];


	/** @var m_PublishedVarValues
	 * @brief
	 * 	An array of SPOCK sensor values for publishing to the MOOSDB.  The
	 * 	elements of this array correspond to the elements of e_SpockSensorIds.
	 */
	double m_PublishedVarValues[NUM_SPOCK_SENSOR_IDS];


	/** @var sm_DefaultPublishedVariables
	 * @brief
	 * 	Default variable names sensors will be published to.  The elements of
	 * 	this array correspond to the elements of e_SpockSensorIds
	 */
	static const char* sm_DefaultPublishedVariables[NUM_SPOCK_SENSOR_IDS];


    //=========================================================================
    /** Loads and applies mission file parameters */
    bool LoadMissionFileParameters( void );

    //=========================================================================
    /** Returns a double containing a specified SPOCK sensor value */
    double GetSpockSensorAsDouble( int SensorId );

	// Prevent automatic generation of copy constructor and assignment operator
	iSpock (const iSpock&);
    const iSpock& operator= (const iSpock&);
};



#endif	// END #ifdef _ISPOCK_H_
