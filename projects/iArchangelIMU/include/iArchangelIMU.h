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
/** @file iArchangelIMU.h
 *
 * @brief
 *   A MOOS instrument class used to implement communication with an Archangel
 *   IMU in the U of I 'Yellow Sub' AUV
 *
 * @author Dave Billin
 *
 * @par Created for
 * 	The Microcomputer Research and Communications Institute (MRCI) - at the
 * 	University of Idaho, USA.
 */
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef _IARCHANGELIMU_H_
#define _IARCHANGELIMU_H_

#include <string>
#include <stdint.h>

#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core library
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "LinuxSerialPortEx.h"
#include "IMU3Module.h"

// Software version
#define SOFTWARE_REVISION 1.0


//=============================================================================
/** @brief
 * 	A MOOS instrument to publish data from an Archangel IMU3 inertial
 *  measurement unit.
 *
 * @ingroup iArchangelIMU
*/
//=============================================================================
class iArchangelIMU : public CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	iArchangelIMU( void );

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~iArchangelIMU();


	//======================================
	// Methods inherited from CMOOSApp
	//======================================

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

protected:

    //=========================================================================
    /** Loads and applies mission file parameters */
    bool LoadMissionFileParameters( void );

    //=========================================================================
    bool ConfigureSerialPort( void );


    /** ID's used to index numeric values reported by the IMU.  Note: all axes
     * and angles are referenced to MOOS conventions (not necessarily those of
     * the Archangel IMU)
     */
    enum e_Imu3SensorIds
    {
    	IMUSENSOR_DeltaAngle_Roll = 0,	/**< Gyro-measured change in angle
    										 about the Y-axis (rad/sec) */

    	IMUSENSOR_DeltaAngle_Pitch,		/**< Gyro-measured change in angle
    										 about the X-axis (rad/sec) */

    	IMUSENSOR_DeltaAngle_Yaw,		/**< Gyro-measured change in angle
    										 about the Z-axis (rad/sec) */

    	IMUSENSOR_DeltaVLongitudinal,	/**< Accelerometer-measured change in
    									     velocity along the vehicle's X
    									     axis (m/sec) */

    	IMUSENSOR_DeltaVLateral,		/**< Accelerometer-measured change in
    									     velocity along the vehicle's Y
    									     axis (m/sec) */

    	IMUSENSOR_DeltaVNormal,			/**< Accelerometer-measured change in
    									     velocity along the vehicle's Z
    									     axis (m/s) */

    	IMUSENSOR_AHRS_Roll,		/**< AHRS-computed vehicle roll (rad
    									 bounded to +/- pi) */

    	IMUSENSOR_AHRS_Pitch,		/**< AHRS-computed vehicle pitch (rad
    									 bounded to +/- pi) */

    	IMUSENSOR_AHRS_Yaw,			/**< AHRS-computed vehicle yaw (rad
    									 bounded to +/- pi) */

    	IMUSENSOR_AHRS_RollRate,	/**< AHRS-computed roll rate (rad/s) */
    	IMUSENSOR_AHRS_PitchRate,	/**< AHRS-computed pitch rate (rad/s) */
    	IMUSENSOR_AHRS_YawRate,		/**< AHRS-computed yaw rate (rad/s) */

    	NUM_IMU3_SENSOR_VALUES		/**< Internal use only */
    };


private:
    CMOOSSerialPort* m_SerialPort;	/**< Serial port used to communicate with
                                         the IMU module */

    bool m_UseSimDataFile;  /**< true if a file is being used as the source of
                                 IMU data rather than a serial connection */

    IMU3Module m_Imu;	/**< Object used to receive data from the IMU */

	int m_Verbosity;	/**< Verbosity of debugging messages */

	bool m_MOOSDBIsConnected;   /**< set to true once the Comms thread has
	                                 connected to the MOOS database */

	float m_DeltaRoll_Polarity;
	float m_DeltaPitch_Polarity;
	float m_DeltaYaw_Polarity;

	float m_DeltaVLongitudinal_Polarity;
	float m_DeltaVLateral_Polarity;
	float m_DeltaVNormal_Polarity;

	float m_InertialPitchPolarity;
	float m_InertialRollPolarity;
	float m_InertialYawPolarity;

	/** @var m_PublishedVarNames
	 * @brief
	 *  An array of names of MOOS variables IMU3 data is published to
	 */
	std::string m_PublishedVarNames[NUM_IMU3_SENSOR_VALUES];

	/** @var m_PublishedVarNames
	 * @brief
	 *  An array of default names of MOOS variables IMU3 data is published to
	 */
	static const char* sm_DefaultPublishedVarNames[NUM_IMU3_SENSOR_VALUES];


	std::string& AppendToCSVList( const char* szStringToAppend,
	                              std::string& sTarget );


	// Prevent automatic generation of copy constructor and assignment operator
	iArchangelIMU (const iArchangelIMU&);
    const iArchangelIMU& operator= (const iArchangelIMU&);
};


#endif	// END #ifdef _IARCHANGELIMU_H_
