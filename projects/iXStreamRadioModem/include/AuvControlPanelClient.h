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
/*! @file AuvControlPanelClient.h

@brief
	Classes and functions used to implement communication via radio modem with
	the AUV Control	Panel software

@author Dave Billin

@par Created For
	The University of Idaho Microelectronics Research and Communications
	Institute (MRCI)
*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _AUVCONTROLPANELCLIENT_H_
#define _AUVCONTROLPANELCLIENT_H_

#include "MOOS/libMOOS/MOOSLib.h"   // MOOS core library
#include <stdint.h>

//=============================================================================
/** Function called to receive and handle radio messages from the AUV Control
	Panel software
*/
void ServiceAUVControlPanelRx( CMOOSSerialPort& m_Port, CMOOSCommClient& m_Comms );


//=============================================================================
/** This packet is sent to the base station telling it whether it should send
	ROV commands to an AUV
*/
class RM_ROVEnablementPacket
{
public:

	//==========================================================================
	/** Creates an instance of the object
	@param ModemAddress
		The address of the radio modem being configured
	@param ShouldEnableROVCommands
		true if the base station should send ROV commands to the specified
		radio modem; else false to disable sending ROV commands
	 */
	RM_ROVEnablementPacket( unsigned char ModemAddress,
								bool ShouldEnableROVCommands )
	{
		m_RawBytes[0] = 'N';
		m_RawBytes[1] = (char)ModemAddress;
		m_RawBytes[2] = (ShouldEnableROVCommands) ? 1:0;
	}

	//==========================================================================
	/** Returns the length of serialized data in an ROV Enablement packet */
	int GetNumBytes(void) { return 3; }

	//==========================================================================
	/** Sets whether the packet should enable (true) or disable (false) the
     *  base station sending ROV commands
     *
	 * @param ShouldEnableROVCommands
	 *	true if the base station should send ROV commands; else false
	 */
	void SetRovCommandsEnabled( bool ShouldEnableROVCommands )
	{
		m_RawBytes[2] = (ShouldEnableROVCommands) ? 1:0;
	}

	//==========================================================================
	/** Returns a pointer to the raw serialized data of the packet */
	const char* Serialize( void )
	{
		return m_RawBytes;
	}

	char m_RawBytes[3];
};


//=============================================================================
/** A class encapsulating data in a sensor report sent via radio modem to the
	AUV Control Panel software
*/
class RadioModemSensorReport
{
public:

	//! Creates an empty sensor report with all fields zeroed.
	RadioModemSensorReport( void );

	//! Called when the object goes out of scope.
	~RadioModemSensorReport();

	uint8_t VehicleId;	/**< ID of the vehicle reporting sensors */
	float CompassHeading_deg;	/**< Digital compass heading (degrees) */
	float Depth_cm;				/**< Depth (cm) from pressure sensor */
	float BatteryVolts;			/**< Battery voltage (Volts) */
	short H20Leak;				/**< Leak sensor (1=no leak; 0=leak) */
	float Temperature_C;		/**< Temp (Celcius) inside vehicle chassis */
	float Longitude_deg;		/**< Longitude of vehicle */
	float Latitude_deg;			/**< Latitude of vehicle */
	float Velocity_mps;			/**< Velocity of vehicle (meters/sec) */
	float GpsHPE;				/**< GPS horizontal position error */
	float GpsHeading;			/**< GPS heading */
	float Acceleration_X;		/**< X acceleration from SPOCK */
	float Acceleration_Y;		/**< Y acceleration from SPOCK */
	char Pad;					/**< Padding Byte (not used) */
	char MissionNumber;  			/**< Current mission underway */
	uint8_t RunNumber;		/**< Current run number underway */
	uint8_t PropSpeedIndex;	/**< Propeller speed index */
	int8_t LastAbortCode;  			/**< Last abort code received */
	int8_t LBL_TransponderFlags;	/**< Active LBL buoy flags */
	float Coordinate_North;		/**< Geodesy North (Y) coordinate */
	float Coordinate_East;		/**< Geodesy East (X) coordinate */


	//==========================================================================
	/** Serializes data in the sensor report into a form suitable for sending to
		the AUV Control Panel software.

	@param[out] OUT_NumBytes
		Reference to a variable to be populated with the number of Bytes
		in the buffer returned by this function

	@return
		Pointer to an internal buffer containing the serialized sensor report
	*/
	const char* Serialize( void );


	//==========================================================================
	//! Returns the number of Bytes in a serialized sensor report
	int GetNumSerialBytes(void);

private:

	uint8_t* m_SerialBuffer;	/**< Buffer used to serialize data */

	/** Byte offsets of sensor data in a serialized radio modem sensor report */
	enum e_SensorReportByteOffsets
	{
		BYTEOFFSET_OPCODE = 0,
		BYTEOFFSET_VEHICLEID = 1,
		BYTEOFFSET_COMPASSHEADING = 2,
		BYTEOFFSET_DEPTH_CM = 6,
		BYTEOFFSET_BATTERYVOLTS = 10,
		BYTEOFFSET_H20LEAK = 14,
		BYTEOFFSET_TEMPERATURE_C = 16,
		BYTEOFFSET_LONGITUDE = 20,
		BYTEOFFSET_LATITUDE = 24,
		BYTEOFFSET_VELOCITY = 28,
		BYTEOFFSET_GPS_HPE = 32,
		BYTEOFFSET_GPS_HEADING = 36,
		BYTEOFFSET_ACCELERATION_X = 40,
		BYTEOFFSET_ACCELERATION_Y = 44,
		BYTEOFFSET_MISSION_NUMBER = 48,
		BYTEOFFSET_RUN_NUMBER = 49,
		BYTEOFFSET_PROP_SPEED_INDEX = 50,
		BYTEOFFSET_LAST_ABORT_CODE = 51,
		BYTEOFFSET_TRANSPONDER_FLAGS = 52,
		BYTEOFFSET_COORDINATE_NORTH = 53,
		BYTEOFFSET_COORDINATE_EAST = 57,
		SENSOR_REPORT_NUM_BYTES = 61	/**< Number of Bytes in a sensor report transmission */
	};

};



#endif 	// END #ifndef _AUVCONTROLPANELCLIENT_H_
