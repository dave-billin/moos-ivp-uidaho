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
/*! @file AuvControlPanelClient.cpp

@brief
	Implementation of classes and functions defined in AuvControlPanelClient.h

@author Dave Billin

@par Created For
	The University of Idaho Microelectronics Research and Communications
	Institute (MRCI)
*/
//=============================================================================

#include <string.h>
#include "AuvControlPanelClient.h"



//=============================================================================
RadioModemSensorReport::RadioModemSensorReport( void )
:	VehicleId(0),
	CompassHeading_deg(0.0f),
	Depth_cm(0.0f),
	BatteryVolts(0.0f),
	H20Leak(0),
	Temperature_C(0.0f),
	Longitude_deg(0.0f),
	Latitude_deg(0.0f),
	Velocity_mps(0.0f),
	GpsHPE(0.0f),
	GpsHeading(0.0f),
	Acceleration_X(0.0f),
	Acceleration_Y(0.0f),
	MissionNumber(0),
	RunNumber(0),
	PropSpeedIndex(0),
	LastAbortCode(0),
	LBL_TransponderFlags(0),
	Coordinate_North(0.0f),
	Coordinate_East(0.0f)
{
	m_SerialBuffer = new uint8_t[SENSOR_REPORT_NUM_BYTES];
}




//=============================================================================
RadioModemSensorReport::~RadioModemSensorReport()
{
	if (m_SerialBuffer != NULL)		delete[] m_SerialBuffer;
}



//=============================================================================
const char* RadioModemSensorReport::Serialize( void )
{
	memset(m_SerialBuffer, 0, SENSOR_REPORT_NUM_BYTES);
	m_SerialBuffer[BYTEOFFSET_OPCODE] = 'S';
	m_SerialBuffer[BYTEOFFSET_VEHICLEID] = VehicleId;
	*((float*)(m_SerialBuffer + BYTEOFFSET_COMPASSHEADING)) = CompassHeading_deg;
	*((float*)(m_SerialBuffer + BYTEOFFSET_DEPTH_CM)) = Depth_cm;
	*((float*)(m_SerialBuffer + BYTEOFFSET_BATTERYVOLTS)) = BatteryVolts;
	*((short*)(m_SerialBuffer + BYTEOFFSET_H20LEAK)) = H20Leak;
	*((float*)(m_SerialBuffer + BYTEOFFSET_TEMPERATURE_C)) = Temperature_C;
	*((float*)(m_SerialBuffer + BYTEOFFSET_LONGITUDE)) = Longitude_deg;
	*((float*)(m_SerialBuffer + BYTEOFFSET_LATITUDE)) = Latitude_deg;
	*((float*)(m_SerialBuffer + BYTEOFFSET_VELOCITY)) = Velocity_mps;
	*((float*)(m_SerialBuffer + BYTEOFFSET_GPS_HPE)) = GpsHPE;
	*((float*)(m_SerialBuffer + BYTEOFFSET_GPS_HEADING)) = GpsHeading;
	*((float*)(m_SerialBuffer + BYTEOFFSET_ACCELERATION_X)) = Acceleration_X;
	*((float*)(m_SerialBuffer + BYTEOFFSET_ACCELERATION_Y)) = Acceleration_Y;
	m_SerialBuffer[BYTEOFFSET_MISSION_NUMBER] = MissionNumber;
	m_SerialBuffer[BYTEOFFSET_RUN_NUMBER] = RunNumber;
	m_SerialBuffer[BYTEOFFSET_PROP_SPEED_INDEX] = PropSpeedIndex;
	m_SerialBuffer[BYTEOFFSET_LAST_ABORT_CODE] = LastAbortCode;
	m_SerialBuffer[BYTEOFFSET_TRANSPONDER_FLAGS] = LBL_TransponderFlags;
	*((float*)(m_SerialBuffer + BYTEOFFSET_COORDINATE_NORTH)) = Coordinate_North;
	*((float*)(m_SerialBuffer + BYTEOFFSET_COORDINATE_EAST)) = Coordinate_East;

	return (char*)m_SerialBuffer;
}


//=============================================================================
int RadioModemSensorReport::GetNumSerialBytes(void)
{
	return SENSOR_REPORT_NUM_BYTES;
}






//=============================================================================

/* 	A magic number from UHURA source code used to convert % thrust sent over
	the radio modem to an RPM value used by SCOTTY */
#define THRUST_PERCENTAGE_TO_RPM	20

// Size of the buffer used to receive serial data from the modem
#define RX_BUFFER_NUM_BYTES	20		// DB: Rabbit code uses 20-Byte buffer


void ServiceAUVControlPanelRx( CMOOSSerialPort& m_Port, CMOOSCommClient& m_Comms )
{
	static char RxBuffer[RX_BUFFER_NUM_BYTES];	// Data is received into this buffer

	int Rudder_deg;		// Rudder setting from control packet in +/- degrees
	int Elevator_deg;	// Elevator setting from control packet in +/- degrees
	double Rudder_rad;	// Rudder setting translated from radio control packet
	double Elevator_rad;	// Elevator setting translated from radio control packet
	short ThrustPercent;	// Thrust as percentage of full scale from radio control packet

	// Read an opcode character into the first element of RxBuffer
	// Give up after 50 ms
	if ( m_Port.ReadNWithTimeOut(RxBuffer, 1, 0.05) > 0 )
	{
		switch (RxBuffer[0])
		{
			case 'A':		// Abort
			{
				m_Comms.Notify("iXRM_RxAbort", "TRUE");
				break;
			}

			case 'C':		// Control values
			{
				// Read the remaining control values.
				// Give up after 100 ms
				if (m_Port.ReadNWithTimeOut(RxBuffer, 6, 0.1) == 6)
				{
					// Assemble the control parameters
					Rudder_deg = ((RxBuffer[0] << 8) | RxBuffer[1]);// * 10;
					Elevator_deg = ((RxBuffer[2] << 8) | RxBuffer[3]);// * 10;
					//Aileron_deg = 0;		// DB: yep.  It was this way in UHURA...
					// DB: Leave thrust as percentage, as this jives with MOOS conventions
					//ThrustPercent = (double)(((RxBuffer[4] << 8) | RxBuffer[5]));// * THRUST_PERCENTAGE_TO_RPM);
					ThrustPercent = (RxBuffer[4] << 8) | RxBuffer[5];

					// Translate the radio commands to MOOS conventions
					Rudder_rad = MOOS_ANGLE_WRAP( MOOSDeg2Rad((double)Rudder_deg) );
					Elevator_rad = MOOS_ANGLE_WRAP( MOOSDeg2Rad((double)Elevator_deg) );

					// Publish the control commands to the MOOS database
					m_Comms.Notify("DESIRED_RUDDER", Rudder_rad);
					m_Comms.Notify("DESIRED_ELEVATOR", Elevator_rad);
					m_Comms.Notify("DESIRED_THRUST", (double)ThrustPercent);

					//m_Comms.Notify("iXRM_RovCommand",
					//			   MOOSFormat("RudderDeg=%d,ElevatorDeg=%d,Thrust=%d%%",
					//						  Rudder_deg, Elevator_deg, ThrustPercent) );

					// DB: Aileron isn't currently sent by the AUV Control Panel software, so let's not
					//	   report it.
					//m_Comms.Notify("iXRM_RovCommand",
					//			   MOOSFormat("RudderDeg=%d,ElevatorDeg=%d,AileronDeg=%d,PropRPM=%d",
					//						  Rudder_deg, Elevator_deg, Aileron_deg, PropRPM) );
				}
				else
				{
					MOOSTrace("WARNING: Dropped ROV radio command after 100 ms\n");
				}
				break;
			}


			case 'M':
			{
				// Read a single Byte containing the mission number to start.
				// give up after 100 ms
				if ( m_Port.ReadNWithTimeOut(RxBuffer, 1, 0.1) == 1 )
				{
					m_Comms.Notify("ABORT_CODE", "NONE");
					m_Comms.Notify( "iXRM_StartMission",
									MOOSFormat("%d", RxBuffer[0]) );
				}
				else
				{
					MOOSTrace("WARNING: Dropped Start Mission radio command after 100 ms\n");
				}
				break;
			}

			default:
			{
				MOOSFormat("Unknown AUV radio opcode: %c\n", RxBuffer[0]);
			}
		}
	}

}
