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
/** @file SpockModule.cpp
 *
 * @brief
 *	Implementation of the SpockModule class used to provide an interface to the
 *	SPOCK Rabbit module on the U of I YellowSub AUV
 *
 * @author	Dave Billin
 */
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "SpockModule.h"

using namespace std;
using namespace BunnySock;


#define LOCAL_DEVICEID		2	// Send packets as KIRK
#define SPOCK_DEVICEID		3	// BunnySock ID of the SPOCK module

// Macros to implement verbosity-dependent conditional statements
#define SPOCKVERBOSE1(_expr_)	if (m_Verbosity >= 1) { (_expr_); }
#define SPOCKVERBOSE2(_expr_)	if (m_Verbosity >= 2) { (_expr_); }
#define SPOCKVERBOSE3(_expr_)	if (m_Verbosity >= 3) { (_expr_); }



/** @enum e_SensorZeroIds
 * @brief
 * 	ID's used to specify which sensors on the SPOCK module to zero
 */
enum e_SensorZeroIds
{
	ZERO_ID_ALL = 0,
	ZERO_ID_DEPTH
};


//=============================================================================
SpockModule::SpockModule( string& sHostName, uint16_t Port, int Verbosity )
: m_pNode(NULL),
  m_Verbosity(Verbosity),
  m_AutoReportingEnabled(false),
  m_SensorValuesAreFresh(false),
  m_Depth(0.0),
  m_BatteryVoltage(0.0),
  m_WaterLeakIsDetected(false),
  m_Temperature(0.0),
  m_GPS_Longitude(0.0),
  m_GPS_Latitude(0.0),
  m_GPS_Velocity(0.0),
  m_GPS_HPE(0.0),
  m_GPS_Heading(0.0),
  m_GPS_Hours(0),
  m_GPS_Minutes(0),
  m_GPS_Seconds(0),
  m_AccelPitch(0.0),
  m_AccelRoll(0.0),
  m_CompassHeading(0.0),
  m_CompassPitch(0.0),
  m_CompassRoll(0.0),
  m_CompassDip(0.0)
{
	BunnySockTcpNode* pTcpNode;

	pTcpNode = new BunnySockTcpNode( BunnySockTcpNode::CLIENT, /* mode */
									 sHostName,		/* Remote host */
									 Port, 			/* Remote Port */
									 LOCAL_DEVICEID,/* Device ID to send as */
									 1, 			/* Retry period (sec) */
									 3000, 		/* Connection timeout (ms) */
									 Verbosity );	/* Verbosity */

	// Verify that we created a BunnySock node
	if (pTcpNode == NULL)
	{
		string s = "SpockModule: Failed to create BunnySock TCP client node "
				   "to connect to " + sHostName +
				   MOOSFormat(":%d",Port) + "\n";
		throw CMOOSException(s);
	}

	m_pNode = pTcpNode;
	m_pNode->AddListener(this);	// Register for BunnySock packets and events
	m_pNode->Start();	// Start the BunnySock connection's network thread
}




//=============================================================================
SpockModule::~SpockModule()
{
	if (m_pNode != NULL)
	{
		delete m_pNode;
	}
}



//=============================================================================
bool SpockModule::IsConnected( void ) const
{
	return m_pNode->IsConnected();
}



//=============================================================================
void SpockModule::RequestMultiSensors( void )
{
	if ( m_pNode->IsConnected() )
	{
		BunnySockPacket ReqPacket;
		CommandPacket_t* pPacket = (CommandPacket_t*)ReqPacket.GetRawBytes();

		ReqPacket.SetHeader(TYPE_COMMAND, LOCAL_DEVICEID, SPOCK_DEVICEID, 0);

		pPacket->CommandId = CMD_REQUEST_SENSORS;
		pPacket->Parameter[0] = SENSORS;

		m_pNode->SendPacket(ReqPacket);
	}
}



//=============================================================================
void SpockModule::RequestDepth( void )
{
	if (m_pNode->IsConnected())
	{
		BunnySockPacket ReqPacket;
		CommandPacket_t* pPacket = (CommandPacket_t*)ReqPacket.GetRawBytes();

		ReqPacket.SetHeader(TYPE_COMMAND, LOCAL_DEVICEID, SPOCK_DEVICEID, 0);

		pPacket->CommandId = CMD_REQUEST_SENSORS;
		pPacket->Parameter[0] = HI_RATE_DEPTH;

		m_pNode->SendPacket(ReqPacket);
	}
}



//=============================================================================
void SpockModule::SetAutoReportingEnablement( bool EnableAutoReporting )
{
	m_AutoReportingEnabled = EnableAutoReporting;

	if (m_pNode->IsConnected())
	{
		BunnySockPacket ReqPacket;
		CommandPacket_t* pPacket = (CommandPacket_t*)ReqPacket.GetRawBytes();

		ReqPacket.SetHeader(TYPE_COMMAND, LOCAL_DEVICEID, SPOCK_DEVICEID, 0);

		pPacket->CommandId = CMD_ENABLE_SENSOR_UPDATES;
		pPacket->Parameter[0] = (EnableAutoReporting) ? 1 : 0;

		// If we're disabling auto-reporting, we need to send the 'magic' key
		// values
		if (EnableAutoReporting == false)
		{
			pPacket->Parameter[1] = 0x01ff;
			pPacket->Parameter[2] = 0xfe02;
			pPacket->Parameter[3] = 0x03fd;
			pPacket->Parameter[4] = 0xfc04;
		}

		m_pNode->SendPacket(ReqPacket);
	}
}




//=============================================================================
void SpockModule::ZeroSensor( int SensorID )
{
	if (m_pNode->IsConnected())
	{
		BunnySockPacket ReqPacket;
		CommandPacket_t* pPacket = (CommandPacket_t*)ReqPacket.GetRawBytes();

		ReqPacket.SetHeader(TYPE_COMMAND, LOCAL_DEVICEID, SPOCK_DEVICEID, 0);

		pPacket->CommandId = CMD_ZERO;

		switch (SensorID)
		{
			case SensorId_All:
				pPacket->Parameter[0] = ZERO_ID_ALL;
				break;

			case SensorId_Depth:
				pPacket->Parameter[0] = ZERO_ID_DEPTH;
				break;
		}

		m_pNode->SendPacket(ReqPacket);
	}
}


//=============================================================================
bool SpockModule::SensorValuesAreFresh( void )
{
	bool b = m_SensorValuesAreFresh;
	m_SensorValuesAreFresh = false;

	return b;
}


//=============================================================================
void SpockModule::OnPacketReceived( BunnySockPacket& RxPacket,
									BunnySockNode& Node,
									double TimeStamp_sec )
{
	SensorPacket_t* pSensorPacket;
	DepthPacket_t* pDepthPacket;

	switch (RxPacket.GetHeader()->PacketType)
	{
		case TYPE_SENSOR:
		{
			pSensorPacket =
					reinterpret_cast<SensorPacket_t*>(RxPacket.GetRawBytes());
			HandleMultiSensorPacket(pSensorPacket);
			break;
		}

		case TYPE_DEPTH:
		{
			pDepthPacket =
					reinterpret_cast<DepthPacket_t*>(RxPacket.GetRawBytes());
			HandleDepthPacket(pDepthPacket);
			break;
		}

		default:
			return;
	}
}



//=============================================================================
void SpockModule::OnConnectionEvent( int EventId, BunnySockNode& Node,
									 double TimeStamp_sec )
{
	string s;

	switch (EventId)
	{
		case BunnySockListener::CONNECTED:
			s = "Connected to SPOCK\n";
			break;

		// If SCOTTY disconnects, reset all reported variable values
		case BunnySockListener::CONNECTION_TIMEOUT:
			s = "Connection to SPOCK timed out!\n";
			break;

		case BunnySockListener::CONNECTION_ERROR:
			s = "An error occurred on the connection to SPOCK!\n";
			break;

		case BunnySockListener::DISCONNECTED:
			s = "Connection to SPOCK closed.\n";
			break;

		default:
			return;
	}

	MOOSTrace(s);	// Print the event


	if (EventId == BunnySockListener::CONNECTED)
	{
		//----------------------------------------
		// STATUS:
		//	A connection with SPOCK just opened.
		//	Set auto-reporting enablement to the
		//	last setting
		//----------------------------------------
		SetAutoReportingEnablement(m_AutoReportingEnabled);
	}
	else
	{
		//------------------------------------------
		// STATUS:
		//	The connection with SPOCK has closed.
	}

	m_SensorValuesAreFresh = false;
}



//=============================================================================
void SpockModule::HandleMultiSensorPacket( SensorPacket_t* pPacket )
{
	SPOCKVERBOSE1( MOOSTrace("Received a sensor packet from SPOCK\n") );

	  m_Depth = pPacket->Depth_cm * 0.01;	// Convert depth to meters

	  m_WaterLeakIsDetected = (pPacket->WaterLeakDetected == 0);
	  m_BatteryVoltage = pPacket->BatteryVoltage;
	  m_Temperature = pPacket->Temperature_C;

	  m_GPS_Longitude = pPacket->GPS_Longitude;
	  m_GPS_Latitude = pPacket->GPS_Latitude;
	  m_GPS_Velocity = pPacket->GPS_Velocity;
	  m_GPS_HPE = pPacket->GPS_HPE;
	  m_GPS_Heading = pPacket->GPS_Heading;
	  m_GPS_Hours = pPacket->GPS_Hours;
	  m_GPS_Minutes = pPacket->GPS_Minutes;
	  m_GPS_Seconds = pPacket->GPS_Seconds;

	  // DB: x and y axes in MOOS body coordinates are reversed from x and y
	  // in the Rabbit AUV.
	  m_AccelPitch = static_cast<float>(
			  	  MOOS_ANGLE_WRAP(MOOSDeg2Rad(pPacket->Accelerometer_X)) );
	  m_AccelRoll = static_cast<float>(
			  	  MOOS_ANGLE_WRAP(MOOSDeg2Rad(pPacket->Accelerometer_Y)) );

	  m_CompassHeading = pPacket->CompassHeading;
	  m_CompassPitch = static_cast<float>( MOOSDeg2Rad(pPacket->Pitch) );
	  m_CompassRoll = static_cast<float>(
			  	  MOOS_ANGLE_WRAP(MOOSDeg2Rad(pPacket->Roll)) );

	  m_CompassDip = static_cast<float>(
			  	  MOOS_ANGLE_WRAP(MOOSDeg2Rad(pPacket->Dip)) );

	  m_SensorValuesAreFresh = true;

	  /*
	  MOOSTrace("Heading=%6.3f  Yaw=%6.3f  Pitch=%6.3f  Roll=%6.3f\n",
			  m_CompassHeading,
	  		  MOOS_ANGLE_WRAP( MOOSDeg2Rad(m_CompassHeading)),
	  		  m_CompassPitch,
	  		  m_CompassRoll);
	  */
}


//=============================================================================
void SpockModule::HandleDepthPacket( DepthPacket_t* pPacket )
{
   if (m_Verbosity >= 1)
   {
      std::cout << "Received a high-rate depth packet from SPOCK" << std::endl;
   }
}


