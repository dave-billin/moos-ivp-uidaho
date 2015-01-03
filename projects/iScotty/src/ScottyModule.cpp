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
/** @file ScottyModule.cpp
 * @brief
 *	Implementation of the ScottyModule class
 *
 * @author Dave Billin
 */
//=============================================================================

#include "ScottyModule.h"


#define LOCAL_DEVICEID		2		// Send packets as KIRK
#define SCOTTY_DEVICEID		4		// BunnySock ID of the SCOTTY module
#define SCOTTY_MAX_PROP_RPM	3000	// Maximum propeller RPM to send to Scotty

#define DECIDEG2SERVO 0.2822	// Conversion from decidegrees to servo values

#define SCOTTYVERBOSE1(_expr_)	if (m_Verbosity >= 1) { _expr_; }
#define SCOTTYVERBOSE2(_expr_)	if (m_Verbosity >= 2) { _expr_; }
#define SCOTTYVERBOSE3(_expr_)	if (m_Verbosity >= 3) { _expr_; }



//=============================================================================
ScottyModule::ScottyModule(string& sHostName, uint16_t Port, int Verbosity)
: m_pNode(NULL),
  m_Verbosity(Verbosity),
  m_RudderAngle_rad(0.0f),
  m_ElevatorAngle_rad(0.0f),
  m_AileronAngle_rad(0.0f),
  m_DesiredPropRPM(0),
  m_MeasuredPropRPM(0),
  m_DesiredServoCentersSet(false),
  m_RudderServoCenter(0),
  m_ElevLServoCenter(0),
  m_ElevRServoCenter(0),
  m_CouplingCoefficient(0.0f)
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
		string s = "ScottyModule: Failed to create BunnySock TCP client node "
				   "to connect to " + sHostName +
				   MOOSFormat(":%d",Port) + "\n";
		throw CMOOSException(s);
	}

	m_pNode = pTcpNode;
	m_pNode->AddListener(this);	// Register for BunnySock packets and events
	m_pNode->Start();	// Start the BunnySock connection's network thread
}




//=============================================================================
ScottyModule::~ScottyModule()
{
	if (m_pNode != NULL)
	{
		delete m_pNode;
	}
}




//=============================================================================
void ScottyModule::SetActuators( float Rudder_rad, float Elevator_rad,
					   	    	 float Aileron_rad, float PropThrust )
{
	if (!m_pNode->IsConnected())
	{
		return;
	}

	// Register new settings
	m_RudderAngle_rad = Rudder_rad;
	m_ElevatorAngle_rad = Elevator_rad;
	m_AileronAngle_rad = Aileron_rad;
	m_DesiredPropRPM = static_cast<int16_t>( SCOTTY_MAX_PROP_RPM * PropThrust *
											 0.01);

	float Rudder_ddeg = MOOSRad2Deg(m_RudderAngle_rad) * 10.0;
	float Elevator_ddeg = MOOSRad2Deg(m_ElevatorAngle_rad) * 10.0;
	float Aileron_ddeg = MOOSRad2Deg(m_AileronAngle_rad) * 10.0;
	uint16_t PropRpm = m_DesiredPropRPM;

	BunnySockPacket ControlsPacket;
	ControlsPacket_t* pPacket =
				static_cast<ControlsPacket_t*>(ControlsPacket.GetRawBytes());

	ControlsPacket.SetHeader(TYPE_CONTROLS,			/* Packet type ID */
							 LOCAL_DEVICEID,		/* Source device ID */
							 SCOTTY_DEVICEID,		/* Dest device ID */
							 0);					/* ms time stamp */

	pPacket->RudderAngle_ddeg = static_cast<int16_t>(Rudder_ddeg);
	pPacket->Elevator_ddeg = static_cast<int16_t>(Elevator_ddeg);
	pPacket->Aileron_ddeg = static_cast<int16_t>(Aileron_ddeg);
	pPacket->DesiredPropRpm = PropRpm;

	// Send the controls packet.  This does nothing if SCOTTY is not connected
	m_pNode->SendPacket(ControlsPacket);

	SCOTTYVERBOSE2( MOOSTrace("Sent actuators to SCOTTY:\n"
							  "  Rudder=%5.2f deg  Elevator=%5.2f deg\n"
							  "  Aileron=%5.2f deg Prop=%d RPM (%5.2f%% Thrust)\n",
							  MOOSRad2Deg(m_RudderAngle_rad),
							  MOOSRad2Deg(m_ElevatorAngle_rad),
							  MOOSRad2Deg(m_AileronAngle_rad),
							  pPacket->DesiredPropRpm,
							  PropThrust) );

}




//=============================================================================
void ScottyModule::SetServoTrimAngles( float RudderOffset_rad,
								  	   float ElevatorOffsetL_rad,
								  	   float ElevatorOffsetR_rad,
								  	   float CouplingCoeff )
{
	float RudderOffset_ddeg;	// Rudder offset in decidegrees
	float ElevatorOffsetL_ddeg;	// Left elevator offset in decidegrees
	float ElevatorOffsetR_ddeg;	// Right elevator offset in decidegrees

	// Calculate offsets in decidegrees
	RudderOffset_ddeg = (float)(MOOSRad2Deg(RudderOffset_rad)) * 10.0f;
	ElevatorOffsetL_ddeg = (float)(MOOSRad2Deg(ElevatorOffsetL_rad)) * 10.0f;
	ElevatorOffsetR_ddeg = (float)(MOOSRad2Deg(ElevatorOffsetR_rad)) * 10.0f;

	// Servo settings range from 1 to 255
	uint8_t RudderCenter = (254 / 2);
	uint8_t ElevLCenter  = 254 / 2;
	uint8_t ElevRCenter  = 254 / 2;

	RudderCenter += (uint8_t)(RudderOffset_ddeg * DECIDEG2SERVO);
	ElevLCenter += (uint8_t)(ElevatorOffsetL_ddeg * DECIDEG2SERVO);
	ElevRCenter += (uint8_t)(ElevatorOffsetR_ddeg * DECIDEG2SERVO);

	SetServoCenters(RudderCenter, ElevLCenter, ElevRCenter, CouplingCoeff);
}





//=============================================================================
void ScottyModule::SetServoCenters( uint8_t Rudder, uint8_t ElevatorL,
									uint8_t ElevatorR, float CouplingCoeff )
{
	bool PacketWasSent;

	m_DesiredServoCentersSet = true;

	m_RudderServoCenter = Rudder;
	m_ElevLServoCenter = ElevatorL;
	m_ElevRServoCenter = ElevatorR;
	m_CouplingCoefficient = CouplingCoeff;

	//----------------------------------------------------------
	// If the SCOTTY module is connected, request the current
	// servo trims.  When the SCOTTY trim settings are returned,
	// they will be compared with the new (desired) settings.
	// If different, the new settings will be applied to the
	// SCOTTY module.  This helps to minimize wear on the Rabbit
	// user block FLASH memory
	if (m_pNode->IsConnected())
	{
		BunnySockPacket TrimPacket;
		CommandPacket_t* pPacket =
					static_cast<CommandPacket_t*>(TrimPacket.GetRawBytes());

		TrimPacket.SetHeader(TYPE_COMMAND, 		/* Packet type */
							 LOCAL_DEVICEID,	/* Source device */
							 SCOTTY_DEVICEID,	/* Dest device */
							 0 );				/* Time stamp */

		pPacket->CommandId = CMD_REQ_TRIM;

		PacketWasSent = m_pNode->SendPacket(TrimPacket);

		if (m_Verbosity > 0)
		{
			string s = (PacketWasSent) ?
						"Requesting servo trims from SCOTTY\n" :
						"Failed to request SCOTTY servo trims!\n";
			MOOSTrace(s);
		}
	}

}




//=============================================================================
bool ScottyModule::GetServoCenters( uint8_t& Rudder, uint8_t& ElevatorL,
						  	  	    uint8_t& ElevatorR )
{
	if ( m_pNode->IsConnected() )
	{
		Rudder = m_RudderServoCenter;
		ElevatorL = m_ElevLServoCenter;
		ElevatorR = m_ElevRServoCenter;
		return true;
	}
	else
	{
		return false;
	}
}



//=============================================================================
bool ScottyModule::GetServoCouplingCoefficient( float& CouplingCoeff )
{
	if ( m_pNode->IsConnected() )
	{
		CouplingCoeff = m_CouplingCoefficient;
		return true;
	}
	else
	{
		return false;
	}
}



//=============================================================================
bool ScottyModule::IsConnected( void ) const
{
	return m_pNode->IsConnected();
}



//=============================================================================
float ScottyModule::RudderAngle() const
{
	return (m_pNode->IsConnected()) ? m_RudderAngle_rad : 0.0;
}


//=============================================================================
float ScottyModule::ElevatorAngle() const
{
	return (m_pNode->IsConnected()) ? m_ElevatorAngle_rad : 0.0;
}


//=============================================================================
float ScottyModule::AileronAngle() const
{
	return (m_pNode->IsConnected()) ? m_AileronAngle_rad : 0.0;
}


//=============================================================================
float ScottyModule::Thrust() const
{
	if ( m_pNode->IsConnected() )
	{
		return (float)m_DesiredPropRPM / SCOTTY_MAX_PROP_RPM;
	}
	else
	{
		return 0.0;
	}
}



//=============================================================================
uint16_t ScottyModule::PropellerRpm() const
{
	return (m_pNode->IsConnected()) ? m_MeasuredPropRPM : 0;
}




//=============================================================================
void ScottyModule::OnPacketReceived( BunnySockPacket& RxPacket,
									 BunnySockNode& Node,
									 double TimeStamp_sec )
{
	CommandPacket_t* pPacket;
	pPacket = static_cast<CommandPacket_t*>(RxPacket.GetRawBytes());

	// We only need concern ourselves with COMMAND packets from SCOTTY
	if (pPacket->Header.Type != TYPE_COMMAND)
	{
		return;
	}

	// Handle RPM and actuator trim reports
	switch (pPacket->CommandId)
	{
		case CMD_REPORT_RPM:
		{
			m_MeasuredPropRPM = pPacket->Parameter[0] * m_DesiredPropRPM;
			SCOTTYVERBOSE3( MOOSTrace( "SCOTTY reports %d%% of prop target "
									   "(%d RPM)\n", pPacket->Parameter[0],
									   m_DesiredPropRPM ) );
			break;
		}

		case CMD_SET_TRIM:
		{
			HandleServoTrimReport(pPacket);
			break;
		}

		default:
			return;
	}
}




//=============================================================================
void ScottyModule::OnConnectionEvent( int EventId, BunnySockNode& Node,
									  double TimeStamp_sec )
{
	string s;

	switch (EventId)
	{
		case BunnySockListener::CONNECTED:
			s = "Connected to SCOTTY\n";
			break;

		// If SCOTTY disconnects, reset all reported variable values
		case BunnySockListener::CONNECTION_TIMEOUT:
			s = "Connection to SCOTTY timed out!\n";
			break;

		case BunnySockListener::CONNECTION_ERROR:
			s = "An error occurred on the connection to SCOTTY!\n";
			break;

		case BunnySockListener::DISCONNECTED:
			s = "Connection to SCOTTY closed.\n";
			break;

		default:
			return;
	}

	MOOSTrace(s);	// Print the event

	if (EventId == BunnySockListener::CONNECTED)
	{
		//----------------------------------------
		// STATUS:
		//	A connection with SCOTTY just opened.
		//	Request actuator trims
		//----------------------------------------
		BunnySockPacket TrimReqPacket;
		CommandPacket_t* pPacket =
					static_cast<CommandPacket_t*>(TrimReqPacket.GetRawBytes());

		TrimReqPacket.SetHeader(TYPE_COMMAND, 			/* Packet type */
								LOCAL_DEVICEID, 		/* Source device */
								SCOTTY_DEVICEID,		/* Dest device */
								0 );					/* ms time stamp */

		pPacket->CommandId = CMD_REQ_TRIM;
		m_pNode->SendPacket(TrimReqPacket);
	}
	else
	{
		//------------------------------------------
		// STATUS:
		//	The connection with SCOTTY has closed.
		//	Reset registered actuator variables.
		//	Preserve servo center values, since
		//	they get checked when SCOTTY connects
		m_MeasuredPropRPM = 0;
		m_RudderAngle_rad = 0.0;
		m_ElevatorAngle_rad = 0.0;
		m_AileronAngle_rad = 0.0;
	}
}




//=============================================================================
void ScottyModule::HandleServoTrimReport(CommandPacket_t* pPacket)
{
	if (pPacket != NULL)
	{
		// If desired servo values have been set, compare them with what
		// SCOTTY has sent us.  If they don't match, send the new values.
		if (m_DesiredServoCentersSet)
		{
			if ( (m_RudderServoCenter != pPacket->Parameter[0]) ||
				 (m_ElevRServoCenter != pPacket->Parameter[1]) ||
				 (m_ElevLServoCenter != pPacket->Parameter[2]) ||
				 (m_CouplingCoefficient != pPacket->Parameter[3]) )
			{
				BunnySockPacket TrimPacket;
				CommandPacket_t* pPacket =
						static_cast<CommandPacket_t*>(TrimPacket.GetRawBytes());

				TrimPacket.SetHeader(TYPE_COMMAND, 			/* Packet type */
									 LOCAL_DEVICEID,		/* Source device */
									 SCOTTY_DEVICEID,		/* Dest device */
									 0 );					/* Time stamp */

				pPacket->CommandId = CMD_SET_TRIM;
				pPacket->Parameter[0] = m_RudderServoCenter;
				pPacket->Parameter[1] = m_ElevRServoCenter;
				pPacket->Parameter[2] = m_ElevLServoCenter;
				pPacket->Parameter[3] = (uint16_t)(m_CouplingCoefficient * 100);

				m_pNode->SendPacket(TrimPacket);

				SCOTTYVERBOSE1( MOOSTrace("Sent servo center values to SCOTTY: "
						  	    "  Rudder=%d  LElev=%d  RElev=%d  CC=%f\n",
						  	    m_RudderServoCenter, m_ElevLServoCenter,
						  	    m_ElevRServoCenter, m_CouplingCoefficient) );
				return;
			}
		}
		else
		{
			m_RudderServoCenter = pPacket->Parameter[0];
			m_ElevRServoCenter = pPacket->Parameter[1];
			m_ElevLServoCenter = pPacket->Parameter[2];
			m_CouplingCoefficient = (float)pPacket->Parameter[3] / 100.0f;
		}
	}

	SCOTTYVERBOSE1( MOOSTrace("Scotty reports servo center values: "
			  	    "  Rudder=%d  LElev=%d  RElev=%d  CC=%f\n",
			  	    m_RudderServoCenter, m_ElevLServoCenter,
			  	    m_ElevRServoCenter, m_CouplingCoefficient) );
}

