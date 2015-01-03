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
/** @file BSDuCModule.cpp
 *
 * @brief
 *   Implementation of the BSDuCModule class
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <cstring>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "parapet.h"
#include "BSDuC_parapet_packets.h"
#include "BSDuCModule.h"

using namespace std;
using namespace parapet;
using namespace BSDuC_parapet_packets;
using namespace YellowSubUtils;



//=============================================================================
const uint32_t BSDuCModule::BSD_MICROCONTROLLER_SERIAL_BAUD_RATE = 115200;


//=============================================================================
BSDuCModule::BSDuCModule( void )
  : m_ParapetSerialPort( IBSDUC_PARAPET_DEVICE_ID ),
    m_StartupFSM( m_ParapetSerialPort ),
    m_FirmwareVersion( 0.0f ),
    m_BatteryVolts( 0.0f ),
    m_BatteryAmps( 0.0f ),
    m_BatteryTemp_C( 0.0f ),
    m_BatteryAmpsDischarged( 0.0f ),
    m_Depth_meters( 0.0f ),
    m_DepthAlarm( false ),
    m_DepthZeroOffset( 0.0f ),
    m_WaterIsDetected( false ),
    m_PropellerIsEnabled( true ),
    m_MeasuredPropellerRPM( 0.0f ),
    m_Thrust( 0.0f ),
    m_RudderAngle_deg( 0.0f ),
    m_ElevatorAngle_deg( 0.0f ),
    m_AileronAngle_deg( 0.0f ),
    m_ActuatorCouplingCoefficient( 0.0f ),
    m_RudderNullDegrees( 0.0 ),
    m_LeftElevatorNullDegrees( 0.0 ),
    m_RightElevatorNullDegrees( 0.0 ),
    m_ShouldSendControls( false ),
    m_ShouldSendBatteryMonitorReset( false ),
    m_ShouldSendFirmwareReset( false )
{
    // Register this object to receive parapet events and packets
    m_ParapetSerialPort.AddListener( this );

    // Reset change flags
    m_ChangeFlags.reset();

}


//=============================================================================
BSDuCModule::~BSDuCModule()
{
    // Un-register this object from the parapet port
    m_ParapetSerialPort.RemoveListener( this );
    m_ParapetSerialPort.Close();
}



//=============================================================================
bool BSDuCModule::Open( std::string const& SerialPortDevice )
{
    bool SerialPortIsOpen = m_ParapetSerialPort.Open( SerialPortDevice,
                                        BSD_MICROCONTROLLER_SERIAL_BAUD_RATE );

    if ( SerialPortIsOpen )
    {
        MOOSTrace("Opened the serial connection to the BSD microcontroller\n");
        m_StartupFSM.Reset();       // Reset the startup FSM
    }

    return SerialPortIsOpen;
}


//=============================================================================
void BSDuCModule::Close( void )
{
    if ( m_ParapetSerialPort.IsOpen() )
    {
        // If a session is active, send an END_SESSION notification
        // prior to closing the serial connection.
        if ( m_StartupFSM.IsOnline() )
        {
            TransmitRequest( parapet::OPCODE_EXECUTE,
                             BSDuC_parapet_packets::END_SESSION );

            m_StartupFSM.Reset();
        }

        m_ParapetSerialPort.Close();

        MOOSTrace("Closed the serial connection to the BSD microcontroller\n");
    }

}



//=============================================================================
bool BSDuCModule::Process( void )
{
    bool RetVal = false;

    if ( m_ParapetSerialPort.IsOpen() )
    {
        // Receive and process data packets from the BSD microcontroller
        // This results in calls to OnPacketReceived() for each packet
        // received.
        m_ParapetSerialPort.Receive();

        if ( false == m_StartupFSM.IsOnline() )
        {
            // Run the startup FSM to implement session startup
            bool IsOnline = m_StartupFSM.Process();

            if ( IsOnline )
            {
                MOOSTrace( "<BSD Microcontroller communications online>\n" );
            }
        }
        else
        {
            // Process polling and messages for an active session
            ProcessOnlineSession();
        }

        RetVal = true;
    }

    return RetVal;
}



//=============================================================================
float BSDuCModule::Depth_meters( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_DEPTH );

    return ( m_DepthAlarm == false ) ?
            (m_Depth_meters - m_DepthZeroOffset) : -100.0;
}



//=============================================================================
void BSDuCModule::OnPacketReceived( ParapetEventBroadcaster const& Source,
                                    parapet_opcode_t Opcode,
                                    uint8_t SequenceID,
                                    parapet_packet_t const& Packet )
{

}


//=============================================================================
void BSDuCModule::OnPacketEvent( ParapetEventBroadcaster const& Source,
                                 int EventID )
{

}


//=============================================================================
void BSDuCModule::ProcessOnlineSession(void)
{
    //----------------------------------
    // Implement sensor poll period
    //----------------------------------
    PrecisionTime t = PrecisionTime::Now();
    PrecisionTimeInterval Elapsed_time = ( t - m_LastSensorPollTime );
    uint32_t Elapsed_ms =
            Elapsed_time.As( PrecisionTimeInterval::MILLISECONDS );

    if ( Elapsed_ms > BSD_SENSOR_POLL_PERIOD_MS )
    {
        int8_t SequenceID = TransmitRequest( parapet::OPCODE_READ_PARAM,
                                             BSDuC_parapet_packets::SENSORS );
        if ( SequenceID >= 0 )
        {
            m_LastSensorPollTime = t;
        }
    }

    //----------------------------------------
    // Implement battery monitor poll period
    //----------------------------------------
    Elapsed_time = ( t - m_LastBatteryMonitorPollTime );
    Elapsed_ms = Elapsed_time.As( PrecisionTimeInterval::MILLISECONDS);
    if ( Elapsed_ms > BSD_BATTERY_MONITOR_POLL_PERIOD_MS )
    {
        int8_t SequenceID = TransmitRequest(
                                      parapet::OPCODE_READ_PARAM,
                                      BSDuC_parapet_packets::BATTERY_MONITOR );
        if ( SequenceID >= 0 )
        {
            m_LastBatteryMonitorPollTime = t;
        }
    }


    //----------------------------------------
    // Service actuator values
    //----------------------------------------



    //----------------------------------------
    // Service firmware reset requests
    //----------------------------------------
    if ( m_ShouldSendFirmwareReset == true )
    {
        // Set up 16-Byte safety key for reset firmware command
        BSDuC_SoftReset_Request ResetKey =
                {{ 0x55aa55aa, 0x81818181, 0xaa55aa55, 0x18181818 }};

        int8_t SequenceID = TransmitRequest( parapet::OPCODE_EXECUTE,
                                             BSDuC_parapet_packets::SOFT_RESET,
                                             &ResetKey,
                                             sizeof(BSDuC_SoftReset_Request) );

        if ( SequenceID >= 0 )
        {
            m_ShouldSendFirmwareReset = false;
        }
    }

    //----------------------------------------
    // Service battery monitor reset request
    //----------------------------------------
    else if ( m_ShouldSendBatteryMonitorReset == true )
    {
        int8_t SequenceID =TransmitRequest(
                             parapet::OPCODE_EXECUTE,
                             BSDuC_parapet_packets::RESET_BATTERY_ACCOUNTING );

        if ( SequenceID >= 0 )
        {
            m_ShouldSendBatteryMonitorReset = false;
        }
    }
}


//=============================================================================
void BSDuCModule::ProcessSessionDisconnect( void )
{
    m_StartupFSM.Reset();   // Reset the startup FSM

    m_LastRxTime.Zero();
    m_LastSensorPollTime.Zero();
    m_LastBatteryMonitorPollTime.Zero();

    m_FirmwareVersion = -1.0f;

    m_BatteryMonitorSerial.clear();
    m_BatteryVolts = 0.0f;
    m_BatteryAmps = 0.0f;
    m_BatteryTemp_C = 0.0f;
    m_BatteryAmpsDischarged = 0.0f;

    m_Depth_meters = 0.0f;
    m_DepthAlarm = false;
    m_WaterIsDetected = false;
    m_MeasuredPropellerRPM = 0;

    m_ChangeFlags.reset();      // Reset sensor change flags
}


//=============================================================================
int8_t BSDuCModule::TransmitRequest( parapet_opcode_t Opcode, uint16_t ID,
                                     void* PayloadData,
                                     uint16_t NumPayloadBytes )
{
    parapet_packet_t Packet;

    parapet_BuildRequest( &Packet.Request, static_cast<uint8_t>(Opcode),
                          0, // Source device is automatically populated
                          BSDuC_parapet_packets::BSDuC_PARAPET_DEVICE_ID,
                          0, // Sequence ID is automatically populated
                          ID );

    if ( NULL != PayloadData )
    {
        parapet_AddPayloadData( &Packet, PayloadData, NumPayloadBytes );
    }

    return m_ParapetSerialPort.Transmit( Packet );
}

