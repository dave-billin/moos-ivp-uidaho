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
/** @file BSDuCModule.h
 *
 * @brief
 *   Declaration of a class used to encapsulate the interface with the PSoC
 *   microcontroller on the BSD CPU board
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef BSDUCMODULE_H_
#define BSDUCMODULE_H_

#include <stdint.h>
#include <string>
#include <bitset>

#include "PrecisionTime.h"
#include "MOOS/libMOOS/Utils/MOOSSerialPort.h"
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "parapet.h"
#include "ParapetSerialDeviceNode.h"
#include "ParapetEventBroadcaster.h"
#include "ParapetListener.h"
#include "BSDuCStartupFSM.h"

//=============================================================================
/** @class BSDuCModule
 *
 * @brief
 *   A class encapsulating communications with the PSoC microcontroller on the
 *   MOOS AUV BSD module
 */
//=============================================================================
class BSDuCModule : public parapet::ParapetListener
{
public:

    /** The parapet Device ID used by iBSDuC */
    enum { IBSDUC_PARAPET_DEVICE_ID = 1 };

    //=========================================================================
    /** Creates an instance of the object */
    BSDuCModule( void );

    /** Destructor called when the object goes out of scope */
    virtual ~BSDuCModule();


    //=========================================================================
    /** Configures and opens the object's serial port connection
     *
     * @param [in] SerialPortDevice
     *   Serial port device to open (e.g. '/dev/ttyS0')
     *
     * @return
     *   true if the specified serial port was configured and opened; else
     *   false
     */
    bool Open( std::string const& SerialPortDevice );


    /** @return
     *   true if the Serial Port connected to the BSD microcontroller is open
     */
    bool IsOpen( void ) const { return m_ParapetSerialPort.IsOpen(); }


    //=========================================================================
    /** Closes the object's serial port connection if it is open */
    void Close( void );


    //=========================================================================
    /** Services the serial connection to the BSD microcontroller, sending and
     *  receiving data
     *
     * @return
     *   true on success; false on error
     */
    bool Process( void );



    /** ID's returned by get_state() */
    enum eBSDuCModuleStates
    {
        /** Indicates an error state */
        BSDUC_MODULE_ERROR = -1,

        /** Indicates the BSD microcontroller is disconnected */
        BSDUC_MODULE_DISCONNECTED,

        /** Indicates the BSD microcontroller startup sequence is running */
        BSDUC_MODULE_CONNECTING,

        /** Indicates the BSD microcontroller is connected and functioning */
        BSDUC_MODULE_ONLINE,

        NUM_BSDUC_MODULE_STATES /**< Internal use only - not a valid ID! */
    };

    //=========================================================================
    /** @return
     *   The state of the BSD microcontroller interface as a member of
     *   eBSDuCModuleStates
     */
    bool IsOnline( void ) const;


    //-------------------------------------------------------------------------
    /** @enum eBSDSensorChangeFlagIDs
     * @brief
     *   ID's of BSD sensor change flags
     */
    enum eBSDSensorChangeFlagIDs
    {
        CHANGEFLAG_BATTERY_VOLTS = 0,   /**< Battery voltage */
        CHANGEFLAG_BATTERY_AMPS,        /**< Battery current */
        CHANGEFLAG_BATTERY_DISCHARGED_AMPS,  /**< Discharged battery amps */
        CHANGEFLAG_BATTERY_DISCHARGED_WATTS, /**< Discharged battery watts */
        CHANGEFLAG_BATTERY_TEMP,        /**< Battery monitor temperature */
        CHANGEFLAG_DEPTH,               /**< Depth from pressure sensor */
        CHANGEFLAG_DEPTH_ALARM,         /**< Failure to measure depth */
        CHANGEFLAG_WATER_DETECTED,      /**< Water detection */
        CHANGEFLAG_PROPELLER_RPM,       /**< Measured propeller RPM */
        CHANGEFLAG_BATTERYMON_SERIAL,   /**< Battery monitor serial number */
        CHANGEFLAG_FIRMWARE_VERSION,    /**< BSD microcontroller firmware
                                             version */

        NUM_CHANGE_FLAGS    /**< Internal use only - not a valid flag ID */
    };
    //-------------------------------------------------------------------------


    //=========================================================================
    /** @return
     *   A reference to a bitset of sensor change flags whose bit indices are
     *   denoted by the members of eBSDSensorChangeFlagIDs
     */
    std::bitset<NUM_CHANGE_FLAGS> const& ChangeFlags( void ) const
    { return m_ChangeFlags; }



    //=========================================================================
    /** @return
     *   Instantaneous battery voltage measurement (Volts) reported by the
     *   BSD microcontroller */
    float BatteryVolts( void );

    //=========================================================================
    /** @return
     *   Instantaneous battery current measurement (Amps) reported by the BSD
     *   microcontroller */
    float BatteryAmps( void );

    //=========================================================================
    /** @return
     *   Time integral of measured battery discharge current (Amps) reported
     *   by the BSD microcontroller */
    float BatteryAmpsDischarged( void );

    //=========================================================================
    /** @return
     *   Measured temperature (Celsius) of the Battery Monitor IC on the power
     *   supply PCB as reported by the BSD microcontroller */
    float BatteryTempC( void );



    //=========================================================================
    /** @return
     *   Depth in meters based on the pressure sensor reading reported by the
     *   BSD microcontroller after depth zeroing has been applied */
    float Depth_meters( void );

    //=========================================================================
    /** @return
     *   true if the depth sensor voltage measurement reported by the BSD
     *   microcontroller indicates an invalid sensor reading */
    bool DepthAlarm( void );

    //=========================================================================
    /** @return
     *   The current depth offset (meters) applied to the depth sensor
     *   reading in order to 'zero' the depth */
    bool DepthZeroOffset( void ) const { return m_DepthZeroOffset; }



    //=========================================================================
    /** @return
     *   true if the BSD microcontroller reports that water is detected inside
     *   the vehicle's hull */
    bool WaterIsDetected();


    //=========================================================================
    /** @return
     *   The measured propeller RPM reported by the BSD microcontroller */
    float PropellerRpm( void );


    //=========================================================================
    /** @return
     *   A reference to a string containing the serial number of the battery
     *   monitor IC reported by the BSD microcontroller */
    std::string const& BatteryMonitorSerialNumber( void );


    //=========================================================================
    /** @return Firmware version reported by the BSD microcontroller */
    float FirmwareVersion( void );


    //=========================================================================
    /** Updates the desired throttle setting
     *
     * @details
     *   Positive thrust settings increase propeller speed, producing forward
     *   motion.  Negative thrust settings increase propeller speed in the
     *   opposite direction, producing aft-ward thrust.  A thrust value of
     *   zero disables drive applied to the propeller.
     *
     * @param [in] thrust
     *   Desired thrust as a percentage of full scale (0.0 to 1.0)
     */
    void SetThrust( float thrust );

    //=========================================================================
    /** @return
     * The current thrust setting as a percentage of full scale (0.0 to 1.0)
     */
    float GetThrust() const { return m_Thrust; }


    //=========================================================================
    /** Controls whether the propeller should be enabled (default) or disabled
     *  (typically for testing purposes)
     *
     * @remarks
     *   This function may be used to disable the propeller by always sending
     *   a propeller speed of zero.  This functionality is provided for testing
     *   situations when the propeller motor must be disabled in order to
     *   reduce heat or prolong battery life.
     *
     * @param PropellerShouldBeEnabled
     *   true if the propeller should be enabled; else false to disable it
     */
    void SetPropellerEnabled( bool PropellerShouldBeEnabled )
    { m_PropellerIsEnabled = PropellerShouldBeEnabled; }


    //=========================================================================
    /** @return true if the propeller is enabled; else false */
    bool PropellerIsEnabled( void ) const { return m_PropellerIsEnabled; }



    //=========================================================================
    /** Updates the desired rudder angle
     *
     * @details
     *   When the vehicle is in motion, positive rudder angles result in yaw
     *   to starboard, while negative angles result in a yaw to port.  A rudder
     *   setting of 0 degrees corresponds to the actuator's null position.
     *
     * @param [in] Angle_degrees
     *   Desired rudder angle in degrees (-85.0 to +85.0)
     */
    void SetRudderAngleDeg( float Angle_degrees );

    /** @return The current desired rudder angle in degrees from null */
    float GetRudderAngleDeg() const { return m_RudderAngle_deg; }



    //=========================================================================
    /** Updates the desired elevator angle
     *
     * @details
     *   When the vehicle is in motion, positive elevator angles result in
     *   negative changes in pitch (diving), while negative angles result in
     *   positive changes in pitch (climbing).  An elevator setting of 0
     *   degrees corresponds to the actuator's null position.
     *
     * @param [in] Angle_deg
     *   Desired elevator angle in degrees (-85.0 to +85.0)
     */
    void SetElevatorAngleDeg( float Angle_deg );

    /** @return The current elevator angle in degrees */
    float GetElevatorAngleDeg() const { return m_ElevatorAngle_deg; }



    //=========================================================================
    /** Updates the desired aileron angle
     *
     * @details
     *   When the vehicle is in motion, positive aileron angles result in
     *   positive roll (roll to starboard), while negative angles
     *   result in negative roll (roll to port).  An aileron setting of 0
     *   degrees corresponds to the null setting.
     *
     * @param [in] Angle_deg
     *   Desired aileron angle in degrees (-85.0 to +85.0)
     */
    void SetAileronAngleDeg( float Angle_deg );

    /** @return The current aileron angle in degrees */
    float GetAileronAngleDeg() const
    {
        return m_AileronAngle_deg;
    }


    //=========================================================================
    /** Updates the actuator coupling coefficient applied to actuator
     *  settings
     *
     * @details
     *   The actuator coupling coefficient is used to compensate for secondary
     *   effects resulting from actuator settings.  For example: adjustments to
     *   the AUV rudder fin (the vertically-oriented control plane on the tail
     *   of the AUV) that angle it away from its null setting primarily affect
     *   vehicle yaw (heading).  However, since the adjustment increases drag
     *   above the vehicle's center of gravity, it also causes the vehicle to
     *   pitch upward slightly.
     *
     *   To compensate for this, the AUV elevator fins (the bottom two control
     *   planes on the tail of the AUV) are adjusted to produce a small
     *   negative pitch offset, counteracting the secondary effect of the
     *   rudder plane.  The actuator coupling coefficient determines the amount
     *   of adjustment applied to counteract secondary effects.  Small coupling
     *   coefficient values result in very small compensatory adjustments,
     *   while larger values produce more compensatory adjustment.  A coupling
     *   coefficient of zero disables compensatory adjustment altogether, while
     *   a coefficient value of 1 results in directly proportional adjustment.
     *
     * @param [in] Coupling
     *   Desired coupling coefficient between 0.0 (no coupling) and 1.0
     *   (full coupling)
     */
    void SetActuatorCouplingCoefficient(float Coupling);

    /** @return The value of the actuator coupling coefficient */
    float GetActuatorCouplingCoefficient() const
    { return m_ActuatorCouplingCoefficient; }





    //=========================================================================
    /** Requests that a battery monitor reset command be sent to the BSD
     *  microcontroller during the next call to Process()
     */
    void RequestBatteryMonitorReset( void )
    { m_ShouldSendBatteryMonitorReset = true; }


    //=========================================================================
    /** Requests that a firmware reset command be sent to the BSD
     *  microcontroller during the next call to Process()
     */
    void RequestFirmwareReset( void )
    { m_ShouldSendFirmwareReset = true; }


    //=========================================================================
    /** @return
     *   A reference to a PrecisionTime object indicating the time when the
     *   last message from the BSD microcontroller was received and processed
     */
    YellowSubUtils::PrecisionTime const& LastRxTime() const
    {
        return m_LastRxTime;
    }



    //================================
    // ParapetListener API methods
    //================================

    //=========================================================================
    /** This function will be called when a parapet packet has been received
     *  on the serial port connected to the BSD microcontroller
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object  that received the
     *  packet.
     *
     * @param [in] Opcode
     *  The value of the OPCODE field in the received packet's header
     *
     * @param [in] Packet
     *  A reference to the packet that was received.
     */
    virtual void OnPacketReceived(
                                parapet::ParapetEventBroadcaster const& Source,
                                parapet::parapet_opcode_t Opcode,
                                uint8_t SequenceID,
                                parapet::parapet_packet_t const& Packet );


    //=========================================================================
    /** This function will be called when a notable event occurs when receiving
     *  parapet packets from the BSD microcontroller
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object signaling the event
     *
     * @param [in] EventID
     *  ID of the event
     */
    virtual void OnPacketEvent( parapet::ParapetEventBroadcaster const& Source,
                                int EventID );


    /** Period in milliseconds at which sensor values are requested from the
     * BSD microcontroller */
    enum { BSD_SENSOR_POLL_PERIOD_MS = 100 };


    /** Period in milliseconds at which battery monitor values are requested
     * BSD microcontroller */
    enum { BSD_BATTERY_MONITOR_POLL_PERIOD_MS = 1000 };


    enum { BSD_UC_COMMS_TIMEOUT_MS };   /**< Maximum milliseconds that may
                                             elapse without receiving data from
                                             the BSD microcontroller before the
                                             communications session is flagged
                                             as offline */

    enum { BSD_UC_PARAPET_DEVICE_ID = 1 };  /**< parapet device ID of the BSD
                                                 microcontroller */

    /** Serial port baud rate used for BSD microcontroller communications */
    static const uint32_t BSD_MICROCONTROLLER_SERIAL_BAUD_RATE;



private:
    /** Parapet master node used to communicate with the BSD microcontroller */
    ParapetSerialDeviceNode m_ParapetSerialPort;

    /** Object used to implement BSD microcontroller session startup */
    BSDuC_StartupFSM m_StartupFSM;


    /** Time when data was last received from the BSD microcontroller */
    YellowSubUtils::PrecisionTime m_LastRxTime;

    /** Timestamp of the last successful request for sensors */
    YellowSubUtils::PrecisionTime m_LastSensorPollTime;

    /** Timestamp of the last successful request for battery monitor data*/
    YellowSubUtils::PrecisionTime m_LastBatteryMonitorPollTime;


    /** Firmware version reported by the BSD microcontroller */
    float m_FirmwareVersion;

    /** Battery monitor serial number reported by the BSD microcontroller */
    std::string m_BatteryMonitorSerial;

    float m_BatteryVolts;     /**< Instantaneous battery voltage in volts */
    float m_BatteryAmps;      /**< Instantaneous battery current in amps */
    float m_BatteryTemp_C;    /**< Temperature (Celsius) at battery monitor*/

    /** Time integral of battery discharge current measurements (Amps)*/
    float m_BatteryAmpsDischarged;


    float m_Depth_meters;   /**< Measured depth in meters */
    bool m_DepthAlarm;      /**< true if pressure sensor reading is invalid */
    float m_DepthZeroOffset;  /**< Offset (meters) applied to depth sensor
                                   reading in order to 'zero' depth */

    bool m_WaterIsDetected; /**< true if water is detected inside the hull;
                                 else false */

    bool m_PropellerIsEnabled;  /**< true if propeller drive is enabled */

    /** Measured motor RPM reported by the BSD microcontroller */
    uint32_t m_MeasuredPropellerRPM;

    /** Flags used to indicate sensor values that have changed where bit
     *   positions correlate to entries in eBSDSensorChangeFlagIDs */
    std::bitset<NUM_CHANGE_FLAGS> m_ChangeFlags;


    //----------------------------------
    // Propeller and Actuator Settings
    //----------------------------------

    /** Amount of drive applied to the prop motor as a percentage of
     *  full scale (0.0 to 1.0) where positive values indicate thrust
     *  in the direction of the AUV bow, and negative values indicate
     *  stern-ward (reverse) thrust */
    float m_Thrust;

    float m_RudderAngle_deg;               /**< Rudder angle in degrees */
    float m_ElevatorAngle_deg;             /**< Elevator angle in degrees */
    float m_AileronAngle_deg;              /**< Aileron angle in degrees */
    float m_ActuatorCouplingCoefficient;   /**< Actuator coupling coefficient*/


    /** Trimmed rudder null position in degrees */
    float m_RudderNullDegrees;

    /** Trimmed left elevator plane null position in degrees */
    float m_LeftElevatorNullDegrees;

    /** Trimmed right elevator plane null position in degrees */
    float m_RightElevatorNullDegrees;

    /** true if propeller and actuator settings should be sent during
     *  the next call to Process() */
    bool m_ShouldSendControls;

    /** true if a battery monitor reset command should be sent during
     *  the next call to Process() */
    bool m_ShouldSendBatteryMonitorReset;

    /**< true if a firmware reset command should be sent during the next
     *   call to Process() */
    bool m_ShouldSendFirmwareReset;


    //=========================================================================
    /** Processes the serial port connection with the BSD microcontroller when
     *  a communication session is online */
    void ProcessOnlineSession( void );


    //=========================================================================
    /** Resets local values when a session disconnects */
    void ProcessSessionDisconnect( void );


    //=========================================================================
    /** Sends a request packet with a specified opcode, descriptor ID, and
     *  optional payload to the BSD microcontroller
     *
     * #param [in] Opcode
     *   Opcode of the request to transmit
     *
     * @param [in] ID
     *   ID of the function or parameter to request - this will be placed in
     *   the ID bits of the packet's descriptor word
     *
     * @param [in] PayloadData
     *   Pointer to payload data to be added to the request
     *
     * @param [in] NumPayloadBytes
     *   Size of the payload data in Bytes
     *
     * @return
     *   The sequence ID of the transmitted request on success; else -1 on
     *   failure to transmit the request
     */
    int8_t TransmitRequest( parapet_opcode_t Opcode, uint16_t ID,
                            void* PayloadData = NULL,
                            uint16_t NumPayloadBytes = 0 );



    //----------------------------------------------------------------
    // Copy constructor, and assignment operator are disallowed
    BSDuCModule( BSDuCModule const& );
    BSDuCModule& operator=( BSDuCModule const& );
    //----------------------------------------------------------------
};

//=============================================================================
inline bool BSDuCModule::IsOnline( void ) const
{
    return m_ParapetSerialPort.IsOpen() && m_StartupFSM.IsOnline();
}

//=============================================================================
inline float BSDuCModule::BatteryVolts( void )
{
    m_ChangeFlags[ CHANGEFLAG_BATTERY_VOLTS ] = false;
    return m_BatteryVolts;
}

//=============================================================================
inline float BSDuCModule::BatteryAmps( void )
{
    m_ChangeFlags[ CHANGEFLAG_BATTERY_AMPS ];
    return m_BatteryAmps;
}

//=============================================================================
inline float BSDuCModule::BatteryAmpsDischarged( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_BATTERY_DISCHARGED_AMPS );
    return m_BatteryAmpsDischarged;
}

//=============================================================================
inline float BSDuCModule::BatteryTempC( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_BATTERY_TEMP );
    return m_BatteryTemp_C;
}

//=============================================================================
inline bool BSDuCModule::DepthAlarm( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_DEPTH_ALARM );
    return m_DepthAlarm;
}

//=============================================================================
inline bool BSDuCModule::WaterIsDetected()
{
    m_ChangeFlags.reset( CHANGEFLAG_WATER_DETECTED );
    return m_WaterIsDetected;
}

//=============================================================================
inline float BSDuCModule::PropellerRpm( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_PROPELLER_RPM );
    return m_MeasuredPropellerRPM;
}

//=============================================================================
inline std::string const& BSDuCModule::BatteryMonitorSerialNumber( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_BATTERYMON_SERIAL );
    return m_BatteryMonitorSerial;
}

//=============================================================================
inline float BSDuCModule::FirmwareVersion( void )
{
    m_ChangeFlags.reset( CHANGEFLAG_FIRMWARE_VERSION );
    return m_FirmwareVersion;
}

//=============================================================================
inline void BSDuCModule::SetThrust(float thrust)
{
    m_Thrust = thrust;
    m_ShouldSendControls = true;
}

//=============================================================================
inline void BSDuCModule::SetRudderAngleDeg( float Angle_degrees )
{
    m_RudderAngle_deg = Angle_degrees;
    m_ShouldSendControls = true;
}

//=============================================================================
inline void BSDuCModule::SetElevatorAngleDeg( float Angle_deg )
{
    m_ElevatorAngle_deg = Angle_deg;
    m_ShouldSendControls = true;
}

//=============================================================================
inline void BSDuCModule::SetAileronAngleDeg( float Angle_deg )
{
    m_AileronAngle_deg = Angle_deg;
    m_ShouldSendControls = true;
}

//=============================================================================
inline void BSDuCModule::SetActuatorCouplingCoefficient(float Coupling)
{
    m_ActuatorCouplingCoefficient = Coupling;
    m_ShouldSendControls = true;
}

#endif /* BSDUCMODULE_H_ */

