//=============================================================================
/*  Copyright (C) 2013  Dave Billin

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
/** @file BSD_parapet_packets.h
 *
 * @brief
 *   Data structures and associated functions for parapet packets exchanged
 *   with the BSD microcontroller
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef BSD_PARAPET_PACKETS_H_
#define BSD_PARAPET_PACKETS_H_

#include <stdint.h>

#ifdef __cplusplus
namespace BSDuC_parapet_packets
{
#endif


/** parapet device ID of the BSD microcontroller */
enum { BSDuC_PARAPET_DEVICE_ID = 2 };


//-----------------------------------------------------------------------------
/** Values that may be specified for the ID bits of the descriptor word of
 *  a parapet EXECUTE packet sent to the BSD microcontroller */
//-----------------------------------------------------------------------------
enum eBSDuC_Execute_IDs
{
    /** No operation - may be used to signal or test connectivity;
     *  BSD microcontroller will reply by sending an EXECUTE
     *  response signaling ACK */
    NOP = 0,

    /** Informs the BSD microcontroller that a parapet peer is connecting and
     *  starting a communication session
     */
    START_SESSION,

    /** Informs the BSD microcontroller that a parapet peer is disconnecting
     *  and ending its communication session
     */
    END_SESSION,

    /** Instructs the BSD microcontroller to reset accumulated battery
     *  discharge current and time to zero */
    RESET_BATTERY_ACCOUNTING,

    /** Requests that the BSD microcontroller perform a soft reset and
     *  indicates that the packet payload contains a BSDuC_SoftReset_Request
     *  with the required key
     */
    SOFT_RESET,

    NUM_BSDUC_EXECUTE_IDS   /**< Internal use only - not a valid ID! */
};



//-----------------------------------------------------------------------------
/** Values that may be specified for the ID bits of the descriptor word of
 *  a parapet READ_PARAM packet sent to the BSD microcontroller */
//-----------------------------------------------------------------------------
enum eBSDuC_ReadParam_IDs
{
    /** Requests that the BSD microcontroller send a READ_PARAM reply
     *  whose payload is a single 32-bit float indicating the version
     *  of microcontroller firmware being executed */
    FIRMWARE_VERSION = 0,


    /**< Requests that the BSD microcontroller send a READ_PARAM
     *   reply whose payload is a BSDuC_Sensor_Report */
    SENSORS = 1,


    /**< Requests that the BSD microcontroller send a READ_PARAM
     *   reply whose payload is a BSDuC_BatteryMonitor_Report */
    BATTERY_MONITOR = 2,


    NUM_BSDUC_READ_PARAM_IDS   /**< Internal use only - not a valid ID! */
};



//-----------------------------------------------------------------------------
/** Values that may be specified for the ID bits of the descriptor word of
 *  a parapet WRITE_PARAM packet sent to the BSD microcontroller */
//-----------------------------------------------------------------------------
enum eBSDuC_WriteParam_IDs
{
    /** Indicates that the payload of the WRITE_PARAM packet consists of a
     *  BSDuC_MotorActuator_Request specifying desired propeller speed and
     *  actuator servo settings */
    PROP_AND_SERVOS = 0,

    NUM_BSDUC_WRITE_PARAM_IDS   /**< Internal use only - not a valid ID! */
};



//=============================================================================
/** Payload of a parapet packet sent from the BSD microcontroller to report
 *  sensor values
 */
struct BSDuC_Sensor_Report
{
    enum eStatusFlagMasks
    {
        H20_IS_DETECTED = 1,    /**< 1 if a water leak is detected; else 0 */
    };

    uint32_t StatusFlags;   /**< Sensor status flags */
    float DelsigADCVolts;   /**< Delta-Sigma ADC voltage reading */
    int32_t PropRPM;        /**< Propeller RPM (negative value indicates
                                 propeller is being driven in reverse) */
};



//=============================================================================
/** Payload of a parapet packet sent from the BSD microcontroller to report
 *  readings obtained from the battery monitor IC on the power supply PCB
 */
struct BSDuC_BatteryMonitor_Report
{
    float Battery_Volts;    /**< Instantaneous battery voltage (Volts) */
    float Battery_Amps;     /**< Instantaneous discharge current (Amps) */
    float Discharged_Amps;  /**< Accumulated discharge current (Amps) */

    uint32_t DischargeSeconds;  /**< Total discharge time in seconds */

    float Temperature_C;    /**< Temperature (Celsius) at battery monitor IC */

    enum eNumSerialNumberBytes { NUM_SERIAL_NUMBER_BYTES = 6 };

    /** 6-Byte serial number reported by the battery monitor */
    uint8_t SerialNumber[ NUM_SERIAL_NUMBER_BYTES ];

    /** State of communications between the BSD microcontroller and the
     *  battery monitor IC on the power supply PCB */
    typedef enum
    {
        NOT_CONNECTED = 0,  /**< Battery monitor IC is not responding */
        CONNECTED     = 1,  /**< Battery monitor IC is connected */
        COMM_ERROR    = 2,  /**< Battery monitor IC is responding, but one or
                                 more communication errors have occurred */
    } battery_monitor_state_t;

    /** State of communications with the battery monitor IC */
    battery_monitor_state_t BatteryMonitorState;
};






//=============================================================================
/** Payload of a parapet WRITE_PARAM packet sent to the BSD microcontroller to
 *  communicate desired motor speed and actuator servo settings
 */
struct BSDuC_MotorActuator_Request
{
    enum eServoIDs
    {
        SERVO_TOP_FIN = 0,   /**< Servo controlling the position of the
                                  top (rudder) AUV fin */

        SERVO_PORT_FIN,      /**< Servo controlling the position of the
                                  port-side AUV fin */

        SERVO_STARBOARD_FIN, /**< Servo controlling the position of the
                                  starboard-side AUV fin */

        NUM_SERVOS  /**< Internal use only - not a valid servo ID */
    };


    /** Desired propeller RPM - a negative value indicates that
     *  the motor should be driven in reverse */
    int32_t DesiredPropRPM;

    /** Desired servo duty cycles as a percentage of full
     *  scale (0.0 to 1.0), indexed by ID's in eServoIDs */
    float DesiredServoDutyCycle[ NUM_SERVOS ];
};



//=============================================================================
/** Payload of a parapet EXECUTE packet sent to the BSD microcontroller to
 *  request a soft reset */
struct BSDuC_SoftReset_Request
{
    /** Soft reset key used to guard against accidental reset requests: must
     *  be exactly equal to the sequence:
     *      Key[0] = 0x55aa55aa
            Key[1] = 0x81818181
            Key[2] = 0xaa55aa55
            Key[3] = 0x18181818
     */
    uint32_t Key[4];
};



#ifdef __cplusplus
}   // END namespace BSD_parapet
#endif


#endif /* BSD_PARAPET_PACKETS_H_ */
