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
/** @file SpockModule.h
 *
 * @brief
 *	Declaration of the SpockModule class used to provide an interface to the
 *	SPOCK Rabbit module on the U of I YellowSub AUV
 *
 * @author	Dave Billin
 */
//=============================================================================
#ifndef SPOCKMODULE_H_
#define SPOCKMODULE_H_

#include <stdint.h>
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "LibBunnySock.h"
#include "UAVnet_PacketTypes.h"

/** A class to wrap interaction with the SPOCK module and provide translation
 * of Rabbit AUV sensor values to MOOS conventions and units.
 */
class SpockModule: public BunnySock::BunnySockListener
{
public:

   //=========================================================================
   /** Creates an instance of the object and starts its network connection
    * thread
    *
    * @param HostName
    *	Hostname or IP address of the SPOCK module to connect to, or "UDP" to
    *	receive sensor packets sent in UDP broadcast datagrams
    *
    * @param TcpPort
    *	Network port the target SPOCK module is listening on for a BunnySock
    *	TCP connection
    *
    * @param UdpPort
    * 	Network port SPOCK uses to broadcast sensor packets
    *
    * @param Verbosity
    *	Verbosity level to use when printing debug messages (0 = none, 1+ gives
    *	increasing amounts of debugging messages
    *
    * @throw
    * 	A CMOOSException on failure to create a BunnySock network socket to
    * 	communicate with SPOCK
    */
   SpockModule(std::string& HostName, uint16_t Port, int Verbosity = 0);

   //=========================================================================
   /** Called when the object goes out of scope */
   virtual ~SpockModule();

   //=========================================================================
   /** Returns true if a TCP connection with the SPOCK module is open */
   bool IsConnected(void) const;

   //=========================================================================
   /** Requests that the SPOCK module measure all of its sensors and send them
    * in a multi-sensor packet (this is the traditional 'sensor packet'
    * produced by the SPOCK module in the Rabbit AUV)
    */
   void RequestMultiSensors(void);

   //=========================================================================
   /** Requests that the SPOCK module measure the depth and return a single
    * depth packet.
    */
   void RequestDepth(void);

   //=========================================================================
   /** Enables or disables automatic reporting of sensors on the SPOCK module
    *
    * @param EnableAutoReporting
    * 	Set this to true if SPOCK should automatically send sensor updates at
    *  a regular rate, or false if sensor data should only be sent when it is
    *  requested.
    */
   void SetAutoReportingEnablement(bool EnableAutoReporting);

   //=========================================================================
   /** Tells SCOTTY to "zero" a specified sensor (i.e. use the current sensor
    * measurement as a zero reference)
    *
    * @param SensorID
    * 	Integer ID from e_SensorIds specifying which sensor to zero
    */
   void ZeroSensor(int SensorID);

   //=========================================================================
   /** Returns true if SPOCK has reported sensor values since the last time
    *  this function was called.
    */
   bool SensorValuesAreFresh(void);

   //==============================
   // Sensor value accessors
   //==============================

   /** Returns the compass heading in degrees (adheres to MOOS Conventions) */
   inline float CompassHeading(void) const
   {
      return m_CompassHeading;
   }

   /** Returns the compass pitch angle in radians */
   inline float CompassPitch(void) const
   {
      return m_CompassPitch;
   }

   /** Returns the compass roll angle in radians */
   inline float CompassRoll(void) const
   {
      return m_CompassRoll;
   }

   /** Returns the compass dip angle in radians */
   inline float CompassDip(void) const
   {
      return m_CompassDip;
   }

   /** Returns the measured depth in meters based SPOCK pressure sensor */
   inline float Depth(void) const
   {
      return m_Depth;
   }

   /** Returns the measured pitch from the SPOCK accelerometers in radians */
   inline float AccelPitch(void) const
   {
      return m_AccelPitch;
   }

   /** Returns the measured roll from the SPOCK accelerometers in radians */
   inline float AccelRoll(void) const
   {
      return m_AccelRoll;
   }

   /** Returns the GPS latitude in degrees */
   inline float GpsLatitude(void) const
   {
      return m_GPS_Latitude;
   }

   /** Returns the GPS longitude in degrees */
   inline float GpsLongitude(void) const
   {
      return m_GPS_Longitude;
   }

   /** Returns the GPS horizontal position error in meters */
   inline float GpsHPE(void) const
   {
      return m_GPS_HPE;
   }

   /** Returns the GPS velocity in meters per second */
   inline float GpsVelocity(void) const
   {
      return m_GPS_Velocity;
   }

   /** Returns the GPS heading in degrees (adheres to MOOS conventions */
   inline float GpsHeading(void) const
   {
      return m_GPS_Heading;
   }

   /** Returns the GPS hours in the day */
   inline uint32_t GpsHours(void) const
   {
      return m_GPS_Hours;
   }

   /** Returns the GPS minutes in the hour */
   inline uint32_t GpsMinutes(void) const
   {
      return m_GPS_Minutes;
   }

   /** Returns the GPS seconds in the minute */
   inline uint32_t GpsSeconds(void) const
   {
      return m_GPS_Seconds;
   }

   /** Returns true if a water leak is detected */
   inline bool WaterLeakIsDetected(void) const
   {
      return m_WaterLeakIsDetected;
   }

   /** Returns the temperature on the SPOCK gyros in degrees C */
   inline float Temperature(void) const
   {
      return m_Temperature;
   }

   /** Returns the measured AUV battery bus voltage in DC volts */
   inline float BatteryVoltage(void) const
   {
      return m_BatteryVoltage;
   }

   //========================================
   // BunnySockListener interface functions
   //========================================

   //=========================================================================
   /** @internal */
   void OnPacketReceived(BunnySock::BunnySockPacket& RxPacket,
                         BunnySock::BunnySockNode& Node, double TimeStamp_sec);

   //=========================================================================
   /** @internal */
   void OnConnectionEvent(BunnySockListener::ConnectionEventId EventId,
                          BunnySock::BunnySockNode& Node, double TimeStamp_sec);

   /** @enum e_SensorIds
    * @brief
    * 	Sensor ID's for use with the ZeroSensor() method
    *
    * @see ZeroSensor
    */
   enum e_SensorIds
   {
      SensorId_All = -1, /**< ALL sensor values */
      SensorId_Depth, /**< Depth (pressure) sensor */
      NUM_SENSOR_IDS /**< The number of valid sensor ID's (used for
       bounds-checking - not a valid sensor ID) */
   };

private:
   BunnySock::BunnySockNode* m_pNode; /**< Connection to SCOTTY */
   bool m_receiving_udp; /**< true to receive UDP broadcast sensor packets */

   int m_Verbosity; /**< Verbosity level for debugging messages */

   bool m_AutoReportingEnabled;/**< true if SPOCK should automatically report
    sensor data; false if sensors should only
    be reported when requested */

   bool m_SensorValuesAreFresh;/**< true if sensor values have been received
    since the last request; else false */

   //===============================
   // Received sensor values
   //===============================
   float m_Depth; /**< Depth (meters) from pressure sensor */
   float m_BatteryVoltage; /**< AUV battery bus voltage (Volts) */
   bool m_WaterLeakIsDetected; /**< true if a water leak is detected;
    else false */

   float m_Temperature; /**< Temperature (degrees C) from SPOCK gyros */

   float m_GPS_Longitude; /**< GPS longitude (degrees) */
   float m_GPS_Latitude; /**< GPS latitude (degrees) */
   float m_GPS_Velocity; /**< GPS velocity (meters per second) */
   float m_GPS_HPE; /**< GPS horizontal position error (meters) */
   float m_GPS_Heading; /**< GPS heading (degrees) */
   uint16_t m_GPS_Hours; /**< GPS Time: hour in the day */
   uint16_t m_GPS_Minutes; /**< GPS Time: minute in the hour */
   uint16_t m_GPS_Seconds; /**< GPS Time: second in the minute */

   float m_AccelPitch; /**< Pitch (radians) from SPOCK accelerometers (adheres
    to MOOS conventions) */

   float m_AccelRoll; /**< Roll (radians) from SPOCK accelerometers (adheres
    to MOOS conventions) */

   float m_CompassHeading; /**< Digital compass heading (degrees) */

   float m_CompassPitch; /**< Pitch (radians) from digital compass (adheres
    to MOOS conventions)*/

   float m_CompassRoll; /**< Roll (radians) from digital compass (adheres
    to MOOS conventions)*/

   float m_CompassDip; /**< Dip measurement (radians) from digital compass */

   //=========================================================================
   /** Helper function used to handle received multisensor packets */
   void HandleMultiSensorPacket(SensorPacket_t* pPacket);

   //=========================================================================
   /** Helper function used to handle received depth packets */
   void HandleDepthPacket(DepthPacket_t* pPacket);

};

#endif /* SPOCKMODULE_H_ */
