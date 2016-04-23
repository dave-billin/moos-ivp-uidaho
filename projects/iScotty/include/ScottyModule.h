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
/** @file ScottyModule.h
 * @brief
 *	Declaration of the ScottyModule class
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef _SCOTTYMODULE_H_
#define _SCOTTYMODULE_H_

#include <string>
#include <stdint.h>
#include "MOOS/libMOOS/MOOSLib.h"
#include "LibBunnySock.h"		// BunnySock communications library
#include "UAVnet_PacketTypes.h"	// UAVnet packet structure definitions

//=============================================================================
/** A class to encapsulate communications and interaction with the SCOTTY
 * motor and actuator control module used in the University of Idaho AUV
 *
 * @ingroup iScotty
 */
class ScottyModule: BunnySock::BunnySockListener
{
public:

   //=========================================================================
   /** Creates an instance of the ScottyModule object and starts its network
    *  connection thread
    *
    * @param HostName
    *	Hostname or IP address of the SCOTTY module to connect to
    *
    * @param Port
    *	Network port the target SCOTTY module is listening on for a BunnySock
    *	connection
    *
    * @param Verbosity
    *	Verbosity level to use when printing debug messages (0 = none, 1+ gives
    *	increasing amounts of debugging messages
    *
    * @throw
    * 	A CMOOSException on failure to create a BunnySock network socket to
    * 	communicate with SCOTTY
    */
   ScottyModule(std::string& HostName, uint16_t Port, int Verbosity = 0);

   //=========================================================================
   /** Called when the object goes out of scope */
   virtual ~ScottyModule();

   //=========================================================================
   /** Sends actuator settings to SCOTTY
    *
    * @param Rudder_rad
    *	Desired rudder angle in radians (0..+/-Pi)
    *
    * @param Elevator_rad
    * 	Desired elevator angle in radians (0..+/-Pi)
    *
    * @param Aileron_rad
    * 	Desired aileron angle in radians (0..+/-Pi)
    *
    * @param PropThrust
    *	Desired propeller thrust as a percentage of full scale
    */
   void SetActuators(float Rudder_rad, float Elevator_rad, float Aileron_rad,
                     float PropThrust);

   //=========================================================================
   /** Sets servo trims applied to the actuators on SCOTTY as actuator angle
    * 	offsets
    *
    * @param RudderOffset_rad
    * 	Offset in radians applied to the rudder servo's center position
    *
    * @param ElevatorOffsetL_rad
    * 	Offset in radians applied to the left elevator servo's center position
    *
    * @param ElevatorOffsetR_rad
    * 	Offset in radians applied to the right elevator servo's center position
    *
    * @param CouplingCoeff
    *	Servo coupling coefficient
    */
   void SetServoTrimAngles(float RudderOffset_rad, float ElevatorOffsetL_rad,
                           float ElevatorOffsetR_rad, float CouplingCoeff);

   //=========================================================================
   /** Sets SCOTTY servo center positions as raw 8-bit servo values
    *
    * @param Rudder
    * 	Servo value SCOTTY should use for the rudder center position (0..254)
    *
    * @param ElevatorL
    * 	Servo value SCOTTY should use for the left elevator center position
    * 	(0..254)
    *
    * @param ElevatorR
    * 	Servo value SCOTTY should use for the right elevator center position
    * 	(0..254)
    *
    * @param CouplingCoeff
    * 	Servo coupling coefficient to apply
    */
   void SetServoCenters(uint8_t Rudder, uint8_t ElevatorL, uint8_t ElevatorR,
                        float CouplingCoeff);

   //=========================================================================
   /** Returns the values used for servo center positions
    *
    * @param [out] Rudder
    * 	Reference to a variable to populate with the rudder servo center
    * 	position (0..255)
    *
    * @param [out] ElevatorL
    * 	Reference to a variable to populate with the left elevator servo center
    * 	position (0..255)
    *
    * @param [out] ElevatorR
    * 	Reference to a variable to populate with the right elevator servo
    * 	center position (0..255)
    *
    * @returns
    * 	true if servo settings were returned in the referenced variables; else
    * 	false if the SCOTTY module is not connected
    */
   bool GetServoCenters(uint8_t& Rudder, uint8_t& ElevatorL,
                        uint8_t& ElevatorR);

   //=========================================================================
   /** Returns the value of the coupling coefficient used by the SCOTTY module
    *
    * @param CouplingCoeff
    *	Reference to a variable to populate with the SCOTTY servo coupling
    *	coefficient
    *
    * @returns
    * 	true if the coupling coefficient was returned by reference; else false
    * 	if the SCOTTY module is not connected.
    */
   bool GetServoCouplingCoefficient(float& CouplingCoeff);

   //=========================================================================
   /** Returns true if a TCP connection with the SCOTTY module is open */
   bool IsConnected(void) const;

   //========================================
   // Actuator position Accessor functions
   //========================================

   /** Returns the current angle of the rudder in radians (0..+/-Pi) or zero
    * if the SCOTTY module is not connected */
   float RudderAngle() const;

   /** Returns the current angle of the right elevator in radians (0..+/-Pi)
    * or zero if the SCOTTY module is not connected */
   float ElevatorAngle() const;

   /** Returns the current aileron angle setting in radians (0..+/-Pi) or
    * zero if the SCOTTY module is not connected */
   float AileronAngle() const;

   /** Returns the current measured propeller thrust as a percentage of full
    * scale or zero if the SCOTTY module is not connected */
   float Thrust() const;

   /** Returns the current measured propeller RPM or zero if the SCOTTY module
    * is not connected */
   uint16_t PropellerRpm() const;

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
                          BunnySock::BunnySockNode& Node,
                          double TimeStamp_sec);

private:
   BunnySock::BunnySockTcpNode* m_pNode; /**< Connection to SCOTTY */
   int m_Verbosity; /**< Verbosity level for debugging messages */

   float m_RudderAngle_rad; /**< Current rudder angle setting (radians) */
   float m_AileronAngle_rad; /**< Current aileron angle setting (radians) */
   float m_ElevatorAngle_rad; /**< Current elevator angle setting (radians) */

   uint16_t m_DesiredPropRPM; /**< Last desired Prop RPM sent to SCOTTY */

   uint16_t m_MeasuredPropRPM; /**< Measured propeller RPM reported by SCOTTY
    module */

   bool m_DesiredServoCentersSet; /**< true if SetServoCenters has been
    called to set desired values; else
    false */

   uint8_t m_RudderServoCenter; /**< Rudder servo center position indicated
    by the SCOTTY module */
   uint8_t m_ElevLServoCenter; /**< Left elevator (aileron) servo center
    position reported by SCOTTY */
   uint8_t m_ElevRServoCenter; /**< Right elevator servo center position
    reported by SCOTTY */
   float m_CouplingCoefficient; /**< Servo coupling coefficient reported by
    SCOTTY */

   //=========================================================================
   /** Helper function to handle an actuator trim report network packet
    *  received from the SCOTTY module */
   void HandleServoTrimReport(CommandPacket_t* pPacket);
};

#endif	// END #ifndef _SCOTTYMODULE_H_
