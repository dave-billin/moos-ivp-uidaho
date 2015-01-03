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
/** @file VehicleEKF.h
 *
 * @brief
 * 	Declaration of the vehicle EKF object.
 */
//=============================================================================
#ifndef _VEHICLEEKF_H_
#define _VEHICLEEKF_H_

#include <stdint.h>
#include <vector>

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <newmat.h>	// newmat matrix library
#include "EKF.h"		// EKF base class
#include "LblBeacon.h"	// LBL transponder beacon class



namespace YellowSubNav
{

// Pre-declaration of VehicleEKF measurement classes defined later
// in this file
class ImuMeasurement;
class HeadingMeasurement;
class SpeedMeasurement;
class LblBeaconMeasurement;


//=============================================================================
/** An object implementing the Vehicle EKF (Extended Kalman Filter) used on the
 *  U of I AUV to estimate the current position based on navigation beacon
 *  ranges and sensor readings
 */
class VehicleEKF : public EKF
{
public:

	enum e_VehicleEkfNumStates { NUM_STATES = 5 };
	enum e_NoiseCounts { NUM_NOISE_SOURCES = 6	};

	static const float SPEED_CHECK; /**< Bound for speed measurements */
	static const float JUMP_CHECK;	 /**< Bound for range measurements */


	/** Measurement type ID's for use with ApplyMeasurement()
	 */
	enum e_EkfSensorTypeIds
	{
		MEAS_HEADING,			/**< Vehicle heading in degrees */
		MEAS_SPEED,				/**< Vehicle speed in meters per second */
		MEAS_LBL_RANGES,		/**< Ranges to LBL transponders measured by
		 	 	 	 	 	 	 	 navigation PING time of travel */
		MEAS_IMU,				/**< IMU measurements */
		NUM_EKF_MEASUREMENT_TYPES	/**< Used for bounds-checking. Not a valid
										 measurement ID */
	};


	enum e_EkfState_MatrixIndices
	{
		EST_COORD_E = 1,	/**< Estimated local grid East coordinate */
		EST_COORD_N,		/**< Estimated local grid North coordinate */
		EST_SPEED,			/**< Estimated vehicle speed (m/sec) */
		EST_YAW,			/**< Estimated vehicle yaw (radians) */
		EST_YAW_BIAS, 		/**< Estimated yaw bias (radians) */
	};



	//=========================================================================
	/** Creates an instance of the EKF object
	 * @param ReferenceGeodesy
	 *	Reference to a MOOS Geodesy object that is used to translate between
	 *	local coordinates and latitude/longitude
	 */
	VehicleEKF( CMOOSGeodesy& ReferenceGeodesy, uint32_t Verbosity = 0 );


	//=========================================================================
	/** Called when the object goes out of scope */
	virtual	~VehicleEKF();


	//=========================================================================
	/** Called to initialize the EKF with seed values.  This just wraps the
	 *	Initialize() method of the EKF base class for convenience
	 *
	 * @param CurrentE
	 *	Current estimated vehicle position - East (X) coordinate
	 *
	 * @param CurrentN
	 *	Current estimated vehicle position - North (Y) coordinate
	 *
	 * @param CurrentVelocity
	 *	Current vehicle velocity (meters per second)
	 *
	 * @param CurrentHeading
	 *	Current vehicle heading in degrees (using MOOS heading conventions)
	 *
	 * @param t
	 * 	Current time in seconds
	 *
	 * @param LblBeaconPingPeriod_sec
	 *	Period (in seconds) of LBL navigation pings.  Current supported values
	 *	include 0 (default), 2, and 15.
	 */
	void Initialize( NEWMAT::Real CurrentE, NEWMAT::Real CurrentN,
					 NEWMAT::Real CurrentVelocity, NEWMAT::Real CurrentHeading,
					 uint32_t LblBeaconPingPeriod_sec, double t );


	//=========================================================================
	/** Applies a pseudo-measurement of data from the IMU.
	 *
	 * @param ImuData
	 * 	A reference to an ImuMeasurement object containing the IMU measurement
	 * 	data to apply to the EKF
	 *
	 * @note
	 * 	IMU data is applied regardless of whether the EKF is online or not.
	 * 	Calling this function does not result in an EKF update.
	 */
	void ApplyImuMeasurement( const ImuMeasurement& ImuData );



	//=========================================================================
	/** Returns the current estimated local grid position
	 *
	 * @param[out] OUT_E
	 *	Reference to a variable to populate with the current estimated East
	 *	(X) coordinate
	 *
	 * @param[out] OUT_N
	 *	Reference to a variable to populate with the current estimated North
	 *	(Y) coordinate
	 *
	 * @return
	 *	true if an estimated position was returned in OUT_E and OUT_N; else
	 *	false if the EKF has not been initialized
	 */
	bool GetEstimatedCoordinates( NEWMAT::Real& OUT_E, NEWMAT::Real& OUT_N ) const;


	//=========================================================================
	/** Returns the current estimated velocity
	 * @param[out] OUT_Velocity
	 *	Reference to a variable to populate with the current estimated velocity
	 *
	 * @return
	 * 	true if an estimated velocity was returned in OUT_Velocity; else false
	 *	if the EKF has not been initialized
	 */
	bool GetEstimatedVelocity( NEWMAT::Real& OUT_Velocity ) const;


	//=========================================================================
	/** Returns the current estimated heading
	 * @param[out] OUT_Velocity
	 *	Reference to a variable to populate with the current estimated heading
	 *	(in degrees)
	 *
	 * @return
	 * 	true if an estimated heading was returned in OUT_Heading; else false
	 *	if the EKF has not been initialized
	 */
	bool GetEstimatedHeading( NEWMAT::Real& OUT_Heading ) const;



	//=========================================================================
	/** Sets the velocity of sound in water used by the vehicle EKF */
	void SetH20SoundVelocity( float Velocity )
	{ m_SoundVelocity = Velocity; }

	//=========================================================================
	/** Returns the velocity of sound in water used by the vehicle EKF */
	float GetH20SoundVelocity( void ) const
	{ return m_SoundVelocity; }


	//=========================================================================
	bool BadRangesFlagIsSet(void) const {return m_BadRangesFlag; }
	void ResetBadRangesFlag(void) { m_BadRangesFlag = false; }

	//=========================================================================
	bool BadJumpFlagIsSet(void) const {return m_BadJumpFlag; }
	void ResetBadJumpFlag(void) { m_BadJumpFlag = false; }

	//=========================================================================
	/** Returns a reference to one of the VehicleEKF's LBL navigation beacon
	 *  objects.
	 *
	 * @param BeaconId
	 *	An ASCII character identifying the beacon to return: ('A', 'B', 'C',
	 *	or 'D')
	 */
	LblBeacon& NavBeacon( char BeaconId );

protected:

	/** @name EKF class interface methods */
	//@{

	//=========================================================================
	/** Initializes the vehicle EKF using specified initial state values
	 *
	 * @param pInitialStateValues
	 *	Pointer to an array of (floating-point) initial values of EKF states.
	 *	Elements of this array correspond to the enumeration members in
	 *	e_EkfStates.
	 */
	void DoInitializeEKF( NEWMAT::Real* pInitialStateValues );


	//=========================================================================
	/** Gets called from within EKF::ApplyMeasurement() to handle a new
	 *  measurement.
	 *
	 * @param MeasurementData
	 *	An object containing the measurement data to apply.
	 *
	 * @param [out] OUT_ShouldUpdateEKF
	 *	If the data in MeasurementData is invalid or could not be applied,
	 *	this function should set this value to false.  Otherwise, if set to
	 *	true or left unmodified, an EKF measurement update will be carried out
	 *	upon this function returning.
	 *
	 * @precondition
	 *	The EKF has been initialized (this is verified before
	 *	PreUpdateMeasurement() is called)
	 *
	 * @see EKF::ApplyMeasurement, EKF::PreUpdateMeasurement
	 */
	void PreUpdateMeasurement( const EKFMeasurement& Measurement,
							   bool& OUT_ShouldUpdateEKF );


	//=========================================================================
	/** This function gets called just after an EKF measurement update.
	 *
	 * @param Measurement
	 * 	The measurement object that triggered the EKF update.
	 */
	void PostUpdateMeasurement( const EKFMeasurement& Measurement );


	//=========================================================================
	/** This function gets called to calculate the F matrix and states just
	 *	prior to EKF propagation
	 *
	 * @param Delta_t
	 * 	Time (in seconds) since the last EKF propagation
	 *
	 * @precondition
	 *	The EKF has been initialized (this is verified before
	 *	PrePropagate() is called)
	 */
	void PrePropagate( double Delta_t );


	//=========================================================================
	/** This function gets called just after the EKF is propagated.
	 *
	 * @param Delta_t
	 * 	Time (in seconds) preceding the EKF propagation just performed
	 *
	 */
	void PostPropagate( double Delta_t );

	//@}




	//=========================================================================
	/** Called to handle a heading sensor update
	 *
	 * @param Measurement
	 *	Reference to a HeadingMeasurement object containing measurement values.
	 *
	 * @return
	 *	true for a valid heading update; else false on error
	 */
	bool PreUpdate_Heading( const HeadingMeasurement& Measurement );


	//=========================================================================
	/** Called to handle a speed measurement update
	 *
	 * @param Measurement
	 *	Reference to a SpeedMeasurement object containing measurement values.
	 *
	 * @return
	 *	true for a valid speed update; else false on error
	 */
	bool PreUpdate_Speed( const SpeedMeasurement& Measurement );



	//=========================================================================
	/** Called to apply ranges from LBL navigation beacons
	 *
	 * @param Measurement
	 *	Reference to an LblBeaconMeasurement object containing measurement
	 *	values
	 *
	 * @return
	 *	true for a valid LBL beacon measurement; else false on error or if no
	 *	beacons returned PING travel times
	 */
	bool PreUpdate_LblBeacons( const LblBeaconMeasurement& Measurement );

private:
	CMOOSGeodesy& m_ReferenceGeodesy;	/**< MOOS Geodesy used to convert
											 between Latitude/Longitude and
											 local X/Y coordinates */

	double m_SoundVelocity;	/**< Velocity of sound in water used by the EKF */

	uint32_t m_LblPingPeriod;	/**< Period (seconds) of LBL navigation beacon
									 travel time measurements */

	bool m_BadJumpFlag;		/**< true if a range jump has occured; else false */
	bool m_BadRangesFlag;	/**< true if a range error occurs */

	NEWMAT::Real XEN[2];	/**< added 12.3.08 moved 7.2.10 */

	int m_NumRangesRejected;		/**< Count of rejected nav beacon ranges */

	int m_RangeRejectAbortTrigger;	/**< Number of rejected nav beacon ranges
										 that will trigger an abort */

	float m_NavBeaconRangeBound;	/**< Nav beacon ranges will be rejected if
										 they are greater than this value.
										 Counts toward m_NumRangesRejected */

	// Z-Gyro from IMU
	float m_Last_ZGyro;
	float m_LastUsed_ZGyro;

	uint32_t m_Verbosity;

	/** @name LBL navigation beacon objects
	 * @brief
	 * 	These objects encapsulate the location of the LBL navigation beacons
	 * 	used by the EKF for acoustic ranging
	 */
	//@{
	LblBeacon m_LblBeaconA;	/**< LBL transponder beacon A */
	LblBeacon m_LblBeaconB;	/**< LBL transponder beacon A */
	LblBeacon m_LblBeaconC;	/**< LBL transponder beacon A */
	LblBeacon m_LblBeaconD;	/**< LBL transponder beacon A */
	//@}


	// Prevent automatic generation of copy constructor and assignment operator
	VehicleEKF (const VehicleEKF&);
    const VehicleEKF& operator= (const VehicleEKF&);
};




//==============================
// VEHICLE EKF
// MEASUREMENT OBJECTS
//==============================

//=============================================================================
/** An object used to pass heading sensor measurements to the VehicleEKF
 *  ApplyMeasurement() method
 */
class HeadingMeasurement : public EKFMeasurement
{
public:
	/** Constructor
	 *
	 * @param Heading
	 * 	Vehicle heading in degrees (MOOS conventions)
	 */
	HeadingMeasurement(float Heading)
	: EKFMeasurement(VehicleEKF::MEAS_HEADING)
	{
		m_Heading = Heading;
	}

	float m_Heading;	/**< Vehicle heading in degrees (MOOS convention) */
};





//=============================================================================
/** An object used to pass speed measurements to the VehicleEKF
 *  ApplyMeasurement() method
 */
class SpeedMeasurement : public EKFMeasurement
{
public:
	/** Constructor
	 *
	 * @param Heading
	 * 	Vehicle heading in degrees (MOOS conventions)
	 *
	 * @param Velocity
	 *	Vehicle velocity in meters per second
	 */
	SpeedMeasurement(float Speed)
	: EKFMeasurement(VehicleEKF::MEAS_SPEED)
	{
		m_Speed = Speed;
	}

	float m_Speed;	/**< Vehicle speed (meters per second) */
};





//=============================================================================
/** An object used to pass PING travel times from LBL transponder beacons to
 * the VehicleEKF ApplyMeasurement() method
 */
class LblBeaconMeasurement : public EKFMeasurement
{
public:
	/** Constructor
	 *
	 * @param TravelTimes
	 * 	A four-element array containing navigation ping travel times in seconds
	 * 	for LBL beacons A through D.  TravelTimes[0] = A ... TravelTimes[3] = D.
	 * 	Travel times for beacons that did not reply to a PING should be set to
	 * 	0.0.
	 *
	 * @param Depth_m
	 * 	Vehicle depth in meters when beacon travel times were received
	 */
	LblBeaconMeasurement( NEWMAT::Real Ranges[], float Depth_m )
	: EKFMeasurement(VehicleEKF::MEAS_LBL_RANGES)
	{
		m_Depth = Depth_m;

		for (int i = 0; i < 4; i++)
		{
			m_BeaconRange[i] = Ranges[i];
		}
	}

	float m_Depth;	/**< Depth of the vehicle (meters) when navigation beacon
						 travel times were received */

	float m_BeaconRange[4]; /**< Beacon range in meters. */
};




//=============================================================================
/** An object used to pass IMU measurements to the VehicleEKF
 * ApplyMeasurement() method.
 */
class ImuMeasurement : public EKFMeasurement
{
public:
	/** Creates the measurement object
	 *
	 * @param wz
	 * 	Lateral velocity (radians per second)
	 *
	 * @param wy
	 * 	Normal velocity (radians per second)
	 *
	 * @param phi
	 * 	Acceleration X
	 *
	 * @param theta
	 * 	Acceleration Y
	 */
	ImuMeasurement(float wz, float wy, float phi, float theta)
	: EKFMeasurement(VehicleEKF::MEAS_IMU)
	{
		m_wz = wz;
		m_wy = wy;
		m_phi = phi;
		m_theta = theta;
	}

	float m_wz;		/**< (From IMU) Angular velocity about the IMU Z-axis */
	float m_wy;		/**< (From IMU) Angular velocity about the IMU Y-axis */
	float m_phi;	/**< (From SPOCK) Rotation (radians) about X-axis (with
						 X axis running bow-stern) */
	float m_theta;	/**< Rotation (radians) about Y-axis (with Y axis
						 running port-starboard) */
};


} // END namespace YellowSubNav

#endif // END #ifndef _VEHICLEEKF_H_
