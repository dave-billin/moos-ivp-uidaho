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
/** @file LblPositionEstimator.h

@brief
	Declaration of the LblPositionEstimator class
	
@author Dave Billin

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _LBLPOSITIONESTIMATOR_H_
#define _LBLPOSITIONESTIMATOR_H_

#include <string>
#include <vector>
#include "MOOS/libMOOS/MOOSLib.h"	// MOOS core library
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "newmat.h"	// newmat matrix library

#include "LblBeacon.h"	// LBL beacon class



#define NUM_LBL_BEACONS	4



/*
//Position information for one buoy
typedef struct{
   float xloc;		//Local coordinates
   float yloc;
   float zloc;
} buoy_dat;

//LBL data for combination of buoys
typedef struct{
	long gps_zero_lat;			// Latitude at origin of (X,Y)
	long gps_zero_long;			// Longitude at origin of (X,Y)
	buoy_dat buoy[NUM_BUOYS];	// X/Y/Z of buoys
	int c_in_h20;				// Speed of sound in water
	char buoy_state[NUM_BUOYS];	// Active bouy flags '1' = active
								// '0' = inactive
} LBL_dat;
*/

// Default velocity of sound in water (meters per second)
#define DEFAULT_H20_SOUND_VELOCITY_mps	1470

//LBL defines
#define PING_SPACING       2
#define PING_INTERVAL      16

enum 
{ 
	BUOYA, 
	BUOYB, 
	BUOYC, 
	BUOYD, 
	NUM_BUOYS 
};

#define SIZE_OF_LBL        62	// DB: max length of an SNTTA sentence maybe?
#define MPERDEG .0111311		// conversion for nanodegrees of lat/lon to meters

//default buoy A,B,C locations in meters from missions on 10-20-05
//we could use zero as the default locations but these numbers give us
//a better idea as to the accuracy of the new numbers entered.
//These values are changed via the LBL web page
#define DEFAULT_PBAX		-17.0411
#define DEFAULT_PBAY		-167.7467
#define DEFAULT_PBAZ		7.62
#define DEFAULT_PBBX		0.0
#define DEFAULT_PBBY		0.0
#define DEFAULT_PBBZ		6.4
#define DEFAULT_PBCX		-79.126
#define DEFAULT_PBCY		-30.2484
#define DEFAULT_PBCZ		7.62
#define DEFAULT_PBDX		0.0
#define DEFAULT_PBDY		0.0
#define DEFAULT_PBDZ		6.4
//Default longitude and latitude are given for Buttonhook Bay buoys
#define DEFAULT_LONG0	-1165756807
#define DEFAULT_LAT0		479527384



namespace YellowSubNav
{

//=============================================================================
/** An object to encapsulate the calculation and estimation of a position
	based on LBL transponder travel times */
//=============================================================================
class LblPositionEstimator
{
public:

	static float sm_SoundVelocity_mps;	/**< Speed of sound in water
											 (meters per second) */

	//=========================================================================
	/** Creates an instance of the LblPositionEstimator class
	@param Geodesy
		A reference to a geodesy from which local X and Y coordinates will be
		derived
	*/
	LblPositionEstimator( CMOOSGeodesy& Geodesy );

	//=========================================================================
	//! Called when the object goes out of scope
	~LblPositionEstimator();


	//=========================================================================
	/** Calculates new beacon ranges based on navigation ping travel times.
	 * @param NavPingTravelTimes_sec
	 *	Measured times of travel (seconds) for a Navigation PING returned from
	 *	each LBL transponder.  Element 0 in this array corresponds to
	 *	transponder A, element 1 to B, and so on...
	 *
	 * @param NavPingTravelTimes_sec
	 *	Measured nav ping travel times from the four navigation beacons in
	 *	seconds (or 0.0 if no nav ping response was received from a beacon)
	 *
	 * @param CurrentX
	 * 	Local X coordinate when the travel times were measured
	 *
	 * @param CurrentY
	 *  Local Y coordinate when the travel times were measured
	 *
	 * @param Depth_m
	 *	Depth (meters) when the travel times were measured
	*/
	void UpdateBeaconRanges( float NavPingTravelTimes_sec[NUM_LBL_BEACONS],
						     double CurrentX, double CurrentY,
						     double Depth_m );



	//=========================================================================
	/* Calculates the estimated position of the vehicle based on beacon
		distances derived from navigation ping travel times

	@return
		- A positive value if the estimator calculated a new position
		- Zero if no LBL beacon ranges have changed
		- A negative value if the estimator failed to calculate a position

	@postcondition
		If a new 
		position was successfully calculated, or if position hasn't
		changed, it will be updated in
		the estimator, and can be obtained by calling GetPosition().  If a
		new position could not be calculated, the previous position will be
		left unaltered with a reduced quality factor.
	*/
	void CalculatePosition( void );



	//=========================================================================
	/** Returns the current estimated position as local grid coordinates

	@param [out] X
		Reference to a double to be populated with the estimated X coordinate 
		of the vehicle

	@param [out] Y
		Reference to a double to be populated with the estimated Y coordinate 
		of the vehicle

	@param [out] Depth_m
		Reference to a double to be populated with the estimated depth

	@return
		A value between 0.0 and 1.0 representing the quality of the position 
		estimate.  This value ranges from 0.0 (not enough beacons to derive a
		position) to 1.0 (position estimated based on 3+ beacons).
	*/
	double GetLocation_LocalGrid( double& X, double& Y, double Depth_m ) const;



	//=========================================================================
	/** Returns the current estimated position as latitude,longitude and depth

	@param [out] Latitude
		Reference to the variable to be populated with the estimated latitude
	@param [out] Y
		Reference to the variable to be populated with the estimated longitude

	@param [out] Depth_m
		Reference to the variable to be populated with the estimated depth

	@return
		A value between 0.0 and 1.0 representing the quality of the position
		estimate.  This value ranges from 0.0 (not enough beacons to derive a
		position) to 1.0 (position estimated based on 3+ beacons).
	*/
	double GetLocation( double& Latitude, double& Longitude, double Depth_m )
			const;

	
	//=========================================================================
	/** Sets the location of an LBL transponder beacon

	@param BeaconId
		The ID of the beacon whose position is being set: 'A', 'B', 'C', or 'D'

	@param Latitude
		The latitude to apply to the beacon

	@param Longitude
		The longitude to apply to the beacon

	@param Depth_m
		The depth (meters) to apply to the beacon

	@return
		true if the position was assigned to the beacon; else false
	*/
	bool SetNavBeaconLocation( char BeaconId, double Latitude,
							   double Longitude, double Depth_m );



	//=========================================================================
	/** Sets the location of an LBL transponder beacon using local grid
	 *  coordinates
	 *
	 * @param BeaconId
	 *	The ID of the beacon whose position is being set: 'A', 'B', 'C', or 'D'
	 *
	 * @param X
	 *	X coordinate to assign to the beacon
	 *
	 * @param Y
	 *	Y coordinate to assign to the beacon
	 *
	 * @param Depth_m
	 * 	Depth (meters) to assign to the beacon
	 *
	 * @return
	 *	true if the position was assigned to the beacon; else false if
	 *	BeaconId does not specify a valid beacon
	*/
	bool SetNavBeaconLocation_LocalGrid( char BeaconId,
										 double X, double Y, double Depth_m );



	//=========================================================================
	/** Returns a pointer to a specified navigation beacon
	 *
	 * @param BeaconId
	 * 	ID of the beacon to return: 'A', 'B', 'C', or 'D'
	 *
	 * @return
	 * 	A pointer to the specified beacon or NULL if BeaconId does not specify
	 *  a valid beacon.
	 */
	const LblBeacon* GetNavBeacon( char BeaconId ) const;



	//=========================================================================
	/* Called from within whoi_parse
	PARAM sentence
		Pointer to the first travel time field in an SNTTA sentence from the 
		modem
		
	RETURNS
		1 on success; 0 on failure
	*/
	int compute_range(char *sentence);



	//=============================================================================
	/* Updates LBL positions with new positions
	PARAM x
		New x position
		
	PARAM y
		New y position
		
	PARAM buoys
		Bitfield of flags indicating which transponders were used to calculate the 
		new	position.  A transponder's bit is set to 1 if it was used; 0 if it was
		not used in the calculated position
			- bit 0 = Transponder A
			- bit 1 = Transponder B
			- bit 2 = Transponder C
			- bit 3 = Transponder D
	*/
	void update_calc_pos( float x, float y, int buoys );



	//=============================================================================
	void init_LBL();

	//=============================================================================
	void true_init_pos(void);

protected:

	//=====================================================================
	/** Inner class used to encapsulate a position in the local coordinate
	 *  system
	 *
	 *  @param x	Local grid X-coordinate
	 *  @param y	Local grid Y-coordinate
	 *  @param Depth	Depth in meters
	 */
	class LocalGridLocation
	{
	public:
		LocalGridLocation(double x = 0, double y = 0, double depth = 0)
		: X(x), Y(y), Depth(depth) {;}

		~LocalGridLocation() {;}

		double X;	//!< Local x-coordinate
		double Y;	//!< Local y-coordinate
		double Depth;	//!< depth (meters)
	};


private:

	double m_CurrentDepth_m;	/**< Current depth of the vehicle in meters */
	vector <LblBeacon> m_LblBeacon;	/**< Working vector of transponder beacon objects */
	uint m_NumActiveBeacons;	/**< Number of active beacons */
	uint m_ActiveBeaconFlags;	/**< Flags indicating which beacons were
									 active in the last update.  Flag bit
									 0 corresponds to beacon A (1 = active
									 0 = inactive), bit 1 to beacon B, and
									 so on... */


	CMOOSGeodesy& m_Geodesy;		/** Reference geodesy for the vehicle used to
										derive local X and Y coordinates */

	float m_ConfidenceFactor;		/** 1.0 for a 3+ buoy solution; 0.7 for a
										2-buoy solution.  Decreases by 0.2 each
										time transponderEstimatePosition_3Beacon ranges are updated with
										only one active buoy */

	LocalGridLocation m_EstimatedPosition;	//!< Current estimated position

	NEWMAT::Matrix B;		/**< B matrix used for position estimation */
	NEWMAT::Matrix Binv;	/**< Inverse of B matrix used to estimate position */
	NEWMAT::Matrix Z;		/**< Z matrix used for position estimation */


	//=====================================================================
	//! Initializes B and Z matrices used for position estimation
	void InitRange( void );

	void initialize_range(void);






//	LBL_dat LBL;

	// global for WHOI receive and LBL calculations



	float est_pos_x;
	float est_pos_y;

	float pos_best_x;		// Least squared x error from exact/estimated position calculations
	float pos_best_y;		// Least squared y error from exact/estimated position calculations
	float pos_best_diff;	// Least squared error magnitude from exact/estimated position calculations


	/*	A flag bitfield indicating the LBL transponders used to calculate the
		position stored in pos_best_x and pos_best_y.  A bit is set to 1 if the
		corresponding transponder was used to calculate the position; otherwise, it
		is set to zero, indicating the transponder travel time was not available
		for the calculation (and was possibly missing from an SNTTA packet)
			bit 0 - Transponder A
			bit 1 - Transponder B
			bit 2 - Transponder C
			bit 3 - Transponder D
	*/
	int pos_best_buoys;


	//=============================================================================
	/* Two-buoy position solution derived from the Woods-Hole Oceanographic
	 * Institute paper "Integrated Acoustic Communication and Navigation for
	 * Multiple UUVs"
	 *
	 * @param [in] pBeacon1
	 *	Reference to an LblBeacon object describing the first beacon used to
	 *	estimate position
	 *
	 * @param [in] pBeacon2
	 *	Reference to an LblBeacon object describing the second beacon used to
	 *	estimate position
	 *
	 * @param [out] EstPositionA
	 * 	Reference to a LocalGridLocation object to be populated with the first
	 *  estimated position from the 2-beacon solution
	 *
	 * @param [out] EstPositionB
	 * 	Reference to a LocalGridLocation object to be populated with the first
	 *  estimated position from the 2-beacon solution
	 *
	 * @return
	 *	true if an estimated position was calculated; else false on failure
	 */
	bool EstimatePosition_2Beacon( LblBeacon& Beacon1, LblBeacon& Beacon2,
								   LocalGridLocation& EstPositionA,
								   LocalGridLocation& EstPositionB );


	//=============================================================================
	/* Three-buoy position solution derived from WHOI paper Integrated Acoustic
	 * Communication and Navigation for Multiple UUVs
	 *
	 * @param [in] Beacon1
	 * 	First beacon used to calculate the position
	 *
	 * @param [in] Beacon2
	 * 	Second beacon used to calculate the position
	 *
	 * @param [in] Beacon3
	 * 	Third beacon used to calculate the position
	 *
	 * @param [in] MissingBeaconIndex
	 *	Index of the missing beacon in the B matrix
	 *
	 * @param [out] EstimatedPosition
	 * 	Reference to a LocalGridLocation to be populated with the 3-beacon
	 * 	position solution
	*/
	int EstimatePosition_3Beacon( LblBeacon& Beacon1, LblBeacon& Beacon2,
								  LblBeacon& Beacon3, uint MissingBeaconIndex,
								  LocalGridLocation& EstimatedPosition);


};

};  // END namespace YellowSubNav

#endif	// END #ifndef _LBLPOSITIONESTIMATOR_H_
