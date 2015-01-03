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
/** @file LblBeacon.h

 @brief
 *** Add a description of your source file here ***

 @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#ifndef LBLBEACON_H_
#define LBLBEACON_H_

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

namespace YellowSubNav
{


class LblBeacon
{
public:

	//=========================================================================
	/** Creates an instance of the object
	 *
	 * @param RefGeodesy
	 *	Reference to the Geodesy object used to derive local coordinates
	 *
	 * @param Lat
	 * 	Latitude where the beacon is located
	 *
	 * @param Long
	 * 	Longitude where the beacon is located
	 *
	 * @param Depth
	 * 	Depth of the beacon in meters
	 */
	LblBeacon( CMOOSGeodesy& RefGeodesy, double Lat = 0, double Long = 0,
			   double Depth = 0 );

	~LblBeacon();


	//=========================================================================
	/** Sets the absolute latitude, longitude, and depth of the beacon
	 * @param Latitude
	 * 	The latitude of the beacon
	 *
	 * @param Longitude
	 * 	The longitude of the beacon
	 *
	 * @param Depth
	 * 	Depth of the beacon in meters
	 */
	void SetLocation(double Latitude, double Longitude, double Depth);


	//=========================================================================
	/** Returns the absolute latitude, longitude and depth of the beacon
	 *
	 * @param [out] Latitude
	 * 	Variable to be populated with the latitude of the beacon
	 *
	 * @param [out] Longitude
	 *  Variable to be populated with the longitude of the beacon
	 *
	 * @param [out] Depth
	 * 	Variable to be populated with the depth of the beacon in meters
	 */
	void GetLocation( double& Latitude, double& Longitude, double& Depth ) const;



	//=========================================================================
	/** Sets the location of the beacon using local grid coordinates
	 *
	 * @param [in] E
	 * 	East (X) coordinate (in meters) of the beacon location
	 *
	 * @param [in] N
	 * 	North (Y) coordinate (in meters) of the beacon location
	 *
	 * @param [in] Depth
	 * 	Depth of the beacon in meters
	 */
	void SetLocation_LocalGrid(double E, double N, double Depth);


	//=========================================================================
	/** Returns the coordinates of the beacon on the local grid
	 * @param [out] E
	 * 	East (X) coordinate (in meters) of the beacon location
	 *
	 * @param [out] N
	 * 	North (Y) coordinate (in meters) of the beacon location
	 *
	 * @param [out] Depth
	 * 	Depth of the beacon in meters
	 */
	void GetLocation_LocalGrid( double& E, double& N, double& Depth) const;


	//=========================================================================
	double GetLatitude( void ) const { return m_Latitude; }
	double GetLongitude( void ) const { return m_Longitude; }
	double GetX( void ) const { return m_LocalGridX; }
	double GetY( void ) const { return m_LocalGridY; }
	double GetDepth( void ) const { return m_Depth_m; }


	//=========================================================================
	/** Sets the last nav ping time */
	void SetLastPingTime( float PingTime_sec )
	{ m_LastPingTime_sec = PingTime_sec; }

	/** Returns the last nav ping time */
	float GetLastPingTime( void ) const { return m_LastPingTime_sec; }


	//=========================================================================
	/** Calculates the distance in meters between the beacon and a specified
	 *  local grid location
	 *
	 * @param RefX	X location of the reference point
	 * @param RefY	Y location of the reference point
	 * @param RefDepth	Depth (meters) of the reference point
	 *
	 * @return
	 * 	Distance in meters between the beacon and the reference point
	 */
	float GetDistance_LocalGrid( double RefX, double RefY, double RefDepth ) const;


	//=========================================================================
	/** Calculates the distance in meters between the beacon and a specified
	 *  location
	 *
	 * @param RefLatitude	Latitude of the reference point
	 * @param RefLongitude	Longitude of the reference point
	 * @param RefDepth		Depth (meters) of the reference point
	 *
	 * @return
	 * 	Distance in meters between the beacon and the reference point
	 */
	float GetDistance( double RefLatitude, double RefLongidude,
					   double RefDepth ) const;

	/** Assignment operator */
	LblBeacon& operator=(LblBeacon& rhs);

private:
	double m_Latitude;	//! Latitude where the beacon is located
	double m_Longitude;	//! Longitude where the beacon is located
	double m_LocalGridX;
	double m_LocalGridY;
	double m_Depth_m;		//! Beacon depth (meters)
	float m_LastPingTime_sec;	/**< Last measured time of travel from the
									 beacon */

	CMOOSGeodesy& m_ReferenceGeodesy;	/**< Geodesy used to derive local
										 coordinates */

	LblBeacon( void );
};


};   // END namespace YellowSubNav

#endif /* LBLBEACON_H_ */
