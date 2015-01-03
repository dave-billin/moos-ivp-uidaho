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
/** @file LblBeacon.cpp

 @brief
 *** Add a description of your source file here ***

 @author	dave
 */
//=============================================================================

#include <math.h>
#include "LblBeacon.h"


namespace YellowSubNav
{


//=============================================================================
LblBeacon::LblBeacon( CMOOSGeodesy& RefGeodesy,
			   	   	  double Lat, double Long, double Depth )
: m_Latitude(0.0),
  m_Longitude(0.0),
  m_LocalGridX(0.0),
  m_LocalGridY(0.0),
  m_Depth_m(0.0),
  m_LastPingTime_sec(0.0),
  m_ReferenceGeodesy(RefGeodesy)
{
	SetLocation(Lat, Long, Depth);
}



//=============================================================================
LblBeacon::~LblBeacon()
{
	// TODO Auto-generated destructor stub
}




//=============================================================================
void LblBeacon::SetLocation(double Latitude, double Longitude, double Depth)
{
	m_Latitude = Latitude;
	m_Longitude = Longitude;
	m_Depth_m = Depth;

	m_ReferenceGeodesy.LatLong2LocalGrid(	Latitude, Longitude,
										m_LocalGridY, m_LocalGridX );
}



//=============================================================================
void LblBeacon::GetLocation( double& Latitude, double& Longitude, double& Depth ) const
{
	Latitude = m_Latitude;
	Longitude = m_Longitude;
	Depth = m_Depth_m;
}



//=============================================================================
void LblBeacon::SetLocation_LocalGrid(double X, double Y, double Depth)
{
	m_LocalGridX = X;
	m_LocalGridY = Y;
	m_Depth_m = Depth;	// Set depth in meters

	// Calculate latitude/longitude
	m_ReferenceGeodesy.LocalGrid2LatLong(X, Y, m_Latitude, m_Longitude);
}



//=============================================================================
void LblBeacon::GetLocation_LocalGrid( double& X, double& Y, double& Depth) const
{
	X = m_LocalGridX;
	Y = m_LocalGridY;
	Depth = m_Depth_m;
}



//=============================================================================
float LblBeacon::GetDistance_LocalGrid( double RefX, double RefY, double RefDepth ) const
{
	float Dx, Dy, Dz;

	Dx = RefX - m_LocalGridX;
	Dy = RefY - m_LocalGridY;
	Dz = RefDepth - m_Depth_m;

	return sqrt( (Dx * Dx) + (Dy * Dy) + (Dz * Dz) );
}



//=============================================================================
float LblBeacon::GetDistance( double RefLatitude, double RefLongitude,
							  double RefDepth ) const
{
	double X, Y;

	m_ReferenceGeodesy.LatLong2LocalGrid(RefLatitude, RefLongitude, Y, X);
	return GetDistance_LocalGrid(X, Y, RefDepth);
}



//=============================================================================
LblBeacon& LblBeacon::operator=(LblBeacon& rhs)
{
    if ( &rhs != this )
    {
        this->m_Depth_m = rhs.m_Depth_m;
        this->m_LastPingTime_sec = rhs.m_LastPingTime_sec;
        this->m_Latitude = rhs.m_Latitude;
        this->m_Longitude = rhs.m_Longitude;
        this->m_LocalGridX = rhs.m_LocalGridX;
        this->m_LocalGridY = rhs.m_LocalGridY;
        this->m_ReferenceGeodesy = rhs.m_ReferenceGeodesy;
    }
    return *this;
}

}   // END namespace YellowSubNav
