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
/** @file LblPositionEstimator.cpp

@brief
	Implementation of the LblPositionEstimator class
	
@author Dave Billin

*/
//=============================================================================

#include "MOOS/libMOOS/Utils/MOOSFileReader.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include <math.h>
#include "LblPositionEstimator.h"

#define PARM_STR_LEN 128
#define NULL_CH      0


//static int whoi_acomm_params[17] = { 255, 10, 100, 2, 150, 50, 50, 4, 250,
//												0, 1, 0, 0, 1, 0, 16, 1500 };

enum
{ 
	COMMS_CYCLE = 15, 
	PING_TIMEOUT 
};


namespace YellowSubNav
{

float LblPositionEstimator::sm_SoundVelocity_mps = DEFAULT_H20_SOUND_VELOCITY_mps;


//=============================================================================
/* Creates an instance of the LblPositionEstimator class
@param Geodesy
	A reference to the vehicle's geodesy.  Used to generate local X and Y
	coordinates for the vehicle position
*/
LblPositionEstimator::LblPositionEstimator( CMOOSGeodesy& Geodesy )
:  m_CurrentDepth_m(0.0),
   m_NumActiveBeacons(0),
   m_ActiveBeaconFlags(0),
   m_Geodesy(Geodesy),
   m_ConfidenceFactor(0.0),
   B(4,4), Binv(4,4), Z(4,4)
{
	LblBeacon b(Geodesy);

	// Initialize LBL beacons
	m_LblBeacon.reserve(NUM_LBL_BEACONS);
	for (int i = 0; i < NUM_LBL_BEACONS; i++)
	{
		m_LblBeacon[i] = b;
	}

	// Initialize matrices
	B = 0.0;
	Binv = 0.0;
	Z = 0.0;
}



//=============================================================================
LblPositionEstimator::~LblPositionEstimator()
{

}



//=============================================================================
void LblPositionEstimator::UpdateBeaconRanges( float NavPingTravelTimes_sec[NUM_LBL_BEACONS],
											   double CurrentX, double CurrentY,
											   double Depth_m )
{
	uint ActiveBeaconFlags = m_ActiveBeaconFlags;
	LocalGridLocation EstPositionA, EstPositionB;	// Position estimates
	LblBeacon* ActiveBeacon[4] = { NULL, NULL, NULL, NULL };	// Pointers to active beacons
	//LblBeacon* InactiveBeacon[4] = { NULL, NULL, NULL, NULL };	// Pointers to missing beacons
	bool GotEstimate = false;
	int ActiveBeaconCount = 0;
	//int InactiveBeaconCount = 0;

	m_ActiveBeaconFlags = 0;

	for (int i = 0; i < NUM_LBL_BEACONS; i++)
	{
		m_LblBeacon[i].SetLastPingTime(NavPingTravelTimes_sec[i]);
		if (NavPingTravelTimes_sec[i] > 0.0f)
		{
			m_ActiveBeaconFlags |= (1<<i);	// Flag beacon as active

			// Add the beacon to the active list and increment active beacon count
			ActiveBeacon[ActiveBeaconCount++] = &m_LblBeacon[i];
		}
		else
		{
			// Add the beacon to the inactive list and increment active beacon count
			//InactiveBeacon[InactiveBeaconCount++] = &m_LblBeacon[i];
		}
	}

	m_NumActiveBeacons = ActiveBeaconCount;
	m_CurrentDepth_m = Depth_m;	// Store the current depth

	if (m_NumActiveBeacons == 2)
	{
		if ( EstimatePosition_2Beacon( *ActiveBeacon[0], *ActiveBeacon[1],
								  	   EstPositionA, EstPositionB ) )
		{
			// EstimatePosition_2Beacon returns two possible position estimates.
			// Determine which position is closer to the position where the
			// travel times were measured
			double NormSqA, NormSqB, fTemp;

			fTemp = CurrentX - EstPositionA.X;
			NormSqA = fTemp * fTemp;
			fTemp = CurrentY - EstPositionA.Y;
			NormSqA += fTemp * fTemp;

			fTemp = CurrentX - EstPositionB.X;
			NormSqB = fTemp * fTemp;
			fTemp = CurrentY - EstPositionB.Y;
			NormSqB += fTemp * fTemp;

			LocalGridLocation& Solution = (NormSqA < NormSqB) ? EstPositionA : EstPositionB;

			m_EstimatedPosition.X = Solution.X;
			m_EstimatedPosition.Y = Solution.Y;
			m_EstimatedPosition.Depth = Solution.Depth;

			m_ConfidenceFactor = 0.8;		// 2-beacon solution yields an 80% confidence
			GotEstimate = true;
		}
	}
	else if (m_NumActiveBeacons == 3)
	{
		// Determine which beacon was not received
		ActiveBeaconFlags = ~ActiveBeaconFlags;
		uint MissingBeaconIndex = 0;
		for (int i = 0; i < NUM_LBL_BEACONS; i++)
		{
			if (ActiveBeaconFlags & 0x01)
			{
				MissingBeaconIndex = i;
			}
		}

		// Calculate 3-beacon position solution
		if ( EstimatePosition_3Beacon( *ActiveBeacon[0], *ActiveBeacon[1], *ActiveBeacon[2],
									   MissingBeaconIndex, m_EstimatedPosition) )
		{
			m_ConfidenceFactor = 1.0;
			GotEstimate = true;
		}

	}

	// If we failed to estimate a position, decrement the confidence
	// factor by 0.2 (saturate at 0.0)
	if ( !GotEstimate )
	{
		m_ConfidenceFactor -= 0.2;
		m_ConfidenceFactor = (m_ConfidenceFactor < 0.0) ? 0.0 : m_ConfidenceFactor;
	}


}




//=============================================================================
/* Calculates the estimated position of the vehicle based on beacon
	distances derived from navigation ping travel times

@postcondition
	If a new position was successfully calculated, or if position hasn't
	changed, it will be updated in the estimator, and can be obtained by
	calling GetPosition().  If a new position could not be calculated, the
	previous position will be left unaltered with a reduced quality factor.
*/
void LblPositionEstimator::CalculatePosition( void )
{
	uint ActiveBeaconFlags = m_ActiveBeaconFlags;
	LocalGridLocation EstPositionA, EstPositionB;	// Position estimates
	bool GotEstimate = false;
	LblBeacon* ActiveBeacon[4] = { NULL, NULL, NULL, NULL };
	int j = 0;

	// Assemble the active beacons
	for (int i = 0; i < NUM_LBL_BEACONS; i++)
	{
		if (ActiveBeaconFlags & 0x01)
		{
			ActiveBeacon[j++] = &m_LblBeacon[i];
		}
	}
	if (m_NumActiveBeacons == 2)
	{
		if ( EstimatePosition_2Beacon( *ActiveBeacon[0], *ActiveBeacon[1],
								  	   EstPositionA, EstPositionB ) )
		{
			// Select the solution closest to the current estimated position
			double NormSqA, NormSqB, fTemp;

			fTemp = m_EstimatedPosition.X - EstPositionA.X;
			NormSqA = fTemp * fTemp;
			fTemp = m_EstimatedPosition.Y - EstPositionA.Y;
			NormSqA += fTemp * fTemp;

			fTemp = m_EstimatedPosition.X - EstPositionB.X;
			NormSqB = fTemp * fTemp;
			fTemp = m_EstimatedPosition.Y - EstPositionB.Y;
			NormSqB += fTemp * fTemp;

			LocalGridLocation& Solution = (NormSqA < NormSqB) ? EstPositionA : EstPositionB;

			m_EstimatedPosition.X = Solution.X;
			m_EstimatedPosition.Y = Solution.Y;
			m_EstimatedPosition.Depth = Solution.Depth;

			m_ConfidenceFactor = 0.8;
			GotEstimate = true;
		}
	}
	else if (m_NumActiveBeacons == 3)
	{
		// Find the missing beacon
		ActiveBeaconFlags = ~ActiveBeaconFlags;
		uint MissingBeaconIndex = 0;
		for (int i = 0; i < NUM_LBL_BEACONS; i++)
		{
			if (ActiveBeaconFlags & 0x01)
			{
				MissingBeaconIndex = i;
			}
		}

		// Calculate 3-beacon position solution
		if ( EstimatePosition_3Beacon( *ActiveBeacon[0], *ActiveBeacon[1], *ActiveBeacon[2],
									   MissingBeaconIndex, m_EstimatedPosition) )
		{
			m_ConfidenceFactor = 1.0;
			GotEstimate = true;
		}

	}

	// If we failed to estimate a position, decrement the confidence
	// factor by 0.2 (saturate at 0.0)
	if ( !GotEstimate )
	{
		m_ConfidenceFactor -= 0.2;
		m_ConfidenceFactor = (m_ConfidenceFactor < 0.0) ? 0.0 : m_ConfidenceFactor;
	}

}




//=============================================================================
bool LblPositionEstimator::EstimatePosition_2Beacon( LblBeacon& Beacon1, LblBeacon& Beacon2,
												     LocalGridLocation& EstPositionA,
												     LocalGridLocation& EstPositionB )
{
	float r1, r2, P1x, P1y, P1z, P2x, P2y, P2z;
	float A, a, b, c, h;
	//float ya, yb, xa, xb;
	float z;
	LocalGridLocation Beacon1Location, Beacon2Location;

	// Set up variables
	r1 = Beacon1.GetLastPingTime() * sm_SoundVelocity_mps;
	r2 = Beacon2.GetLastPingTime() * sm_SoundVelocity_mps;
	Beacon1.GetLocation_LocalGrid(Beacon1Location.X, Beacon1Location.Y, Beacon1Location.Depth);
	Beacon2.GetLocation_LocalGrid(Beacon2Location.X, Beacon2Location.Y, Beacon2Location.Depth);
	P1x = (float)Beacon1Location.X;
	P1y = (float)Beacon1Location.Y;
	P1z = (float)Beacon1Location.Depth;
	P2x = (float)Beacon2Location.X;
	P2y = (float)Beacon2Location.Y;
	P2z = (float)Beacon2Location.Depth;
	z = m_CurrentDepth_m;

	// Find 'Vector' from Buoy 1 to Buoy 2
	P2x -= P1x;
	P2y -= P1y;
	P2z -= P1z;

	// Find difference in Depth between sub and buoy 1;
	z   -= P1z;

	// Define Quadratic Constants a,b,c
	a = P2x*P2x + P2y*P2y;
	A = (r1*r1 + a - r2*r2 + P2z*P2z - 2.0*P2z*z);  // constant from 3bouy solution
	b = -A*P2y;
	c = A*A/4.0 + (z*z - r1*r1)*P2x*P2x;

	// h is the descriminant of the quadratic equation. negative gives error!!!!
	h = b*b - 4.0*a*c;
	if (h >=0)
	{
		h = sqrt( h );
	}
	else
	{
		return false;
	}

	// solve for two possible solutions (xa,ya) and (xb,yb)
	EstPositionA.Y = ( -b + h )/( 2.0*a );
	EstPositionB.Y = ( -b - h )/( 2.0*a );
	EstPositionA.X = ( A/2.0 - P2y * EstPositionA.Y )/( P2x );
	EstPositionB.X = ( A/2.0 - P2y * EstPositionB.Y )/( P2x );

	// Offset back to global coordinates
	EstPositionA.Y += P1y;
	EstPositionB.Y += P1y;
	EstPositionA.X += P1x;
	EstPositionB.X += P1x;

	// Find the solution closest to xest and yest
/*	a = ( xest - xa )*( xest - xa ) + ( yest - ya )*( yest - ya );
	b = ( xest - xb )*( xest - xb ) + ( yest - yb )*( yest - yb );

	if ( a < b )
	{
	*x = xa;
	*y = ya;
	}
	else
	{
	*x = xb;
	*y = yb;
	}
*/
	return true;
}






//=============================================================================
/* Three-buoy position solution derived from WHOI paper Integrated Acoustic
   Communication and Navigation for Multiple UUVs
*/
int LblPositionEstimator::EstimatePosition_3Beacon( LblBeacon& Beacon1, LblBeacon& Beacon2,
										  	  	    LblBeacon& Beacon3, uint MissingBeaconIndex,
										  	  	    LocalGridLocation& EstimatedPosition )
{
	float r1, r2, r3, a1, a2, ftemp1, ftemp2;
	double MissingBeaconX, MissingBeaconY, MissingBeaconDepth;
	uint index;

	// Prep variables
	r1 = Beacon1.GetLastPingTime() * sm_SoundVelocity_mps;
	r2 = Beacon2.GetLastPingTime() * sm_SoundVelocity_mps;
	r3 = Beacon3.GetLastPingTime() * sm_SoundVelocity_mps;
	index = MissingBeaconIndex;
	m_LblBeacon[index].GetLocation_LocalGrid(MissingBeaconX, MissingBeaconY, MissingBeaconDepth);

	ftemp1 = B(index,0);
	ftemp2 = B(index,1);
	a1 = r1*r1 + ftemp1 * ftemp1 + ftemp2 * ftemp2 - r2*r2;

	// Added to include Depth
	a1 += Z(index,1) - 2.0 * (EstimatedPosition.Depth - MissingBeaconDepth) * Z(index,0);

	ftemp1 = B(index,2);	// B(index,2) squared
	ftemp2 = B(index,3);	// B(index,2) squared
	a2 = r1*r1 + ftemp1 + ftemp2 - r3*r3;

	// Added to include Depth
	a2 += Z(index,3) - 2.0 * (EstimatedPosition.Depth - MissingBeaconDepth) * Z(index,2);

	m_EstimatedPosition.X = (Binv(index,0) * a1 + Binv(index,1) * a2)/2 + MissingBeaconX;

	m_EstimatedPosition.Y = (Binv(index,2) * a1 + Binv(index,3) * a2)/2
							 + MissingBeaconY;

	return 1;
}





//=========================================================================
double LblPositionEstimator::GetLocation_LocalGrid( double& X, double& Y,
													double Depth_m ) const
{
	X = m_EstimatedPosition.X;
	Y = m_EstimatedPosition.Y;
	Depth_m = m_EstimatedPosition.Depth;

	return m_ConfidenceFactor;
}




//=========================================================================
double LblPositionEstimator::GetLocation( double& Latitude,
										  double& Longitude,
										  double Depth_m ) const
{
	m_Geodesy.LocalGrid2LatLong( m_EstimatedPosition.X,
								 m_EstimatedPosition.Y,
								 Latitude, Longitude );

	Depth_m = m_EstimatedPosition.Depth;

	return m_ConfidenceFactor;
}



//=========================================================================
bool LblPositionEstimator::SetNavBeaconLocation( char BeaconID, double Latitude,
											  	 double Longitude, double Depth_m )
{
	unsigned int Index = BeaconID - 'A';

	if (Index < NUM_LBL_BEACONS)
	{
		m_LblBeacon[Index].SetLocation(Latitude, Longitude, Depth_m);
		return true;
	}
	else
	{
		return false;
	}
}




//=========================================================================
bool LblPositionEstimator::SetNavBeaconLocation_LocalGrid( char BeaconID,
														   double X, double Y,
														   double Depth_m )
{
	unsigned int Index = BeaconID - 'A';

	if (Index < NUM_LBL_BEACONS)
	{
		m_LblBeacon[Index].SetLocation_LocalGrid(X, Y, Depth_m);
		return true;
	}
	else
	{
		return false;
	}
}




//=============================================================================
const LblBeacon* LblPositionEstimator::GetNavBeacon( char BeaconId ) const
{
	unsigned int Index = BeaconId - 'A';

	if (Index < NUM_LBL_BEACONS)
	{
		return &m_LblBeacon[Index];
	}
	else
	{
		return NULL;
	}
}




//=============================================================================
void LblPositionEstimator::InitRange(void)
{
	double det;	// Determinant used to calculate inverse of B matrix

	/*
	--------------------------------------------------------------------
	Contents of the 'B matrix'
		[ (Bx - Ax)   (By - Ay)   (Cx - Ax)   (Cy - Ay)  ]
		| (Cx - Bx)   (Cy - By)   (Dx - Bx)   (Dy - By)  |
		| (Dx - Cx)   (Dy - Cy)   (Ax - Cx)   (Ay - Cy)  |
		[ (Ax - Dx)   (Ay - Dy)   (Bx - Dx)   (By - Dy)  ]
	
		Elements are listed as "Jk" where
			J = Beacon ID (A, B, C, or D)
			k = 'x' for x-coordinate; 'y' for y-coordinate
	--------------------------------------------------------------------
	*/
	

	//for(x = 0; x < NUM_LBL_BEACONS; x++)
	for (int r = 0; r < NUM_LBL_BEACONS; r++)
	{
		// Calculate the B matrix
		B(r,0) = m_LblBeacon[(r+1)%4].GetX() - m_LblBeacon[r].GetX();
		B(r,1) = m_LblBeacon[(r+1)%4].GetY() - m_LblBeacon[r].GetY();
		B(r,2) = m_LblBeacon[(r+2)%4].GetX() - m_LblBeacon[r].GetX();
		B(r,2) = m_LblBeacon[(r+2)%4].GetY() - m_LblBeacon[r].GetY();

		//-----------------------------------------
		// Calculate inverse of B matrix

		// Calculate determinant
		det = (B(r,0) * B(r,3)) - (B(r,1) * B(r,2));

		if (det != 0.0)
		{
			Binv(r,0) = B(r,3) / det;
			Binv(r,1) = -B(r,1) / det;
			Binv(r,2) = -B(r,2) / det;
			Binv(r,3) = B(r,0) / det;
		}


		//-------------------------------
		// Calculate the Z matrix
		// DB: Offset of 1 micrometer added to handle case where depth is zero
		Z(r,0) = m_LblBeacon[(r+1)%4].GetDepth() - m_LblBeacon[r].GetDepth() + 0.000001;
		Z(r,1) = Z(r,0) * Z(r,0);
		Z(r,2) = m_LblBeacon[(r+2)%4].GetDepth() - m_LblBeacon[r].GetDepth() + 0.000001;
		Z(r,3) = Z(r,2) * Z(r,2);
	}
}








//=============================================================================
/* Called from within whoi_parse
PARAM sentence
	Pointer to the first travel time field in an SNTTA sentence from the modem
	
RETURNS
	1 on success; 0 on failure
*/
int LblPositionEstimator::compute_range(char *sentence)
{
    /*
    float r1,r2,r3,r4;
	int char_cnt;
	char parm_str[PARM_STR_LEN];
	int error;
	int buoy_flag;
	float xa, ya, xb, yb;


	//--------------------------------------------------------
	// Extract the "TA" (Transponder A travel time) field from 
	// the SNTTA sentence received from the modem
	// Bit 0 in the variable error is set on failure to 
	// extract this value
	//--------------------------------------------------------
	char_cnt = strcpy_dlim(parm_str, sentence,',',PARM_STR_LEN);
	error = 0;
	if( *parm_str )	// Make sure transponder time string isn't empty
	{
		r1 = (float)atof(parm_str);	// Get transponder A travel time (in seconds) as a float
		
		// Verify that a value was extracted from parm_str
		// If atof() failed, the global variable _xtoxErr is set to 1
		//if (_xtoxErr == 1)
		//{
		//	error |= 1;
		//}
		
		// Verify that the transponder travel time is less than the REMUS ping timeout
		// set in the modem NVRAM parameters
		if ( (r1 < 0.0000) || 
			 (r1 > ((float)whoi_acomm_params[PING_TIMEOUT]/1000.0) )
		{
			error |= 1;
		}
	} 
	else 
	{
		error |= 1;
	}
	
	//--------------------------------------------------------
	// Convert transponder B travel time to a distance 
	// in meters [sec]*[m/sec] = [m]
	//--------------------------------------------------------
	r1 *= SOUND_SPEED_WATER;	
	sentence += char_cnt;	// Increment to next transponder time field in SNTTA sentence


	
	
	//--------------------------------------------------------
	// Extract the "TB" (Transponder B travel time) field from 
	// the SNTTA sentence received from the modem
	// Bit 1 in the variable error is set on failure to 
	// extract this value
	//--------------------------------------------------------
	char_cnt = strcpy_dlim(parm_str, sentence,',',PARM_STR_LEN);
	if(*parm_str)
	{
		r2 = atof(parm_str);	// Get float value from string
		//if (_xtoxErr)
		//{
		//	error |= 2;
		//}

		if (r2 < 0.0000 || r2 > ((float)whoi_acomm_params[PING_TIMEOUT]/1000.0))
		{
			error |= 2;
		}
	} 
	else
	{
		error |= 2;
	}
	
	r2 *= SOUND_SPEED_WATER;	// Convert travel time to distance in meters
   sentence += char_cnt;	// Increment to next transponder time field in SNTTA sentence

   
	//--------------------------------------------------------
	// Extract the "TC" (Transponder C travel time) field from 
	// the SNTTA sentence received from the modem
	// Bit 2 in the variable error is set on failure to 
	// extract this value
	//--------------------------------------------------------
	char_cnt = strcpy_dlim(parm_str, sentence,',',PARM_STR_LEN);
	if(*parm_str)
	{
		r3 = atof(parm_str);
		//if (_xtoxErr)
		//{
		//	error |= 4;
		//}
			
		if (r3 < 0.0000 || r3 > ((float)whoi_acomm_params[PING_TIMEOUT]/1000.0))
		{
			error |= 4;
		}
	} 
	else 
	{
		error |= 4;
	}
	r3 *= SOUND_SPEED_WATER;	// Convert travel time to distance in meters
	sentence += char_cnt;	// Increment to next transponder time field in SNTTA sentence
	
	
	//--------------------------------------------------------
	// Extract the "TD" (Transponder D travel time) field from 
	// the SNTTA sentence received from the modem
	// Bit 3 in the variable error is set on failure to 
	// extract this value
	//--------------------------------------------------------
	char_cnt = strcpy_dlim(parm_str, sentence,',',PARM_STR_LEN);
	if(*parm_str)
	{
		r4 = atof(parm_str);
		//if(_xtoxErr)
		//{
		//	error |= 8;
		//}
		if(r4 < 0.0000 || r4 > ((float)whoi_acomm_params[PING_TIMEOUT]/1000.0))
		{
			error |= 8;
		}
	} 
	else 
	{
		error |= 8;
	}
	
	r4 *= SOUND_SPEED_WATER;		// Convert travel time to distance in meters
	sentence += char_cnt;	// Increment to next transponder time field in SNTTA sentence

	// Count remaining characters before NMEA checksum
	char_cnt = strcpy_dlim(parm_str, sentence,'*',PARM_STR_LEN);

//   nav_pos_time = (long)atol(parm_str);
*/
	pos_best_buoys = 0;

/*
	// Handle SNTTA messages missing one or more transponder travel times
	for( buoy_flag = 1; buoy_flag <= 12; buoy_flag++ )
	{
		if( ( buoy_flag | error ) == buoy_flag )
		{
	      switch( buoy_flag )
	      {
	         case 1:
				//we are missing transponder A
				// Calculate exact position from B,C,D distances
				exact_position(r2, r3, r4, &xa, &ya, last_sensor_packet.depth/100.0,1);
				update_calc_pos( xa, ya, (15-error) );
				break;

	         case 2:
				//we are missing transponder B
				// Calculate exact position from A,C,D distances
				// Note: the order of the arguments to exact_position() is C,D,A
	            exact_position( r3, r4,r1, &xa, &ya, last_sensor_packet.depth/100.0,2);
	            update_calc_pos( xa, ya, (15-error) );
	            break;

	         case 3:
	            //we are missing transponders A and B
				// Estimate position based on C and D distances
	            if( est_position(r3, r4,
								 LBL.buoy[BUOYC].xloc,
								 LBL.buoy[BUOYC].yloc,
								 LBL.buoy[BUOYC].zloc,
								 LBL.buoy[BUOYD].xloc,
								 LBL.buoy[BUOYD].yloc,
								 LBL.buoy[BUOYD].zloc,
								 &xa, &ya, &xb, &yb, 
								 last_sensor_packet.depth/100.0) );
	            {
	               update_calc_pos( xa, ya, (15-error) );
	               update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         case 4:
				//we are missing transponder C
				// Calculate exact position from A,B,D distances
				// Note: the order of the arguments to exact_position() is D,A,B
	            exact_position(r4,r1, r2, &xa, &ya, last_sensor_packet.depth/100.0,3);
	            break;

	         case 5:
	            //we are missing transponders A and C
				// Estimate position based on B and D distances
	            if( est_position(r2, r4,
	            LBL.buoy[BUOYB].xloc,
	            LBL.buoy[BUOYB].yloc,
	            LBL.buoy[BUOYB].zloc,
	            LBL.buoy[BUOYD].xloc,
	            LBL.buoy[BUOYD].yloc,
	            LBL.buoy[BUOYD].zloc,
	            &xa, &ya, &xb, &yb, last_sensor_packet.depth/100.0) );
	            {
	               update_calc_pos( xa, ya, (15-error) );
	               update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         case 6:
	            //we are missing transponders B and C
				// Estimate position based on A and D distances
	            if( est_position(r1, r4,
	            LBL.buoy[BUOYA].xloc,
	            LBL.buoy[BUOYA].yloc,
	            LBL.buoy[BUOYA].zloc,
	            LBL.buoy[BUOYD].xloc,
	            LBL.buoy[BUOYD].yloc,
	            LBL.buoy[BUOYD].zloc,
	            &xa, &ya, &xb, &yb, last_sensor_packet.depth/100.0) );
	            {
	               update_calc_pos( xa, ya, (15-error) );
	               update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         case 8:
				//we are missing Transponder D
				// Calculate exact position from A,B,C distances
				// Note: the order of the arguments to exact_position() is A,B,C
	            exact_position(r1, r2, r3, &xa, &ya, last_sensor_packet.depth/100.0,0);
	            update_calc_pos( xa, ya, (15-error) );
	            break;

	         case 9:
	            //we are missing transponders A and D
				// Estimate position based on B and C distances
	            if( est_position(r2, r3,
	            LBL.buoy[BUOYB].xloc,
	            LBL.buoy[BUOYB].yloc,
	            LBL.buoy[BUOYB].zloc,
	            LBL.buoy[BUOYC].xloc,
	            LBL.buoy[BUOYC].yloc,
	            LBL.buoy[BUOYC].zloc,
	            &xa, &ya, &xb, &yb, last_sensor_packet.depth/100.0) );
	            {
					update_calc_pos( xa, ya, (15-error) );
					update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         case 10:
	            //we are missing transponders B and D
				// Estimate position based on A and C distances
	            if( est_position(r1, r3,
	            LBL.buoy[BUOYA].xloc,
	            LBL.buoy[BUOYA].yloc,
	            LBL.buoy[BUOYA].zloc,
	            LBL.buoy[BUOYC].xloc,
	            LBL.buoy[BUOYC].yloc,
	            LBL.buoy[BUOYC].zloc,
	            &xa, &ya, &xb, &yb, last_sensor_packet.depth/100.0) );
	            {
	               update_calc_pos( xa, ya, (15-error) );
	               update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         case 12:
	            //we are missing the third and the fourth buoy
				// Estimate position based on A and B distances
	            if( est_position(r1, r2,
	            LBL.buoy[BUOYA].xloc,
	            LBL.buoy[BUOYA].yloc,
	            LBL.buoy[BUOYA].zloc,
	            LBL.buoy[BUOYB].xloc,
	            LBL.buoy[BUOYB].yloc,
	            LBL.buoy[BUOYB].zloc,
	            &xa, &ya, &xb, &yb, last_sensor_packet.depth/100.0) );
	            {
					update_calc_pos( xa, ya, (15-error) );
					update_calc_pos( xb, yb, (15-error) );
	            }
	            break;

	         default:
	            break;
	      }
		}
	}

*/
	// DB: pos_best_buoys gets updated in update_calc_pos()
	return pos_best_buoys;
}








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
void LblPositionEstimator::update_calc_pos( float x, float y, int buoys )
{
/*	float x_diff, y_diff, pos_diff_sq;

	// Calculate the error between the new x/y position and the
	// vehicle's estimated position
	x_diff = x - est_pos_x;
	y_diff = y - est_pos_y;

	// Calculate the magnitude squared of the error vector
	pos_diff_sq = (x_diff * x_diff) + (y_diff * y_diff);	// x^2 + y^2 = d^2

	// If the error magnitude squared is less than the 'best' error magnitude
	// calculated so far, update the variables used to track the 'best' position
	// values for the next calculation.
	if( pos_diff_sq < pos_best_diff )
	{
		pos_best_x = x_diff;
		pos_best_y = y_diff;
		pos_best_diff = pos_diff_sq;
		pos_best_buoys = buoys;
	}
*/
}








//=============================================================================
/*	Two-buoy position solution derived from the Woods-Hole Oceanographic
	Institute paper "Integrated Acoustic Communication and Navigation for
	Multiple UUVs"

PARAMETERS:
r1	- Distance to first transponder used to estimate position
r2	- Distance to second transponder used to estimate position
P1x	- known x position of the first transponder used to estimate position
P1y	- known y position of the first transponder used to estimate position
P1z	- known z (depth) position of the first transponder used to estimate position
P2x	- x-position of the second transponder used to estimate position
P2y	- y-position of the second transponder used to estimate position
P2z	- known z (depth) position of a second transponder used to estimate position

xap	-
yap	-
xbp	-
ybp	-
z	-
*/
/*int LblPositionEstimator::EstimatePosition_2Beacon( float r1, float r2,
													float P1x, float P1y, float P1z,
													float P2x, float P2y, float P2z,
													float *xap, float *yap,
													float *xbp, float *ybp,
													float z)
{
	float A,a,b,c,h;

	// Find 'Vector' from Buoy 1 to Buoy 2
	P2x -= P1x;
	P2y -= P1y;
	P2z -= P1z;

	// Find difference in Depth between sub and buoy 1;
	z   -= P1z;

	// Define Quadratic Constants a,b,c
	a = P2x*P2x + P2y*P2y;
	A = (r1*r1 + a - r2*r2 + P2z*P2z - 2.0f*P2z*z);	// constant from 3bouy solution
	b = -A*P2y;
	c = A*A/4.0f + (z*z - r1*r1)*P2x*P2x;

   // h is the descriminant of the quadratic equation. negative gives error!!!!
   h = b*b - 4.0f*a*c;
   if (h >=0)
	   h = sqrt( h );
   else
   	return 0;

   // solve for two possible solutions (xa,ya) and (xb,yb)
   *yap = ( -b + h )/( 2.0f*a );
   *ybp = ( -b - h )/( 2.0f*a );
   *xap = ( A/2.0f - P2y*(*yap) )/( P2x );
   *xbp = ( A/2.0f - P2y*(*ybp) )/( P2x );

   // move to Global coordinates
   *yap += P1y;
   *xap += P1x;
   *xbp += P1x;

	return 1;
}
*/













//=============================================================================
void LblPositionEstimator::init_LBL()
{
    /*
    int readResult;

#ifdef WRITE_INIT_BLOCK
	LBL.buoy[BUOYA].xloc = DEFAULT_PBAX;
	LBL.buoy[BUOYA].yloc = DEFAULT_PBAY;
	LBL.buoy[BUOYA].zloc = DEFAULT_PBAZ;

  	LBL.buoy[BUOYB].xloc = DEFAULT_PBBX;
	LBL.buoy[BUOYB].yloc = DEFAULT_PBBY;
	LBL.buoy[BUOYB].zloc = DEFAULT_PBBZ;

	LBL.buoy[BUOYC].xloc = DEFAULT_PBCX;
	LBL.buoy[BUOYC].yloc = DEFAULT_PBCY;
	LBL.buoy[BUOYC].zloc = DEFAULT_PBCZ;

	LBL.buoy[BUOYD].xloc = DEFAULT_PBDX;
	LBL.buoy[BUOYD].yloc = DEFAULT_PBDY;
	LBL.buoy[BUOYD].zloc = DEFAULT_PBDZ;

	LBL.gps_zero_long = DEFAULT_LONG0;
	LBL.gps_zero_lat = DEFAULT_LAT0;

	LBL.buoy_state[BUOYA] = ON;
	LBL.buoy_state[BUOYB] = ON;
	LBL.buoy_state[BUOYC] = ON;
	LBL.buoy_state[BUOYD] = OFF;

   LBL.c_in_h20 = SOUND_SPEED_WATER;

	writeUserBlock(0,&LBL,SIZE_OF_LBL);
#endif
   readResult = 0;
	readResult = readUserBlock(&LBL,0,SIZE_OF_LBL);

#ifdef VERBOSE
if(readResult == 0)
	printf("LBL data initialized.\n");
#endif
*/
	return;
}





//=============================================================================
void LblPositionEstimator::true_init_pos()
{
/*
	pos_best_x = (float)((long)(last_sensor_packet.lat*10000000L) - LBL.gps_zero_lat) *
   					MPERDEG;
	pos_best_y = (float)((long)(last_sensor_packet.lon*10000000L) - LBL.gps_zero_long )
   					 * MPERDEG * cos((last_sensor_packet.lat) * PI / 180.0 );

	pos_best_buoys = 15;
	tm_rd(&tod);
//   nav_pos_time = (long)tod.tm_hour * 10000L +
//   					(long)tod.tm_min * 100L +
//                  (long)tod.tm_sec;
   //we've calculated the initial 'true' x and y and now we need to send to KIRK
	send_nav_packet();
*/
}


} // END namespace YellowSubNav
