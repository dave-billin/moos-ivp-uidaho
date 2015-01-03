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
/** @file VehicleEKF.cpp
 * @brief
 * Implementation of objects declared in VehicleEKF.h
 */
//=============================================================================
#include <iostream>
#include <iomanip>
#include "newmatio.h"

#include "VehicleEKF.h"
#include <math.h>
#include <string.h>
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"

using namespace std;
using namespace NEWMAT;
using namespace YellowSubNav;


//=========================
// MACROS
//=========================

// Implement the EKF_VERBOSE macro
#define _EKF_VERBOSE1(_expr_)	if (m_Verbosity >= 1) { _expr_ }
#define _EKF_VERBOSE2(_expr_)	if (m_Verbosity >= 2) { _expr_ }
#define _EKF_VERBOSE3(_expr_)	if (m_Verbosity >= 3) { _expr_ }

#define RADPERDEG (PI/180.0)

#ifdef USING_FLOAT
	#define Real(_num_) float(_num_)
#else
	#define Real(_num_) _num_
#endif

namespace YellowSubNav
{

const float VehicleEKF::SPEED_CHECK = 5.0;
const float VehicleEKF::JUMP_CHECK = 15.0;



//=============================================================================
VehicleEKF::VehicleEKF( CMOOSGeodesy& ReferenceGeodesy, uint32_t Verbosity )
:	EKF::EKF(NUM_STATES, NUM_NOISE_SOURCES),	/* Base class constructor builds EKF matrices */
 	m_ReferenceGeodesy(ReferenceGeodesy),
 	m_SoundVelocity(1),
 	m_LblPingPeriod(0),
 	m_BadJumpFlag(false),
    m_BadRangesFlag(false),
    m_NumRangesRejected(0),
    m_RangeRejectAbortTrigger(1),
    m_NavBeaconRangeBound(0.0),
    m_Last_ZGyro(0.0),
    m_LastUsed_ZGyro(0.0),
    m_Verbosity(Verbosity),
 	m_LblBeaconA(ReferenceGeodesy),
	m_LblBeaconB(ReferenceGeodesy),
	m_LblBeaconC(ReferenceGeodesy),
	m_LblBeaconD(ReferenceGeodesy)
{
	X = 0.0;	// Initialize Kalman state vector

	/* 	NOTE: Kalman matrices/vectors get initialized in the Initialize()
		method.  The m_EkfStatusFlags variable disables EKF evaluation until
		Initialize() is called and nav beacon locations have been configured.
	*/
}



//=============================================================================
VehicleEKF::~VehicleEKF()
{
	// Clean up any dynamically-allocated resources here!!!
}



//=============================================================================
void VehicleEKF::Initialize( Real CurrentE, Real CurrentN,
		 	 	 	  	  	 Real CurrentVelocity, Real CurrentHeading,
		 	 	 	  	  	 uint32_t LblBeaconPingPeriod_sec, double t )
{
	Real InitialValues[5];	// Initial state value array

	// DB: This function just wraps a call to the
	// base class Initialize() method...
	InitialValues[EST_COORD_E-1] = CurrentE;
	InitialValues[EST_COORD_N-1] = CurrentN;
	InitialValues[EST_SPEED-1] = CurrentVelocity;
	InitialValues[EST_YAW-1] = CurrentHeading;
	InitialValues[EST_YAW_BIAS-1] = 0.0;

	m_LblPingPeriod = LblBeaconPingPeriod_sec;

	// Call Initialize() method of the base class
	EKF::Initialize(InitialValues, t);
}



//=============================================================================
void VehicleEKF::ApplyImuMeasurement( const ImuMeasurement& ImuData )
{
	m_Last_ZGyro = ( (ImuData.m_wy * sin(ImuData.m_phi) +
					  ImuData.m_wz * cos(ImuData.m_phi) ) *
					  ( 1.0 / cos(ImuData.m_theta)) );

	m_Last_ZGyro = MOOSDeg2Rad(m_Last_ZGyro);

	/* The IMU reports measurements at 100 Hz.  Convert the raw readings
	   (in radians-per-second-per-sample) to radians */
	m_Last_ZGyro *=  100;
}



//=============================================================================
bool VehicleEKF::GetEstimatedCoordinates( Real& OUT_E, Real& OUT_N ) const
{
	if (IsInitialized())
	{
		OUT_E = X(EST_COORD_E);
		OUT_N = X(EST_COORD_N);
		return true;
	}
	else
	{
		return false;
	}
}



//=============================================================================
bool VehicleEKF::GetEstimatedVelocity( Real& OUT_Velocity ) const
{
	if ( this->IsInitialized() )
	{
		OUT_Velocity = X(EST_SPEED);
		return true;
	}
	else
	{
		return false;
	}
}



//=============================================================================
bool VehicleEKF::GetEstimatedHeading( Real& OUT_Heading ) const
{
	if ( this->IsInitialized() )
	{
		OUT_Heading = MOOSRad2Deg( X(EST_YAW) );
		return true;
	}
	else
	{
		return false;
	}
}


//=========================================================================
LblBeacon& VehicleEKF::NavBeacon( char BeaconId )
{
	switch (BeaconId)
	{
	case 'A':
		return m_LblBeaconA;
		break;

	case 'B':
		return m_LblBeaconB;
		break;

	case 'C':
		return m_LblBeaconC;
		break;

	case 'D':
		return m_LblBeaconD;
		break;

	default:
		throw CMOOSException("Illegal LBL beacon ID!");
	}

}




//=============================================================================
void VehicleEKF::DoInitializeEKF( Real* pInitialStateValues )
{
	/* Initialize the state vector to the last known GPS coordinates received
	 * before diving.  Initialize the error covariance matrix to ones on diagonal
	 * so the filter will run */

	float sigma_range, sigma_u, sigma_h, sigma_e_sys, sigma_n_sys, sigma_u_sys,
		  sigma_h_sys, sigma_b_sys, p_init_en, p_init_u, p_init_h, p_init_b;

	// EKF coefficients depend on the interval at which navigation beacon ranges arrive
	switch( m_LblPingPeriod )
	{
		case 0:  //"Normal" Synchronous Mission
		{
			// Measurement Check Cutoff
			// Ranges larger than this value will be rejected
			m_NavBeaconRangeBound = 10.0;

			//Measurement Noises (R values)
			sigma_range = 0.1289*0.1289;
			sigma_u     = 0.0212*0.0212;
			sigma_h     = 17.5128*RADPERDEG*17.5128*RADPERDEG;

			//System Noises (Q values)
			sigma_e_sys = 0.1269*0.1269;
			sigma_n_sys = 0.1269*0.1269;
			sigma_u_sys = 1.0490*1.0490;
			//
			sigma_h_sys = 0.4220*RADPERDEG*0.4220*RADPERDEG;
			sigma_b_sys = 0.0217*RADPERDEG*0.0217*RADPERDEG;

			//Initial P Values
			p_init_en = 20.4849;
			p_init_u  = 2.7525;
			p_init_h  = 17.4712;
			p_init_b  = 3.3400;

			//Number of Rejected Ranges to Cause Abort
			m_RangeRejectAbortTrigger = 4;
			break;
		}


		case 2:  //"normal" 2 second navigation
		{
			//Measurement Check Cutoff
			// Ranges larger than this value will be rejected
			m_NavBeaconRangeBound = 10.2570;

			//Measurement Noises (R values)
			sigma_range = 0.7051*0.7051;
			sigma_u     = 0.9856*0.9856;
			sigma_h     = 1.8855*RADPERDEG*1.8855*RADPERDEG;

			//System Noises (Q values)
			sigma_e_sys = 0.9723*0.9723;
			sigma_n_sys = 0.9723*0.9723;
			sigma_u_sys = 0.6819*0.6819;
			sigma_h_sys = 0.1586*RADPERDEG*0.1586*RADPERDEG;
			sigma_b_sys = 1.5667*RADPERDEG*1.5667*RADPERDEG;

			//Initial P Values
			p_init_en = 0.1;
			p_init_u  = 7.0;
			p_init_h  = 32.5;
			p_init_b  = 9.5;

			//Number of Rejected Ranges to Cause Abort
			m_RangeRejectAbortTrigger = 10;
			break;
		}

		case 3:  //MLBL Case
		{
			//Measurement Check Cutoff
			// Ranges larger than this value will be rejected
			m_NavBeaconRangeBound = 20;

			//Measurement Noises (R values)
			sigma_range = 1.0000;
			sigma_u     = 0.0101;
			sigma_h     = 0.0101;

			//System Noises (Q values)
			sigma_e_sys = 10.077;
			sigma_n_sys = 10.071;
			sigma_u_sys = 0.1007;
			sigma_h_sys = 0.1054;
			sigma_b_sys = 0.1008;

			//Initial P Values
			p_init_en = 10.1;
			p_init_u  = 10.1;
			p_init_h  = 10.1;
			p_init_b  = 10.1;

			//Number of Rejected Ranges to Cause Abort
			m_RangeRejectAbortTrigger = 10;
			break;
		}

		case 15: // "Fleet" or unrecognized ping period will run 15-sec EKF
		default:
		{
			//Measurement Check Cutoff
			// Ranges larger than this value will be rejected
			m_NavBeaconRangeBound = 18.9740;

			//Measurement Noises (R values)
			sigma_range = 0.4462*0.4462;
			sigma_u     = 5.8860*5.8860;
			sigma_h     = 0.3462*RADPERDEG*0.3462*RADPERDEG;

			//System Noises (Q values)
			sigma_e_sys = 1.0803*1.0803;
			sigma_n_sys = 1.0803*1.0803;
			sigma_u_sys = 0.1082*0.1082;
			sigma_h_sys = 1.1936*RADPERDEG*1.1936*RADPERDEG;
			sigma_b_sys = 0.5516*RADPERDEG*0.5516*RADPERDEG;

			//Initial P Values
			p_init_en = 0.3023;
			p_init_u  = 0.0838;
			p_init_h  = 1.5912;
			p_init_b  = 0.0085;

			//Number of Rejected Ranges to Cause Abort
			m_RangeRejectAbortTrigger = 4;
			break;
		}
	}


	//-------------------------
	//Set Initial State
	//-------------------------
	X(EST_COORD_E) = pInitialStateValues[EST_COORD_E-1];
	X(EST_COORD_N) = pInitialStateValues[EST_COORD_N-1];
	X(EST_SPEED) = pInitialStateValues[EST_SPEED-1];
	X(EST_YAW) = MOOSDeg2Rad( pInitialStateValues[EST_YAW-1] );
	X(EST_YAW_BIAS) = 0.0;		//Initial Compass Bias in radians


	//-------------------------
	// Initialize P matrix
	//-------------------------
	P = 0.0;
	P <<  p_init_en  << Real(0.0)  << Real(0.0) << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  << p_init_en  << Real(0.0) << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  << Real(0.0)  << p_init_u  << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  << Real(0.0)  << Real(0.0) << p_init_h  << Real(0.0)
	  <<  Real(0.0)  << Real(0.0)  << Real(0.0) << Real(0.0) << p_init_b;
	/* // Initialize P matrix
	P[0][0] = p_init_en;   	P[0][1] = 0.0;   		P[0][2] = 0.0;   	P[0][3] = 0.0;   	P[0][4] = 0.0;
	P[1][0] = 0.0;   		P[1][1] = p_init_en;   	P[1][2] = 0.0;   	P[1][3] = 0.0;   	P[1][4] = 0.0;
	P[2][0] = 0.0;   		P[2][1] = 0.0;   		P[2][2] = p_init_u; P[2][3] = 0.0;   	P[2][4] = 0.0;
	P[3][0] = 0.0;   		P[3][1] = 0.0;   		P[3][2] = 0.0;   	P[3][3] = p_init_h; P[3][4] = 0.0;
	P[4][0] = 0.0;   		P[4][1] = 0.0;   		P[4][2] = 0.0;   	P[4][3] = 0.0;   	P[4][4] = p_init_b;
	*/

	//-------------------------
	// Set up Q matrix
	//-------------------------
	Q = 0.0;
	Q << sigma_e_sys <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << Real(0.0)
	  <<  Real(0.0)  << sigma_n_sys <<  Real(0.0)  <<  Real(0.0)  << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  << sigma_u_sys <<  Real(0.0)  << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << sigma_h_sys << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << sigma_b_sys;
	/* // Setup Q matrix
	Q[0][0] = sigma_e_sys;	Q[0][1] = 0.0;				Q[0][2] = 0.0;			Q[0][3] = 0.0;			Q[0][4] = 0.0;
	Q[1][0] = 0.0;			Q[1][1] = sigma_n_sys;		Q[1][2] = 0.0;			Q[1][3] = 0.0;			Q[1][4] = 0.0;
	Q[2][0] = 0.0;			Q[2][1] = 0.0;				Q[2][2] = sigma_u_sys;	Q[2][3] = 0.0;			Q[2][4] = 0.0;
	Q[3][0] = 0.0;			Q[3][1] = 0.0;				Q[3][2] = 0.0;			Q[3][3] = sigma_h_sys;	Q[3][4] = 0.0;
	Q[4][0] = 0.0;			Q[4][1] = 0.0;				Q[4][2] = 0.0;			Q[4][3] = 0.0;			Q[4][4] =  sigma_b_sys;
	*/

	//-------------------------
	// Set up R matrix
	//-------------------------
	R = 0.0;
	R << sigma_range <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  << sigma_range <<  Real(0.0)  <<  Real(0.0)  << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  << sigma_range <<  Real(0.0)  << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << sigma_range << Real(0.0) << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << sigma_u   << Real(0.0)
	  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  <<  Real(0.0)  << Real(0.0) << sigma_h;
	/* // Setup R matrix
	R[0][0] = sigma_range;	R[0][1] = 0.0;			R[0][2] = 0.0;			R[0][3] = 0.0;			R[0][4] = 0.0;		R[0][5] = 0.0;
	R[1][0] = 0.0;			R[1][1] = sigma_range;	R[1][2] = 0.0;			R[1][3] = 0.0;			R[1][4] = 0.0; 		R[1][5] = 0.0;
	R[2][0] = 0.0;			R[2][1] = 0.0;			R[2][2] = sigma_range;	R[2][3] = 0.0;			R[2][4] = 0.0; 		R[2][5] = 0.0;
	R[3][0] = 0.0;			R[3][1] = 0.0;			R[3][2] = 0.0;			R[3][3] = sigma_range;	R[3][4] = 0.0; 		R[3][5] = 0.0;
	R[4][0] = 0.0;			R[4][1] = 0.0;			R[4][2] = 0.0;			R[4][3] = 0.0;			R[4][4] = sigma_u; 	R[4][5] = 0.0;
	R[5][0] = 0.0;			R[5][1] = 0.0;			R[5][2] = 0.0;			R[5][3] = 0.0;			R[5][4] = 0.0;		R[5][5] = sigma_h;
	*/

	m_NumRangesRejected = 0;
	m_BadRangesFlag = false;
	m_BadJumpFlag = false;
}






//=============================================================================
void VehicleEKF::PreUpdateMeasurement( const EKFMeasurement& Measurement,
							   	   	   bool& OUT_ShouldUpdateEKF )
{
	M = 0.0;	// Initialize the M matrix

	try
	{
		// Dispatch measurement to the appropriate handler function
		switch ( Measurement.MeasurementType() )
		{
		case MEAS_HEADING:	// Vehicle heading measurement
			OUT_ShouldUpdateEKF = PreUpdate_Heading(
								dynamic_cast<const HeadingMeasurement&>(Measurement) );
			break;

		case MEAS_SPEED:	// Vehicle speed measurement
			OUT_ShouldUpdateEKF = PreUpdate_Speed(
								dynamic_cast<const SpeedMeasurement&>(Measurement) );
			break;

		case MEAS_LBL_RANGES:	// Measured LBL ping time of travel
			OUT_ShouldUpdateEKF = PreUpdate_LblBeacons(
								dynamic_cast<const LblBeaconMeasurement&>(Measurement) );
			break;

		default:
			OUT_ShouldUpdateEKF = false;
			break;
		}
	}
	catch (std::exception& e)	// Handle dynamic_cast failure gracefully
	{
		OUT_ShouldUpdateEKF = false;
	}

	// NOTE: if OUT_ShouldUpdateEKF is set to true at this point, the generic
	// EKF update calculation will run as soon as this function returns.
}




//=============================================================================
void VehicleEKF::PostUpdateMeasurement( const EKFMeasurement& Measurement )
{
	Real JumpE, JumpN;

	// DB: changed bounding to be consistent with MOOS yaw convention (+/- PI)
	X(EST_YAW) = MOOS_ANGLE_WRAP( X(EST_YAW) );


    if( Measurement.MeasurementType() == MEAS_LBL_RANGES)
    {
    	JumpE = XEN[0] - X(EST_COORD_E);
    	JumpN = XEN[1] - X(EST_COORD_N);
		if( ((JumpE*JumpE) > (JUMP_CHECK*JUMP_CHECK)) ||
			((JumpN*JumpN) > (JUMP_CHECK*JUMP_CHECK)) )
		{
			m_BadJumpFlag = true;
			DeInitialize();		// Render the EKF uninitialized
		}
    }
}





//=============================================================================
void VehicleEKF::PrePropagate( double Delta_t )
{
	Real SinOfYaw = sin( X(EST_YAW) );
	Real CosOfYaw = cos( X(EST_YAW) );
	Real EstimatedVelocity = X(EST_SPEED);

	//-----------------------------
	// Set up F matrix
	//-----------------------------
	F = m_I;
	F(1,3) = Delta_t * SinOfYaw;
	F(1,4) = Delta_t * EstimatedVelocity * CosOfYaw;
	F(2,3) = Delta_t * CosOfYaw;
	F(2,4) = -Delta_t * EstimatedVelocity * SinOfYaw;

	//cout << setw(10) << setprecision(5) << F;

	/* // ENSPB
	F[0][0] = 1.0;	F[0][1] = 0.0;	F[0][2] = Delta_t*sin(X[3][0]);	F[0][3] = Delta_t*X[2][0]*cos(X[3][0]);		F[0][4] = 0.0;
	F[1][0] = 0.0;	F[1][1] = 1.0;	F[1][2] = Delta_t*cos(X[3][0]);	F[1][3] = -Delta_t*X[2][0]*sin(X[3][0]);	F[1][4] = 0.0;
	F[2][0] = 0.0;	F[2][1] = 0.0;	F[2][2] = 1.0;					F[2][3] = 0.0;								F[2][4] = 0.0;
	F[3][0] = 0.0;	F[3][1] = 0.0;	F[3][2] = 0.0;					F[3][3] = 1.0;								F[3][4] = 0.0;
	F[4][0] = 0.0;	F[4][1] = 0.0;	F[4][2] = 0.0;					F[4][3] = 0.0;								F[4][4] = 1.0;
	*/

	//-----------------------------
	// Calculate new states
	//-----------------------------
	X(EST_COORD_E) = X(EST_COORD_E) + EstimatedVelocity * Delta_t * SinOfYaw;
	X(EST_COORD_N) = X(EST_COORD_N) + EstimatedVelocity * Delta_t * CosOfYaw;
	X(EST_YAW) = X(EST_YAW) + Delta_t * m_Last_ZGyro;	// Zgyro added as control
	m_LastUsed_ZGyro = m_Last_ZGyro;

	/* // ENSPB
	X[0][0] = X[0][0] + X[2][0]*Delta_t*sin(X[3][0]);
	X[1][0] = X[1][0] + X[2][0]*Delta_t*cos(X[3][0]);
	X[2][0] = X[2][0];
	X[3][0] = X[3][0] + Delta_t*last_zgyro;
	X[4][0] = X[4][0];

	last_used_zgyro = last_zgyro; //for logging ensures logged zgyro was from the current step
	*/

	// NOTE: After this function returns, the generic EKF propagate calculation
	// will execute.
}




//=============================================================================
void VehicleEKF::PostPropagate( double Delta_t )
{
	// Bound yaw to +/- PI
	X(EST_YAW) = MOOS_ANGLE_WRAP( X(EST_YAW) );
}









//=============================================================================
bool VehicleEKF::PreUpdate_Heading( const HeadingMeasurement& Measurement )
{
	int N = 1;	// Number of measurements (dynamic with range rejection)

	H.ReSize(N, NUM_STATES);
	M.ReSize(N, NUM_NOISE_SOURCES);
	Residual.ReSize(N,1);

	// Initialize the observation matrix
	H = 0.0;
	H(1,4) = H(1,5) = 1.0;

	//H << 0.0f << 0.0f << 1.0f << 0.0f << 0.0f
	//  << 0.0f << 0.0f << 0.0f << 1.0f << 1.0f;

	M = 0.0;
	M(1,6) = 1.0;	/* Map noise associated with heading measurement to
					   the upcoming heading/speed update equation */

	Residual = 0.0;

	Residual(1) = ( MOOSDeg2Rad(Measurement.m_Heading) -
					(X(EST_YAW) + X(EST_YAW_BIAS)) ); // compass bias

	Residual(1) = MOOS_ANGLE_WRAP( Residual(1) );	// Bound angle to +/- PI

	//& *** DEBUG ***
	/*
	cout << "H (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << H << endl;
	cout << "M (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << M << endl;
	cout << "Residual (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << Residual << endl;
	*/
	//& *** END DEBUG ***

	return true;
}




//=============================================================================
bool VehicleEKF::PreUpdate_Speed( const SpeedMeasurement& Measurement )
{
	int N = 1;	// Number of measurements (dynamic with range rejection)

	H.ReSize(N, NUM_STATES);
	M.ReSize(N, NUM_NOISE_SOURCES);
	Residual.ReSize(N,1);

	// Initialize the observation matrix
	H = 0.0;
	H(1,3) = 1.0;

	//H << 0.0f << 0.0f << 1.0f << 0.0f << 0.0f
	//  << 0.0f << 0.0f << 0.0f << 1.0f << 1.0f;

	M = 0.0;
	M(1,5) = 1.0;	/* Map noise associated with velocity measurement to
					   the upcoming heading/speed update equation */

	Residual = 0.0;
	Residual(1) = ( Measurement.m_Speed - X(EST_SPEED) );

	// If the amount of change in velocity is too much, don't update
	// the speed...
	if( (Residual(1)*Residual(1)) > (SPEED_CHECK * SPEED_CHECK))
	{
		Residual(1) = 0.0;
	}

	//& *** DEBUG ***
	/*
	cout << "H (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << H << endl;
	cout << "M (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << M << endl;
	cout << "Residual (VehicleEKF)\n" <<
			setw(10) << setprecision(5) << Residual << endl;
	*/
	//& *** END DEBUG ***

	return true;
}




//=============================================================================
bool VehicleEKF::PreUpdate_LblBeacons( const LblBeaconMeasurement& Measurement )
{
	int N = 1;	// Used as an index into H and M, and counts valid beacons
	Real dEast, dNorth, dDepth;
	Real RangeEst;
	double BeaconE, BeaconN, BeaconDepth;
	static Matrix Htemp(4,NUM_STATES);
	static Matrix Mtemp(4,NUM_NOISE_SOURCES);
	static ColumnVector ResTemp(NUM_NOISE_SOURCES);

	XEN[0] = X(EST_COORD_E);
	XEN[1] = X(EST_COORD_N);

	// Initialize temp matrices
	Htemp = 0.0;
	Mtemp = 0.0;
	ResTemp = 0.0;

	//cout << setw(10) << setprecision(5) << endl << X << endl;

	for (int i = 0; i < 4; i++)
	{
		if( Measurement.m_BeaconRange[i] != 0.0 )
		{
			// Calculate estimated distance from the beacon
			NavBeacon(i + 'A').GetLocation_LocalGrid(BeaconE, BeaconN, BeaconDepth );
			dEast = X(EST_COORD_E) - BeaconE;
			dNorth = X(EST_COORD_N) - BeaconN;
			dDepth = Measurement.m_Depth - BeaconDepth;

			RangeEst = sqrt( (dEast*dEast) + (dNorth*dNorth) + (dDepth*dDepth) );	// Magnitude

			//& *** DEBUG ***
/*			MOOSTrace("\n[VehicleEKF Beacon %c]\n", 'A' + i);
			MOOSTrace("   dEast:  %10.5f\n", dEast);
			MOOSTrace("   dNorth:  %10.5f\n", dNorth);
			MOOSTrace("   dDepth:  %10.5f\n", dDepth);
			MOOSTrace("   RangeEst:%10.5f\n", RangeEst);
*/			//& *** END DEBUG ***

			ResTemp(N) = Measurement.m_BeaconRange[i] - RangeEst;
			if( (ResTemp(N)*ResTemp(N)) < (m_NavBeaconRangeBound*m_NavBeaconRangeBound))
			{
				Htemp(N,1) = (X(EST_COORD_E) - BeaconE) / RangeEst;
				Htemp(N,2) = (X(EST_COORD_N) - BeaconN) / RangeEst;
				Htemp(N,3) = 0.0;
				Htemp(N,4) = 0.0;
				Htemp(N,5) = 0.0;

				Mtemp(N,i+1) = 1.0;

				N++;
				m_NumRangesRejected = -4;
			}
			else
			{
				m_NumRangesRejected++;
			}
		}
	}

	// Verify that the measurement includes enough ranges to perform the
	// update
	if (m_NumRangesRejected >= m_RangeRejectAbortTrigger)
	{
		m_BadRangesFlag = true;
		return false;	// Don't update if we don't have sufficient ranges
	}

	N -= 1;	// Change N from an index into the number of non-zero beacon ranges

	// If no beacon ranges were given, then don't update
	if (N == 0)
	{
		return false;
	}

	// A negative range rejection count indicates that we have enough
	// ranges to update
	if( m_NumRangesRejected < 0 )
	{
		m_NumRangesRejected = 0;
	}

	// Here we 'dynamically' re-size the M and H matrices to
	// accommodate the number of measurements (e.g. the number
	// of accepted LBL ranges) we've gotten.
	H.ReSize(N, NUM_STATES);
	H = Htemp.Rows(1,N);
	M.ReSize(N, NUM_NOISE_SOURCES);
	M = Mtemp.Rows(1,N);
	Residual.ReSize(N);
	Residual = ResTemp.Rows(1,N);

	//& *** DEBUG ***
/*	cout << setw(10) << setprecision(5);
	cout << "H (VehicleEKF)\n" << H << endl;
	cout << "M (VehicleEKF)\n" << M << endl;
	cout << "Residual (VehicleEKF)\n" << Residual << endl;
*/	//& *** END DEBUG ***

	return true;
}


}   // END namespace YellowSubNav
