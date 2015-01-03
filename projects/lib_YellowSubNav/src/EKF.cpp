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
/** @file EKF.cpp
 * @brief
 * Implementation of methods in the EKF class
 */
//=============================================================================
#include <math.h>
#include <string.h>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h" 
#include "MOOS/libMOOS/Utils/MOOSAssert.h"
#include "MOOS/libMOOS/Utils/MOOSException.h" 
#include "EKF.h"

using namespace::NEWMAT;



//=========================
// MACROS
//=========================


namespace YellowSubNav
{


//=============================================================================
EKF::EKF( uint32_t NumStates, uint32_t NumNoiseSources )
:	X(NumStates),
	P(NumStates, NumStates),
	Q(NumStates, NumStates),
	R(NumNoiseSources, NumNoiseSources),
	H(),
	M(),
	Residual(),
    F(),
    A(),
    K(),
	m_I(NumStates),
	m_IsInitialized(false),
	m_InvertFailed(false),
	m_LastPropagateTime(0.0)
{
}





//=============================================================================
EKF::~EKF()
{
	// Clean up any dynamically-allocated resources here!!!
}





//=============================================================================
void EKF::Initialize( Real* pInitialStateValues, double t )
{
	try
	{
		// Invoke initialization method supplied by derived class
		DoInitializeEKF(pInitialStateValues);
	}
	catch (CMOOSException& e)
	{
		throw e;	// Pass on exception from derived class implementation
	}

	m_LastPropagateTime = t;
	m_IsInitialized = true;	// Flag EKF as initialized
}






//=============================================================================
void EKF::UpdateMeasurement( const EKFMeasurement& MeasurementData )
{
	bool ShouldUpdateEquations = true;

	if (m_IsInitialized)
	{
		// Call the pre-update routine of the derived class
		PreUpdateMeasurement(MeasurementData, ShouldUpdateEquations);

		//--------------------------------------
		// Generic EKF measurement update
		//--------------------------------------
		if (ShouldUpdateEquations)
		{
			//----------------------------------------------------------
			// Formula from Table 3 on pg 17 of Ben Armstrong's Masters
			// thesis:	K = (P-)H' * (H(P-)H' + MRM')^-1
			//----------------------------------------------------------
			// A = HPH' + MRM'
			A = H * P * H.t() + M * R * M.t();

			try
			{
				A = A.i();	// A = (HPH' + MRM')^-1
			}
			catch (Runtime_error& e)		// Catch matrix invert failure
			{
				// If (HPH' + MRM') is singular,
				// set the FAILED_INVERT_FLAG and bail
				MOOSTrace("UpdateMeasurement: (HPH' + MRM') is singular:\n");
				MOOSTrace(e.what());	// Print exception
				m_InvertFailed = true;
				m_IsInitialized = false;	// Stop the EKF on critical error
				return;
			}

			K = P * H.t() * A;	// K = PH' * (HPH' + MRM')^-1

			//----------------------------------------------------------
			// Formula from Table 3 on pg 17 of Ben Armstrong's Masters
			// thesis:	(P+) = (I - KH) * (P-)
			//----------------------------------------------------------
			P = (m_I - K * H) * P;

			//----------------------------------------------------------
			// Formula from Table 3 on pg 17 of Ben Armstrong's Masters
			// thesis:	(x+) = (x-) + K * (y-h*(x-), 0)
			//----------------------------------------------------------
			X = X + K * Residual;

			// Call the post-update routine of the derived class
			PostUpdateMeasurement(MeasurementData);
		}
	}
}




//=============================================================================
void EKF::Propagate( double t )
{
	double Delta_t = t - m_LastPropagateTime;

	if ( Delta_t < 0.0 )
	{
		MOOSTrace("Illegal negative time step in EKF::Propagate()");
		return;
	}
	else if (m_IsInitialized)
	{
		m_LastPropagateTime = t;	// Store most recent propagate time

		// Call the pre-propagate routine of the derived class
		PrePropagate(Delta_t);

		//--------------------------------------
		// Generic EKF propagation
		//--------------------------------------
		P = F * P * F.t() + Q;

		// Call the post-propagate routine of the derived class
		PostPropagate(Delta_t);
	}
}


}   // END namespace YellowSubNav

