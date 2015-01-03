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
/** @file ActuatorPID.h

 @brief
 	 An extension of the MOOS scalar PID class that adds access to the integral
 	 term of the controller

 @author Dave Billin
 */
//=============================================================================

#ifndef ACTUATORPID_H_
#define ACTUATORPID_H_

#include <ScalarPID.h>	// Scalar PID class from

/** An extention of the MOOS CScalarPID class that adds the ability to return
 * the current integral value from the PID controller.
 *
 * @ingroup iScotty
 */
class CActuatorPID : public CScalarPID
{
public:
	//=========================================================================
	//! Creates a PID controller with all parameters set to zero
	CActuatorPID();

	//! Creates a PID controller with specified gains and limits
	// This simply calls the base class constructor
	CActuatorPID( double Kp, double Kd, double Ki,
				  double IntegralLimit, double OutputLimit);


	//! Called when the object goes out of scope
	virtual	~CActuatorPID();

	//=========================================================================
	/** Returns the controller's integral limit value */
	double IntegralLimit() const { return m_dfIntegralLimit; }

	//=========================================================================
	/** Returns the controller's output limit value */
	double OutputLimit() const { return m_dfOutputLimit; }

	//=========================================================================
	/** Returns the current value of the PID controller's integral term for use
		in calibrating the actuator's trim
	*/
	double GetIntegral( void ) const	{ return m_dfeSum; }


	//=========================================================================
	/** Resets the PID controller by zeroing the accumulator and iteration
		count, and clearing the controller's history */
	void Reset();

};

#endif /* ACTUATORPID_H_ */
