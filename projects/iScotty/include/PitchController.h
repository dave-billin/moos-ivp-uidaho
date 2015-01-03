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
/** @file PitchController.h
 *
 * @brief
 *  Declaration of a specialized extension to the ActuatorPID class that
 *  incorporates a second proportional input and is used to provide a pitch-
 *  based depth controller
 *
 * @author  Dave Billin
 */
//=============================================================================

#ifndef PITCHCONTROLLER_H_
#define PITCHCONTROLLER_H_


#include "ActuatorPID.h"

/** An extension to the ActuatorPID class that adds a second proportional
 * gained input to the controller.
 *
 * @details
 * 	This object is used to implement the depth-coupled pitch controller in
 * 	the iScotty MOOS application.
 *
 * @ingroup iScotty
 */
class PitchController : public CActuatorPID
{
public:

	//=========================================================================
	/** Creates an instance of the PitchController class with gains and limit
	 *  values set to zero
	 */
	PitchController( void );

	//=========================================================================
	/** Creates a PitchController object and initializes its gains and limits
	 *
	 * @param KpDepth
	 * 	Proportional gain applied to the depth control input
	 *
	 * @param Kp
	 * 	Proportional gain applied to the pitch error
	 *
	 * @param Kd
	 * 	Derivative gain applied to the pitch error
	 *
	 * @param Ki
	 * 	Integral gain on the pitch error
	 *
	 * @param IntegralLimit
	 * 	Applied as an absolute value to saturate the pitch controller's
	 * 	integral term.
	 *
	 * @param OutputLimit
	 * 	Applied as an absolute value to saturate the output of the pitch
	 * 	controller
	 */
	PitchController(double KpDepth, double Kp, double Kd, double Ki,
			  	    double IntegralLimit, double OutputLimit);

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual	~PitchController();


	//=========================================================================
	/** Runs the controller
	 *
	 * @param dfeIn
	 * 	Input error (pitch error as desired pitch minus current pitch)
	 *
	 * @param DepthLaw
	 * 	Depth control law input (output from the depth controller)
	 *
	 * @param dfErrorTime
	 * 	Time when the error was evaluated
	 *
	 * @param dfOut
	 * 	Output control law produced by the pitch controller
	 *
	 * @return
	 * 	true on success; false if ErrorTime describes a negative time interval
	 */
	bool Run(double dfeIn, double DepthLaw, double dfErrorTime, double& dfOut);


	//=========================================================================
	/** Sets the controller's gain terms
	 *
	 * @param KpDepth
	 *	Proportional gain applied to the depth control law input
	 */
	void SetDepthGain(double KpDepth) { m_KpDepth = KpDepth; }


protected:
	double m_KpDepth;	/**< Gain applied to depth control law input */

	//=========================================================================
	/** Called to write to the controller's log file */
	bool Log( void );
};

#endif /* PITCHCONTROLLER_H_ */
