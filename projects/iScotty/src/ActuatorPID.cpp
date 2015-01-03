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
/** @file ActuatorPID.cpp

 @brief
 	 Implementation of the ActuatorPID class

 @author    Dave Billin
 */
//=============================================================================

#include "ActuatorPID.h"


//=============================================================================
CActuatorPID::CActuatorPID()
: ScalarPID()
{
	// Do nothing.  The base class constructor handles everything
}


//=============================================================================
CActuatorPID::CActuatorPID( double Kp, double Kd, double Ki,
						    double IntegralLimit, double OutputLimit)
: ScalarPID(Kp, Kd, Ki, IntegralLimit, OutputLimit)
{
	// Do nothing.  The base class constructor handles everything
}



//=============================================================================
CActuatorPID::~CActuatorPID()
{
}



//=============================================================================
void CActuatorPID::Reset()
{
    m_dfe = 0.0;
    m_dfeSum = 0.0;
    m_dfeOld = 0.0;
    m_dfeDiff = 0.0;
    m_dfDT = 0.0;
    m_dfOldTime = 0.0;
    m_dfOut = 0.0;
    m_nIterations = 0.0;

    // Clear the history buffer
    std::list<double>::iterator iter;
    for (iter = m_DiffHistory.begin(); iter != m_DiffHistory.end(); iter++)
    {
    	*iter = 0.0;
    }
}
