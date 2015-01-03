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
/** @file PitchController.cpp
 *
 * @brief
 *	Implementation of the PitchController class
 *
 * @author Dave Billin
 */
//=============================================================================

#include <math.h>
#include <iostream>
#include <iomanip>
#include <list>
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "PitchController.h"

using namespace::std;


//=============================================================================
PitchController::PitchController( void )
 : m_KpDepth(0.0)
{

}


//=============================================================================
PitchController::PitchController( double KpDepth, double Kp, double Kd,
								  double Ki, double IntegralLimit,
								  double OutputLimit)
: CActuatorPID(Kp, Kd, Ki, IntegralLimit, OutputLimit),
  m_KpDepth(KpDepth)
{
}



//=============================================================================
PitchController::~PitchController()
{
}



//=============================================================================
bool PitchController::Run( double dfeIn, double DepthLaw, double dfErrorTime,
						   double& dfOut)
{
    m_dfe  = dfeIn;

    //figure out time increment...
    if(m_nIterations++!=0)
    {

        m_dfDT = dfErrorTime-m_dfOldTime;

        if(m_dfDT<0)
        {
            MOOSTrace("PitchController::Run() : negative or zero sample time\n");
            return false;
        }
        else if(m_dfDT ==0)
        {
            //nothing to do...
            dfOut = m_dfOut;
            Log();
            return true;
        }

        //figure out differntial
        double dfDiffNow = (dfeIn-m_dfeOld)/m_dfDT;
        m_DiffHistory.push_front(dfDiffNow);
        while(m_DiffHistory.size()>=m_nHistorySize)
        {
            m_DiffHistory.pop_back();
        }

        m_dfeDiff = 0;
        list<double>::iterator p;
        for(p = m_DiffHistory.begin();p!=m_DiffHistory.end();p++)
        {
            m_dfeDiff   += *p;
        }
        m_dfeDiff/=m_DiffHistory.size();
    }
    else
    {
        //this is our first time through
        m_dfeDiff = 0;
    }


    if(m_dfKi>0)
    {
        //calculate integral term
        m_dfeSum    +=  m_dfKi*m_dfe*m_dfDT;

        //prevent integral wind up...
        if(fabs(m_dfeSum)>=fabs(m_dfIntegralLimit))
        {


            int nSign = (int)(fabs(m_dfeSum)/m_dfeSum);
            m_dfeSum = nSign*fabs(m_dfIntegralLimit);


        }
    }
    else
    {
        m_dfeSum = 0;
    }
    //do pid control
    m_dfOut = m_KpDepth * DepthLaw +
    		  m_dfKp * m_dfe +
              m_dfKd * m_dfeDiff+
              m_dfeSum; //note Ki is already in dfeSum


    //prevent saturation..
    if(fabs(m_dfOut)>=fabs(m_dfOutputLimit) )
    {
        int nSign =(int)( fabs(m_dfOut)/m_dfOut);
        m_dfOut = nSign*fabs(m_dfOutputLimit);
    }

    //save old value..
    m_dfeOld    = m_dfe;
    m_dfOldTime = dfErrorTime;


    dfOut = m_dfOut;

    //do logging..
    Log();

    return true;
}


//=============================================================================
bool PitchController::Log( void )
{

    int nWidth = 17;

    if(m_bLog)
    {
        if(!m_LogFile.is_open())
        {
            string sName = MOOSFormat("%s%s%s.pid",
                m_sLogPath.c_str(),
                m_sName.c_str(),
                MOOSGetTimeStampString().c_str());
            m_LogFile.open(sName.c_str());
            if(!m_LogFile.is_open())
            {
                m_bLog = false;
                return false;
            }

            m_LogFile.setf(ios::left);

            m_LogFile<<"%% Kp = "<<m_dfKp<<endl;
            m_LogFile<<"%% Kd = "<<m_dfKd<<endl;
            m_LogFile<<"%% Ki = "<<m_dfKi<<endl;
            m_LogFile<<setw(20)<<"%T";
            m_LogFile<<setw(nWidth)<<"Kp";
            m_LogFile<<setw(nWidth)<<"KpDepth";
            m_LogFile<<setw(nWidth)<<"Kd";
            m_LogFile<<setw(nWidth)<<"Ki";
            m_LogFile<<setw(nWidth)<<"DT";
            m_LogFile<<setw(nWidth)<<"Output";
            m_LogFile<<setw(nWidth)<<"InputError";
            m_LogFile<<setw(nWidth)<<"Kp*e";
            m_LogFile<<setw(nWidth)<<"KpDepth*e";
            m_LogFile<<setw(nWidth)<<"Kd*de/dt";
            m_LogFile<<setw(nWidth)<<"Ki*int(e)";
            m_LogFile<<setw(nWidth)<<"Desired";
            m_LogFile<<setw(nWidth)<<"Actual"<<endl;

        }

           //do pid control
        //    m_dfOut = m_dfKp*m_dfe+
        //    m_dfKd*m_dfeDiff+
        //   m_dfKi*m_dfeSum;
        m_LogFile.setf(ios::left);
        m_LogFile<<setw(20)<<setprecision(12)<<m_dfOldTime<<' ';
        m_LogFile<<setprecision(5);
        m_LogFile<<setw(nWidth)<<m_dfKp<<' ';
        m_LogFile<<setw(nWidth)<<m_KpDepth<<' ';
        m_LogFile<<setw(nWidth)<<m_dfKd<<' ';
        m_LogFile<<setw(nWidth)<<m_dfKi<<' ';
        m_LogFile<<setw(nWidth)<<m_dfDT<<' ';
        m_LogFile<<setw(nWidth)<<m_dfOut<<' ';
        m_LogFile<<setw(nWidth)<<m_dfe<<' ';
        m_LogFile<<setw(nWidth)<<m_dfKp*m_dfe<<' ';
        m_LogFile<<setw(nWidth)<<m_KpDepth*m_dfe<<' ';
        m_LogFile<<setw(nWidth)<<m_dfKd*m_dfeDiff<<' ';
        m_LogFile<<setw(nWidth)<<m_dfeSum<<' '; //Ki is already in dfeSum
        m_LogFile<<setw(nWidth)<<m_dfGoal<<' ';
        m_LogFile<<setw(nWidth)<<m_dfGoal-m_dfe<<' ';

        m_LogFile<<endl;

    }

    m_LogFile.flush();

    return true;
}

