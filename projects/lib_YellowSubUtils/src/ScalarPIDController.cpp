///////////////////////////////////////////////////////////////////////////////
//
//   MOOS - Mission Oriented Operating Suite 
//  
//   A suit of Applications and Libraries for Mobile Robotics Research 
//   Copyright (C) 2001-2005 Massachusetts Institute of Technology and 
//   Oxford University. 
//    
//   This software was written by Paul Newman at MIT 2001-2002 and Oxford 
//   University 2003-2005. email: pnewman@robots.ox.ac.uk. 
//
//   Comments and modifications were contributed in 2013 by Dave Billin for
//   the University of Idaho. email: david.billin@vandals.uidaho.edu
//
//   This file was removed from the MOOS source tree as of MOOS v10 and later.
//   Consequently, it has been copied, re-worked, and integrated into the
//   moos-ivp-uidaho source tree.
//        
//   This program is free software; you can redistribute it and/or 
//   modify it under the terms of the GNU General Public License as 
//   published by the Free Software Foundation; either version 2 of the 
//   License, or (at your option) any later version. 
//          
//   This program is distributed in the hope that it will be useful, 
//   but WITHOUT ANY WARRANTY; without even the implied warranty of 
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
//   General Public License for more details. 
//            
//   You should have received a copy of the GNU General Public License 
//   along with this program; if not, write to the Free Software 
//   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
//   02111-1307, USA. 
//
//////////////////////////    END_GPL    //////////////////////////////////////
//=============================================================================
/** @file ScalarPID.h
 * @brief
 *  Implementation of the ScalarPID class, implementing a scalar Proportional
 *  Integral Derivative (PID) controller
 */
//=============================================================================
#include <cmath>
#include <cassert>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "ScalarPIDController.h"

using namespace std;

namespace YellowSubUtils
{


//=============================================================================
ScalarPIDController::ScalarPIDParameters::ScalarPIDParameters( void )
  : m_Kp( 1 ), m_Ki( 0 ), m_Kd( 0 ),
    m_LimitsAreEnabled( true ), m_IntegralLimit( 0 ), m_OutputLimit( 0 ),
    m_OutputDeadbandIsEnabled( false ),
    m_OutputDeadband( 0.0f ),
    m_DerivativeFilterWindowSize( DEFAULT_AVERAGING_WINDOW_SIZE )
{
}


//=============================================================================
ScalarPIDController::ScalarPIDParameters::ScalarPIDParameters(
                                          float Kp_, float Ki_, float Kd_,
                                          bool LimitsAreEnabled_,
                                          float IntegralLimit_,
                                          float OutputLimit_,
                                          uint32_t DerivativeFilterWindowSize,
                                          bool ShouldEnableDeadband,
                                          float Deadband )
  : m_Kp( Kp_ ), m_Ki( Ki_ ), m_Kd( Kd_ ),
    m_LimitsAreEnabled( LimitsAreEnabled_ ),
    m_IntegralLimit( IntegralLimit_ ), m_OutputLimit( OutputLimit_ ),
    m_OutputDeadbandIsEnabled( ShouldEnableDeadband ),
    m_OutputDeadband( Deadband ),
    m_DerivativeFilterWindowSize( DerivativeFilterWindowSize )
{
}


//=============================================================================
void ScalarPIDController::ScalarPIDParameters::DerivativeFilterWindowSize(
                                                            uint32_t NumTaps )
{
    m_DerivativeFilterWindowSize = ( NumTaps > 0 ) ? NumTaps : 1;
}



//=============================================================================
ScalarPIDController::ScalarPIDController( void )
  : m_DerivativeAvgFilter(ScalarPIDParameters::DEFAULT_AVERAGING_WINDOW_SIZE),
    m_LoggingIsEnabled( false )
{
    Configure( ScalarPIDParameters() ); // Configure with default parameters
    Reset();    // Reset state
}



//=============================================================================
ScalarPIDController::ScalarPIDController( ScalarPIDParameters const& Params )
  : m_Params(Params),
    m_DerivativeAvgFilter( Params.m_DerivativeFilterWindowSize ),
    m_LoggingIsEnabled( false )
{
    Configure( Params );
    Reset();
}


//=============================================================================
ScalarPIDController::~ScalarPIDController()
{
    if ( m_LogFileStream.is_open() )
    {
        m_LogFileStream.close();
    }
}


//=============================================================================
void ScalarPIDController::Configure( ScalarPIDParameters const& Params )
{
    // Create a derivative filter if the tap count changes
    if ( m_Params.m_DerivativeFilterWindowSize !=
                        Params.m_DerivativeFilterWindowSize )
    {
        m_DerivativeAvgFilter = WindowedAverage<float>(
                                        Params.m_DerivativeFilterWindowSize );
    }

    // Copy controller parameters
    m_Params = Params;
}



//=============================================================================
float ScalarPIDController::Process( PrecisionTimeInterval const& delta_t,
                                    float Error )
throw( CMOOSException )
{
    // Validate input setpoint
    if ( isnanf( Error ) != 0 )
    {
        throw CMOOSException("Illegal setpoint: NAN");
    }
    else if ( isinff( Error ) != 0 )
    {
        throw CMOOSException("Illegal setpoint: infinity");
    }

    // Store the number of seconds elapsed since the controller was
    // last processed
    m_Delta_t = delta_t;
    float dt = static_cast<float> ( delta_t.AsDouble() );

    // Rotate in the new error
    m_Error = Error;

    //------------------------------------------------
    // Calculate derivative term
    //------------------------------------------------
    // Large, sudden jumps in error can take the form of impulses, which
    // impart energy outside the controller's bandwidth and de-stabilize
    // calculation of the derivative term.  To account for these cases,
    // the error applied to the derivative calculation is averaged over
    // several iterations of the controller.  This effectively places one
    // or more zeros at the controller's Nyquist frequency (assuming a
    // regular period of processing) which tends to smooth out such
    // transitions and increase stability.  The width of the averaging
    // window determines the time constant of this filtering.
    float delta_Error = ( m_Error - m_PrevError ) / dt;

    // Calculate average change in error
    m_deltaErrorAvg = m_DerivativeAvgFilter.ProcessSample( delta_Error );
    m_PrevError = m_Error;  // Rotate error


    //------------------------------------------------
    // Calculate integral term
    //------------------------------------------------
    if( m_Params.m_Ki > 0 )
    {
        m_SigmaError += m_Params.m_Ki * m_Error * dt;

        if ( m_Params.m_LimitsAreEnabled )
        {
            // Saturate integral by IntegralLimit
            if( fabs(m_SigmaError) >= m_Params.m_IntegralLimit )
            {
                float Sign = ( m_SigmaError < 0.0 ) ? -1.0 : 1.0;
                m_SigmaError = Sign * m_Params.m_IntegralLimit;
            }
        }
    }
    else
    {
        m_SigmaError = 0;
    }


    //------------------------------------------------
    // Calculate PID control algorithm
    //------------------------------------------------
    // NOTE: m_SigmaError already incorporates Ki
    float NewOutput = m_Params.m_Kp * m_Error
                        + m_Params.m_Kd * m_deltaErrorAvg
                        + m_SigmaError;

    // Saturate output by OutputLimit
    if ( m_Params.m_LimitsAreEnabled &&
         ( fabs(m_Output) >= m_Params.m_OutputLimit ) )
    {
        int Sign = ( m_Output < 0 ) ? -1 : 1;
        NewOutput = Sign * m_Params.m_OutputLimit;
    }


    if ( m_Params.m_OutputDeadbandIsEnabled )
    {
        // Process output deadband
        // Don't update controller output if the new calculated output has
        // not changed by an amount greater than the deadband
        if ( fabs(NewOutput - m_Output) > m_Params.m_OutputDeadband )
        {
            m_Output = NewOutput;
        }
    }
    else
    {
        // Always update controller output if deadband is disabled
        m_Output = NewOutput;
    }

    // Increment controller iteration count
    m_NumIterations++;
    if ( m_NumIterations == 0 )
    {
        m_NumIterations++;  // Prevent iteration count roll-over to zero
    }

    if ( m_LoggingIsEnabled )
    {
        Log();
    }

    return m_Output;
}




//=============================================================================
void ScalarPIDController::Reset( void )
{
    m_Error = m_SigmaError = m_deltaErrorAvg = m_Output = m_PrevError = 0.0f;
    m_NumIterations = 0;

    // Reset averaging filter history
    m_DerivativeAvgFilter.Reset();
}


//=============================================================================
bool ScalarPIDController::ConfigureLogging( string const& LogFileBaseName,
                                            string const& LogDirectory,
                                            bool ShouldEnableLogging )
{
    m_LogFileBaseName = LogFileBaseName;
    m_LogDirectory = LogDirectory;
    return SetLoggingEnabled( ShouldEnableLogging );
}


//=============================================================================
bool ScalarPIDController::SetLoggingEnabled( bool ShouldEnableLogging )
{
    bool LoggingWasEnabled = false;

    // If enablement is not changing, do nothing
    if ( ShouldEnableLogging ^ m_LoggingIsEnabled )
    {
        if ( ShouldEnableLogging == true )
        {
            ostringstream LogPathName;
            LogPathName << m_LogDirectory << "/" << m_LogFileBaseName
                    << PrecisionTime::Now().AsDouble() << ".csv";

            // Transition from log disabled ===> log enabled
            m_LogFileStream.open( LogPathName.str().c_str(), ios_base::trunc );
            m_LoggingIsEnabled = !m_LogFileStream.fail();
            if ( m_LoggingIsEnabled == true )
            {
                m_LogFilePath = LogPathName.str();

                // Write column headings to csv file
                m_LogFileStream
                    << "NumIterations,Error,dt,Kp,Ki,Kd,Output,"
                       "WindowSize" << endl;
            }
            else
            {
                cerr << "Failed to open log file: " << LogPathName.str()
                     << endl;
                m_LogFilePath.clear();
            }

            LoggingWasEnabled = m_LoggingIsEnabled;
        }
        else
        {
            // Transition from log enabled ===> log disabled
            if ( m_LogFileStream.is_open() )
            {
                m_LogFileStream.close();
            }
            m_LoggingIsEnabled = false;
        }
    }

    return LoggingWasEnabled;
}


//=============================================================================
void ScalarPIDController::Log( void )
{
    if( m_LoggingIsEnabled && m_LogFileStream.good() )
    {
        // A 32-bit float has slightly less than 7 digits of precision...
        m_LogFileStream.setf( ios_base::fixed |
                              ios::showpoint |
                              ios_base::floatfield );

        m_LogFileStream << setprecision(7);
        m_LogFileStream.setf(ios::left);    // Left-justify fields

        m_LogFileStream << m_NumIterations << ",";
        m_LogFileStream << m_Error << ",";
        m_LogFileStream << setprecision(14);    // Precision of doubles
        m_LogFileStream << m_Delta_t.AsDouble() << ",";
        m_LogFileStream << setprecision(7);
        m_LogFileStream << m_Params.m_Kp << ",";
        m_LogFileStream << m_Params.m_Ki << ",";
        m_LogFileStream << m_Params.m_Kd << ",";
        m_LogFileStream << m_Output << ",";
        m_LogFileStream << m_Params.m_DerivativeFilterWindowSize << endl;;

        m_LogFileStream.flush();
    }
}


}   // END namespace YellowSubUtils
