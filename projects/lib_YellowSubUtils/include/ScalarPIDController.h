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
 *  Declaration of a class that implements a scalar Proportional Integral
 *  Derivative (PID) controller
 */
//=============================================================================

#ifndef _SCALAR_PID_H_
#define _SCALAR_PID_H_

#include <string>
#include <vector>
#include <fstream>

#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "PrecisionTime.h"
#include "WindowedAverage.h"

namespace YellowSubUtils
{

//=============================================================================
/** @class ScalarPIDController
 * @brief
 *  A class that implements a scalar Proportional Integral Derivative (PID)
 *  controller.
 */
//=============================================================================
class ScalarPIDController
{
public:

    //=========================================================================
    /// @struct ScalarPID_Parameters
    /// @brief
    ///   Data structure used to provide gains and operating parameters for
    ///   a ScalarPID controller
    class ScalarPIDParameters
    {
        friend class ScalarPIDController;
    public:
        //=====================================================================
        /// Default constructor that initializes values to defaults
        ScalarPIDParameters( void );

        //=====================================================================
        /** Parameterized constructor
         * @param Kp   Proportional gain
         * @param Ki   Integral gain
         * @param Kd   Derivative gain
         * @param ShouldEnableLimits    true enables Integral & Output limiting
         * @param IntegralLimit         Integral limit magnitude
         * @param IntegralLimit         Output limit magnitude
         * @param DerivativeFilterWindowSize
         *   Width of the averaging window applied to the error derivative
         *
         * @param ShouldEnableDeadband  true enables output deadband processing
         * @param Deadband              Output deadband magnitude
         */
        ScalarPIDParameters( float Kp_, float Ki_, float Kd_,
                             bool ShouldEnableLimits = false,
                             float IntegralLimit_ = 0.0,
                             float OutputLimit_ = 0.0,
                             uint32_t DerivativeFilterWindowSize =
                                             DEFAULT_AVERAGING_WINDOW_SIZE,
                             bool ShouldEnableDeadband = false,
                             float Deadband = 0.0f );


        //=====================================================================
        /// Sets the controller's proportional gain
        void Kp( float Kp )                { m_Kp = Kp; }
        /// Returns the controller's proportional gain
        float Kp( void ) const             { return m_Kp; }


        //=====================================================================
        /// Sets the controller's integral gain
        void Ki( float Ki )                { m_Ki = Ki; }
        /// Returns the controller's integral gain
        float Ki( void ) const             { return m_Ki; }


        //=====================================================================
        /// Sets the controller's derivative gain
        void Kd( float Kd )                { m_Kd = Kd; }
        /// Returns the controller's derivative gain
        float Kd( void ) const             { return m_Kd; }


        //=====================================================================
        /// Enables or disables integral and output limits
        /// @param [in] ShouldEnableLimits
        ///   true if integral and output limiting should be enabled; else
        ///   false if they should be disabled
        void SetLimitsEnabled( bool ShouldEnableLimits )
        { m_LimitsAreEnabled = ShouldEnableLimits; }

        /// @return true if Integral and output limiting is enabled; else false
        bool LimitsAreEnabled( void ) const { return m_LimitsAreEnabled; }


        //=====================================================================
        /// Sets the controller's integral magnitude limit
        void IntegralLimit( float Limit )  { m_IntegralLimit = fabs(Limit); }
        /// Returns the controller's integral magnitude limit
        float IntegralLimit( void ) const  { return m_IntegralLimit; }


        //=====================================================================
        /// Sets the controller's output magnitude limit
        void OutputLimit( float Limit )  { m_OutputLimit = fabs(Limit); }
        /// Returns the controller's output magnitude limit
        float OutputLimit( void ) const  { return m_OutputLimit; }


        //=====================================================================
        /// Sets whether the output deadband is enabled
        /// @param [in] ShouldBeEnabled
        ///   true to enable the output deadband; else false to disable it
        void SetOutputDeadbandEnabled( bool ShouldBeEnabled )
        { m_OutputDeadbandIsEnabled = true; }

        /// @return true if the output deadband is enabled
        bool OutputDeadbandIsEnabled( void ) const
        { return m_OutputDeadbandIsEnabled; }

        /// Sets the output deadband magnitude
        void OutputDeadband( float Deadband )
        { m_OutputDeadband = fabs(Deadband); }

        /// @return The output deadband magnitude
        float OutputDeadband( void ) const { return m_OutputDeadband; }

        /// The default number of taps used in the derivative averaging filter
        enum { DEFAULT_AVERAGING_WINDOW_SIZE = 3 };

        //=====================================================================
        /// Sets the number of taps in the derivative averaging filter
        void DerivativeFilterWindowSize( uint32_t NumTaps );

        /// @return The number of taps in the derivative averaging filter
        uint32_t DerivativeFilterWindowSize( void ) const
        { return m_DerivativeFilterWindowSize; }

    protected:
        float m_Kp;      ///< Proportional gain
        float m_Ki;      ///< Integral gain
        float m_Kd;      ///< Derivative gain

        bool m_LimitsAreEnabled; ///< true to enable Integral & Output limits
        float m_IntegralLimit;   ///< Magnitude integral will be limited by
        float m_OutputLimit;     ///< Magnitude output will be limited by

        bool m_OutputDeadbandIsEnabled; ///< true to enable output deadband
        float m_OutputDeadband;         ///< Output deadband magnitude


        /// The number of taps in the averaging filter applied when calculating
        /// the controller's derivative term
        uint32_t m_DerivativeFilterWindowSize;

        //----------------------------------------------------------------
        // NOTE:
        //   ScalarPIDParameters is a POD type, so the automatically-
        //   generated copy constructor and assignment operator are used
        //----------------------------------------------------------------
    };
    //=========================================================================



    //=========================================================================
    /// Default constructor: creates a scalar PID controller with proportional
    /// gain set to 1, integral and derivative gains set to zero, limiting
    /// disabled, and logging functionality disabled */
    ScalarPIDController( void );

    //=========================================================================
    /// Creates a scalar PID controller and initializes its gains and limits
    ///
    /// @param [in] Params
    ///  PID parameters structure used to initialize the controller
    ScalarPIDController( ScalarPIDParameters const& Params );

    //=========================================================================
    /// Destructor
    virtual ~ScalarPIDController();



    //=========================================================================
    /// Configures the controller from specified parameters
    ///
    /// @param [in] Params
    ///   Parameters the controller will be configured with
    void Configure( ScalarPIDParameters const& Params );

    //=========================================================================
    /// @return a reference to the controller's parameters
    ScalarPIDParameters& GetParameters( void )
    { return m_Params; }

    //=========================================================================
    /// @return
    ///   A reference to the averaging filter applied to the controller's error
    ///   derivative
    WindowedAverage<float>& GetDerivativeFilter( void )
    { return m_DerivativeAvgFilter; }



    //=========================================================================
    /// Processes a single iteration of the controller
    ///
    /// @param [in] delta_t
    ///   Time interval elapsed since the last call to Process()
    ///
    /// @param [in] Error
    ///   Current error to be processed by the controller (if not specified,
    ///   the current controller setpoint is used)
    ///
    /// @return
    ///   The control law (output value) produced by the controller
    ///
    /// @throw
    ///   A CMOOSException object if processing fails or if Error is invalid
    virtual float Process( PrecisionTimeInterval const& delta_t,
                           float Error ) throw( CMOOSException );


    //=========================================================================
    /// Resets the controller's state
    void Reset( void );


    /// @addtogroup ScalarPIDController_accessors
    /// @{

    //=========================================================================
    /// Returns the value of the current error state variable
    float Error( void ) const { return m_Error; }

    /// Returns the value of the current integral state variable
    float Integral( void ) const { return m_SigmaError; }

    /// Returns the value of the current derivative state variable (after
    /// application of the controller's averaging filter)
    float Derivative( void ) const { return m_deltaErrorAvg; }

    /// @return The controller's current output
    float Output( void ) const { return m_Output; }

    /// @return The number iterations the controller has processed
    uint32_t NumIterations( void ) const { return m_NumIterations; }

    /// @return Change in time from the last call to process()
    float Delta_t( void ) const
    { return static_cast<float>( m_Delta_t.AsDouble() ); }

    //=========================================================================
    /// Configures logging from the controller
    ///
    /// @param [in] LogFileBaseName
    ///   Base name of log files created by this controller - the actual log
    ///   file name will consist of this name followed by a timestamp in
    ///   seconds since the Unix epoch
    ///
    /// @param [in] LogDirectory
    ///   Directory where log files should be written
    ///
    /// @param [in] ShouldEnableLogging
    ///   true if logging should be enabled initially (default); else false
    ///   if logging should be configured, but not enabled
    ///
    /// @return
    ///   true if logging was successfully enabled; else false if disabled or
    ///   if the specified log file could not be opened for writing
    bool ConfigureLogging( std::string const& LogFileBaseName,
                           std::string const& LogDirectory,
                           bool ShouldEnableLogging = true );

    //=========================================================================
    /// Enables or disables controller logging
    ///
    /// @param [in] ShouldEnableLogging
    ///   true if logging should be enabled initially (default); else false
    ///   if logging should be configured, but not enabled
    ///
    /// @return
    ///   true if logging was successfully enabled; else false if disabled or
    ///   if the log file could not be opened for writing
    bool SetLoggingEnabled( bool ShouldEnableLogging );

    //=========================================================================
    /// @return true if logging is enabled in the controller
    bool LoggingIsEnabled( void ) const { return m_LoggingIsEnabled; }

    //=========================================================================
    /// @return
    ///   The base name of log files created by this controller when logging is
    ///   enabled
    std::string const& GetLogFileBaseName( void ) const
    { return m_LogFileBaseName; }

    //=========================================================================
    /// @return
    ///   The directory where log files created by this controller are written
    ///   when logging is enabled
    std::string const& GetLogDirectory( void ) const { return m_LogDirectory; }

    //=========================================================================
    /// @return
    ///   The full path of the current open log file if logging is enabled;
    ///   else an empty string
    std::string const& GetLogFilePath( void ) const { return m_LogFilePath; }



protected:
    ScalarPIDParameters m_Params;   ///< Current controller gains and settings

    /// @addtogroup state_variables State Variables
    /// @{
    float m_Error;         ///< Current error (m_SetPoint - m_Output)
    float m_SigmaError;    ///< Sum of error (error integral)
    float m_deltaErrorAvg; ///< Average error over time (derivative term)
    float m_Output;        ///< Controller output (control law)
    /// @}

    uint32_t m_NumIterations;   ///< Number of controller iterations
    PrecisionTimeInterval m_Delta_t;    ///< Elapsed time since last Process()
    float m_PrevError;          ///< Previous controller iteration error

    /// Derivative averaging filter taps (dimension is controlled by
    /// m_Params.NumDerivativeFilterTaps)
    WindowedAverage<float> m_DerivativeAvgFilter;

    bool m_LoggingIsEnabled;        ///< true if logging is enabled
    std::string m_LogDirectory;     ///< Directory where log files are written
    std::string m_LogFileBaseName;  ///< Name prefix to use for log files
    std::string m_LogFilePath;      ///< Absolute path of open log file
    std::ofstream m_LogFileStream;  ///< File stream used to write to log

    //=========================================================================
    /// Helper function called each controller iteration to write current
    /// controller states to the log file
    void Log( void );
};

}   // END namespace YellowSubUtils

#endif // END #ifndef _SCALAR_PID_H_
