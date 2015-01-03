//=============================================================================
/*    Copyright (C) 2013  Dave Billin

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
/** @file ActuatorController.h
 *
 * @brief
 *   Declaration of the ActuatorController class used in the University of
 *   Idaho MOOS AUV
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef ACTUATORCONTROLLER_H_
#define ACTUATORCONTROLLER_H_

#include <stdint.h>

#include "ScalarPIDController.h"    // YellowSubUtils scalar PID controller


//=============================================================================
/** @class ActuatorController
 *
 * @brief
 *   Class encapsulating a set of controllers used to produce actuator settings
 *   in the University of Idaho MOOS AUV
 */
//=============================================================================
class ActuatorController
{
public:

    //---------------------------------------------------------------------
    /** @struct VehicleStateError
     * @brief
     *   A data structure describing the error (desired value minus actual
     *   value) of vehicle states
     */
    //---------------------------------------------------------------------
    struct VehicleStateError
    {
        float HeadingError_deg; /**< Heading error in degrees*/
        float SpeedError_mps;   /**< Speed error in meters per second */
        float DepthError;   /**< Depth error in meters */
        float PitchError;   /**< Pitch error in degrees */
        float RollError;    /**< Roll error in degrees */
    };


    //---------------------------------------------------------------------
    /** @struct ActuatorSettings
     * @brief
     *   A data structure containing commanded actuator positions produced
     *   by the ActuatorController
     */
    //---------------------------------------------------------------------
    struct ActuatorSettings
    {
        float Rudder_deg;   /**< Rudder setting in degrees */
        float Elevator_deg; /**< Elevator angle setting in degrees */
        float Aileron_deg;  /**< Aileron angle setting in degrees */
        float Thrust;       /**< Thrust as percentage of full scale (0.0 to
                                 1.0) */
    };



    /** Creates an instance of the object */
    ActuatorController();

    /** Destructor called when the object goes out of scope */
    virtual ~ActuatorController();

    //=========================================================================
    /** Processes all enabled actuator controllers
     * @param [in] dt
     *   Amount of time that has elapsed since controllers were last processed
     *
     * @param [in] Error
     *   Current error in vehicle states (desired value minus actual value)
     *
     * @param [out] RecommendedActuators
     *   Recommended settings to apply to actuators
     */
    void Process( YellowSubUtils::PrecisionTimeInterval const& dt,
                  VehicleStateError const& Error,
                  ActuatorSettings& RecommendedActuators );


    //=========================================================================
    /** Enables or disables all controllers
     * @param ShouldBeEnabled true to enable controllers; false to disable
     */
    void SetAllControllersEnabled( bool ShouldBeEnabled )
    {
        m_HeadingControlIsEnabled = m_SpeedControlIsEnabled =
                m_DepthControlIsEnabled = m_PitchControlIsEnabled =
                m_RollControlIsEnabled = ShouldBeEnabled;
    }


    //=========================================================================
    /** Enables or disables heading control
     * @param ShouldBeEnabled   true to enable heading control
     */
    void SetHeadingControlEnabled( bool ShouldBeEnabled )
    { m_HeadingControlIsEnabled = ShouldBeEnabled; }

    /** @return true if heading control is enabled; else false */
    bool HeadingControlIsEnabled( void ) const
    { return m_HeadingControlIsEnabled; }

    /** @return A reference to the heading controller */
    YellowSubUtils::ScalarPIDController& GetHeadingController( void )
    { return m_HeadingController; }


    //=========================================================================
    /** Enables or disables speed control
     * @param ShouldBeEnabled   true to enable speed control
     */
    void SetSpeedControlEnabled( bool ShouldBeEnabled )
    { m_SpeedControlIsEnabled = ShouldBeEnabled; }

    /** @return true if speed control is enabled; else false */
    bool SpeedControlIsEnabled( void ) const
    { return m_SpeedControlIsEnabled; }

    /** @return A reference to the speed controller */
    YellowSubUtils::ScalarPIDController& GetSpeedController( void )
    { return m_SpeedController; }


    //=========================================================================
    /** Enables or disables depth control
     * @param ShouldBeEnabled   true to enable depth control
     */
    void SetDepthControlEnabled( bool ShouldBeEnabled )
    { m_DepthControlIsEnabled = ShouldBeEnabled; }

    /** @return true if depth control is enabled; else false */
    bool DepthControlIsEnabled( void ) const
    { return m_DepthControlIsEnabled; }

    /** @return A reference to the depth controller */
    YellowSubUtils::ScalarPIDController& GetDepthController( void )
    { return m_DepthController; }


    //=========================================================================
    /** Enables or disables pitch control
     * @param ShouldBeEnabled   true to enable pitch control
     */
    void SetPitchControlEnabled( bool ShouldBeEnabled )
    { m_DepthControlIsEnabled = ShouldBeEnabled; }

    /** @return true if pitch control is enabled; else false */
    bool PitchControlIsEnabled( void ) const
    { return m_DepthControlIsEnabled; }

    /** @return A reference to the pitch controller */
    YellowSubUtils::ScalarPIDController& GetPitchController( void )
    { return m_PitchController; }


    //=========================================================================
    /** Enables or disables roll control
     * @param ShouldBeEnabled   true to enable roll control
     */
    void SetRollControlEnabled( bool ShouldBeEnabled )
    { m_DepthControlIsEnabled = ShouldBeEnabled; }

    /** @return true if roll control is enabled; else false */
    bool RollControlIsEnabled( void ) const
    { return m_DepthControlIsEnabled; }

    /** @return A reference to the roll controller */
    YellowSubUtils::ScalarPIDController& GetRollController( void )
    { return m_RollController; }


protected:
    bool m_HeadingControlIsEnabled; /**< true if heading control is enabled */
    bool m_SpeedControlIsEnabled;   /**< true if speed control is enabled */
    bool m_DepthControlIsEnabled;   /**< true if depth control is enabled */
    bool m_PitchControlIsEnabled;   /**< true if pitch control is enabled */
    bool m_RollControlIsEnabled;    /**< true if roll control is enabled */

    YellowSubUtils::ScalarPIDController m_HeadingController;
    YellowSubUtils::ScalarPIDController m_SpeedController;
    YellowSubUtils::ScalarPIDController m_DepthController;
    YellowSubUtils::ScalarPIDController m_PitchController;
    YellowSubUtils::ScalarPIDController m_RollController;
};

#endif /* ACTUATORCONTROLLER_H_ */
