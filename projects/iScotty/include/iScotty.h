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
/** @file iScotty.h

@brief
	Declaration of a MOOS instrument class used to implement communication
	with the Rabbit3000 SCOTTY actuator control module in the U of I
	('Yellow Sub') AUV

@author Dave Billin

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifndef _ISCOTTY_H_
#define _ISCOTTY_H_

#include <string>
#include <stdint.h>
#include "MOOS/libMOOS/MOOSLib.h"
#include "ActuatorPID.h"		// Extension of MOOSNavLib CScalarPID class
#include "PitchController.h"	// Depth-coupled pitch controller class
#include "ScottyModule.h"		// Scotty module class




//=====================================
// CONFIGURATION CONSTANTS

/** @def ISCOTTY_VERBOSITY
@brief
	Used to set the amount of debugging messages posted by this module.  Valid
	parameter values are:
		- 0: Debugging messages disabled
		- 1: Basic debugging messages
		- 2: More annoying debug messages...
*/
#define ISCOTTY_VERBOSITY	2



/** @def ACTUATOR_UPDATE_PERIOD_SEC
 * The minimum period in seconds at which actuator settings will be received
 * from the MOOS database
 */
#define ACTUATOR_UPDATE_PERIOD_SEC	0.1


//=============================================================================
/** MOOSApp object used to implement the iScotty application
 * @ingroup iScotty
*/
//=============================================================================
class iScotty : public CMOOSApp
{
public:

	//=========================================================================
	/** Creates an instance of the object */
	iScotty( void );

	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~iScotty();


	//======================================
	// Methods inherited from CMOOSApp
	//======================================

	//=========================================================================
	/** Called when new mail has arrived from the MOOS Database.
	@details
	    This method will be called at a rate of approximately 1/CommsTick Hz.
		In this function you'll most likely interate over the collection of
		mail messages received or call a m_Comms::PeekMail() to look for a
		specific named message.

	@param NewMail
		A reference to the list of received mail from the MOOS Database

	@return
		true on success; false on error
	*/
	bool OnNewMail(MOOSMSG_LIST & NewMail);


	//=========================================================================
	/** If command message filtering is enabled for this application (via the
		EnableCommandMessageFiltering() function), then this function is called
		when a command message having the identifier
		ISCOTTY_CMD is recieved from the MOOS Database.

	@param Msg
		A copy of the received command message

	@return
		true on success; false on error

	@see CMOOSApp::EnableCommandMessageFiltering
	*/
	bool OnCommandMsg(CMOOSMsg Msg);


	//=========================================================================
    /** This function is where the application can do most of its work.
	@details
		The rate at which Iterate() is called is determined by the value of the
		AppTick parameter specified in the (.moos) mission file.  The value of
		AppTick is loaded automatically when the application starts up.
	*/
    bool Iterate( void );


	//=========================================================================
    /** Called when the application first starts up
	@details
		This function is called as the application first starts up before any
		calls to Iterate() begin.
	*/
    bool OnStartUp( void );


	//=========================================================================
    /** Called when the application connects to the MOOS Database. */
    bool OnConnectToServer( void );


	//=========================================================================
    /** Called when the application is disconnected from the MOOS Database. */
    bool OnDisconnectFromServer( void );



protected:

    //=========================================================================
    /** Loads and applies mission file parameters */
    bool LoadMissionFileParameters( void );

    //=========================================================================
    /** Loads parameters from the vehicle config file */
    bool LoadVehicleConfigParameters( std::string& sConfigFilePath );

    //=========================================================================
    /** Runs the Yaw, Speed, Depth, and Roll controllers and stores
     * controller outputs to m_DesiredRudder, m_DesiredElevator,
     * m_DesiredAileron, and m_DesiredThrust.
     */
    void RunPIDControllers( void );

private:
    bool m_AppIsInitialized;	/**< Set to true at the end of OnStartup() */
    ScottyModule* m_pScotty;	/**< Object encapsulating communication and
    								 interaction with SCOTTY */

    std::string m_sScottyHostname;	/**< Hostname or IP address of the SCOTTY
    								     module */

    uint16_t m_ScottyPort;		/**< Network port of the SCOTTY module */

	bool m_PropIsEnabled;		/**< true (default) if propeller commands are
									 not disabled */

	bool m_HelmIsActive;		/**< Holds the logical opposite of the value of
									 MOOS_MANUAL_OVERIDE.  Used to determine
									 whether pHelmIvp is engaged */

	bool m_YawControlIsEnabled;		/**< True if the Yaw PID controller is enabled */
	bool m_SpeedControlIsEnabled;	/**< True if the Speed PID controller is enabled */
	bool m_DepthControlIsEnabled;	/**< True if the Depth PID controller is enabled */
	bool m_RollControlIsEnabled;	/**< True if the Roll PID controller is enabled */
	bool m_PitchControlIsEnabled;	/**< True if the Pitch controller is enabled */

	CActuatorPID m_YawPID;		/**< Controller to map DESIRED_YAW to a SCOTTY
	 	 	 	 	 	 	 	 	 rudder setting */

	CActuatorPID m_SpeedPID;	/**< Controller to map DESIRED_SPEED to Scotty
									 propeller thrust */

	CActuatorPID m_DepthPID;	/**< Controller to map DESIRED_DEPTH to a depth
	 	 	 	 	 	 	 	 	 control law used by the pitch controller */

	CActuatorPID m_RollPID;		/**< Controller to map DESIRED_ROLL to Scotty
	 	 	 	 	 	 	 	 	 aileron setting */

	/**< @var m_PitchController
	 * @brief
	 *	Depth-coupled pitch controller used to map DESIRED_PITCH and the
	 *	depth control law produced by m_DepthPID to an elevator setting
	 */
	PitchController m_PitchController;

	CActuatorPID* m_ControllerTable[5];	/**< Table used to access controllers */

	/** Constants corresponding to elements of m_SubscribedVars and
	 * m_pSubscribeVarValues
	 */
	enum e_SubscribeVariableIndices
	{
		Idx_NavYaw = 0,
		Idx_NavSpeed,
		Idx_NavDepth,
		Idx_NavPitch,
		Idx_NavRoll,

		Idx_DesiredHeading,
		/*Idx_DesiredYaw,*/
		Idx_DesiredSpeed,
		Idx_DesiredDepth,
		Idx_DesiredPitch,
		Idx_DesiredRoll,

		Idx_DesiredRudder,
		Idx_DesiredElevator,
		Idx_DesiredAileron,
		Idx_DesiredThrust,

		Idx_MaxDepth,

		Idx_YawPid_Kp,
		Idx_YawPid_Ki,
		Idx_YawPid_Kd,
		Idx_YawPid_IntSat,
		Idx_YawPid_OutSat,

		Idx_SpeedPid_Kp,
		Idx_SpeedPid_Ki,
		Idx_SpeedPid_Kd,
		Idx_SpeedPid_IntSat,
		Idx_SpeedPid_OutSat,

		Idx_DepthPid_Kp,
		Idx_DepthPid_Ki,
		Idx_DepthPid_Kd,
		Idx_DepthPid_IntSat,
		Idx_DepthPid_OutSat,

		Idx_RollPid_Kp,
		Idx_RollPid_Ki,
		Idx_RollPid_Kd,
		Idx_RollPid_IntSat,
		Idx_RollPid_OutSat,

		Idx_PitchPid_Kp,
		Idx_PitchPid_Ki,
		Idx_PitchPid_Kd,
		Idx_PitchPid_IntSat,
		Idx_PitchPid_OutSat,
		Idx_PitchPid_KpDepth,

		NUM_SUBSCRIBED_VARIABLES
	};


	enum e_PIDParamIndices
	{
		PIDParam_Kp = 0,
		PIDParam_Ki,
		PIDParam_Kd,
		PIDParam_IntSat,
		PIDParam_OutSat,
		PIDParam_KpDepth,
		NUM_PID_PARAMETERS
	};

	/** @var m_DesiredActuatorValuesChanged
	 * @brief
	 *	Used to signal from OnNewMail() to Iterate() that the value of
	 *	DESIRED_RUDDER, DESIRED_ELEVATOR, DESIRED_AILERON, or DESIRED_THRUST
	 *	has changed and should be updated
	 */
	bool m_DesiredActuatorValuesChanged;


	/** @var m_SubscribedVarChanged
	 * @brief
	 *	Used to signal between functions that the value of a subscribed
	 *	variable has changed.
	 */
	bool m_SubscribedVarChanged[NUM_SUBSCRIBED_VARIABLES];

	/** @var m_pSubscribedVarValueTable
	 * @brief
	 * 	An array of floats containing the latest values of subscribed numeric
	 * 	MOOS variables.  Values are indexed using the elements of
	 * 	e_SubscribeVariableIndices
	 */
	double m_SubscribedVarValueTable[NUM_SUBSCRIBED_VARIABLES];



	// Servo trims loaded from vehicle specific data file
	uint8_t m_ServoCenterRudder;/**< Servo center position for rudder */
	uint8_t m_ServoCenterElevL;	/**< Servo center position for left elevator */
	uint8_t m_ServoCenterElevR;	/**< Servo center position for right elevator */
	float m_CouplingCoeff;		/**< Coefficient of coupling between rudder
									 and elevators */

	float m_VelocityPerRPM;		/**< Used to convert RPM to estimated speed */

	float m_VelocityPerRPMOffset;	/**< Used to convert RPM to estimated
										 speed */

	int m_Verbosity;			/**< Verbosity level of debugging messages */

	/** @var sm_SubscribedVariableNames
	 * @brief
	 *	Lookup table of names of MOOS variables that iScotty subscribes to.
	 *	Elements of this table must directly correspond to the entries in
	 *	e_SubscribeVariableIndices.
	 */
	static const char* sm_SubscribedVariableNames[NUM_SUBSCRIBED_VARIABLES];

	static const char* sm_szControllerNames[5];
	static const char* sm_szPidParamNames[NUM_PID_PARAMETERS];


	//=========================================================================
	/** Helper function to load controller gain mission file parameters */
	bool LoadControllerParameters( void );

	//=========================================================================
	/** Helper function called to publish the current controller gains once
	 *  OnStartup is finished or the app connects to the MOOSDB
	 */
	void PublishControllerParameters( void );

	//=========================================================================
	/** Helper function used to update PID gains and saturation values with
	 * changes from the MOOSDB or the mission file */
	void UpdateControllerParameters( void );

	//=========================================================================
	/** Prints SCOTTY controller gains to stdio */
	void PrintControllerParameters( void );

	//=========================================================================
	/** Resets all SCOTTY controllers */
	void ResetControllers( void );


	//=========================================================================
	/** Helper function to set SCOTTY actuator trims from a message received
	 *  from the MOOSDB
	 *
	 * @param sTrimSettings
	 * 	A string containing the trim settings to be applied
	 */
	void SetTrimsFromMOOSMail( std::string& sTrimSettings );



	// Prevent automatic generation of copy constructor and assignment operator
	iScotty (const iScotty&);
    const iScotty& operator= (const iScotty&);
};

#endif	// END #ifdef _ISCOTTY_H_
