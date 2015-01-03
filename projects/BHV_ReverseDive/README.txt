===============================================================================
* BHV_ReverseDive
*
* Description:
*	An IvP behavior used to implement diving in reverse.  This is useful for
*   positively-buoyant AUV's whose control surfaces must be submerged before
*   full controllability is achieved during a mission.
* 
* Created June 2011
* Author: Dave Billin
*
===============================================================================
 
===================
   OVERVIEW
===================
	BHV_YellowSubReverseDive is an IvP behavior used to implement a reverse-
	dive sequence for an AUV in which the AUV dives aft-first rather than using
	forward motion.  For positively-buoyant AUV's that derive their propulsion
	from an aft-mounted propeller, this may prove a faster, more effective way
	to make the vehicle dive than other, more conventional alternatives.  
	
	The reverse-dive sequence is implemented as a state machine having the 
	following states:
	
	STATE:			DESCRIPTION:
	-------------	----------------------------------------------------------
	INIT			This state runs for a single Helm iteration when the dive 
	                starts, then immediately transitions to the PitchDown 
	                state.  It is used to prepare internal variables used by
	                the behavior.
					
	PitchUp		    In this state, the AUV's propeller is driven in reverse
	                (typically at a fairly high rate), and the elevators are
	                adjusted upward.  This results in aft-ward motion that 
	                tends to push the aft of the AUV down into the water.  This
	                continues until the vehicle's pitch increases to or beyond 
	                a target value, at which point the dive transitions to the 
	                DiveToDepth state.  If the desired pitch is not attained
	                within the timeout configured for the state, a transition
	                is made to the Error state instead.
	                
					
	DiveToDepth		In this state, the AUV (already at a positive pitch) is 
	                further propelled in reverse until it reaches a target 
	                depth.  Once this depth is reached, the dive transitions to
	                the LevelOut state.  If the target depth is not attained
	                within the timeout configured for the state, a transition
	                is made to the Error state instead.
					
	LevelOut		In this state, reverse motion continues while the AUV's
	                pitch is adjusted to a target value (typically 0 degrees).
	                If the target pitch (or less) is attained, the state 
	                machine transitions to the Success state to signal a 
	                successful dive.  If the timeout configured for the state
	                elapses, a transition is made to the Error state instead. 
	
	Success			This state runs for a single iteration and serves to signal
	                a successful dive.  Following this, the dive state will be
	                reset to INIT, and the behavior marked as complete. 

	Error			This state is entered via failure of any other state.  Any 
	                error flags specified in the behavior configuration will be
	                published on entry to this state.  Following this, the dive 
	                state will be reset to INIT, and the behavior marked as 
	                complete.

======================
 BEHAVIOR PARAMETERS
======================
	All standard IvP Helm behavior parameters (duration, runflags, endflags, 
	etc.) are supported by BHV_ReverseDive.  In addition, the following 
	parameters may be used to control dive particulars.

	Parameter			Description
	------------------	-------------------------------------------------------
	init_flag           One or more of these entries may be used to specify
	                    variable-value pairs to be published once when the dive
	                    sequence begins.  This could be used to command the
	                    AUV's front-side controller to reverse the direction of 
	                    the propeller motor, or to trigger other 
	                    vehicle-specific tasks.


    pitchdown_pitch_deg Target pitch (degrees) used in the PitchDown state.  If 
                        not specified, a default value of 20 degrees is used.   
                        
	pitchdown_timeout	Timeout (in seconds) used in the PitchDown state.  If
						not specified, a default value of 5.0 seconds is used.

	pitchdown_speed     Target speed (meters per second) used to create an 
	                    objective function in the speed domain when the 
	                    PitchDown state is running.  Speed values may be 
	                    negative or positive.  It is left to the vehicle 
	                    platform or additional control via the pitchdown_flag
	                    parameter to control the propeller direction to obtain 
    	                reverse motion.  Default value is 5.0 m/sec.
						
    pitchdown_flag      One or more of these entries may be used to specify
                        variable-value pairs to be published on each Helm
                        iteration that the PitchDown state is running
                         
                         
                         
	divetodepth_target	Target depth (meters) that is sought by the DiveToDepth 
						state.  If not specified, the default value of 0.5
						meters is used.
						
    divetodepth_timeout Timeout (in seconds) used in the DiveToDepth state.  If
                        not specified, a default value of 5.0 seconds is used.
                        						
	divetodepth_speed   Target speed (meters per second) used to create an 
                        objective function in the speed domain when the 
                        DiveToDepth state is running.  Speed values may be
                        negative or positive.  It is left to the vehicle 
                        platform or additional control via the divetodepth_flag
                        parameter to control the propeller direction to obtain 
                        reverse motion.  Default value is 5.0 m/sec.

    divetodepth_flag    One or more of these entries may be used to specify
                        variable-value pairs to be published on each Helm
                        iteration that the DiveToDepth state is running


                        
    levelout_pitch_deg  Target pitch (degrees) sought in the LevelOut state.  
                        If not specified, a default value of 0.0 is used.
                        
	levelout_timeout	Timeout (in seconds) used in the LevelOut state.  If
						not specified, a default value of 5.0 seconds is used.

    levelout_speed      Target speed (meters per second) used to create an 
                        objective function in the speed domain when the 
                        LevelOut state is running.  Speed values may be
                        negative or positive.  It is left to the vehicle 
                        platform or additional control via the levelout_flag
                        parameter to control the propeller direction to obtain 
                        reverse motion.  Default value is 1.0 m/sec.
                        
    levelout_flag       One or more of these entries may be used to specify
                        variable-value pairs to be published on each Helm
                        iteration that the LevelOut state is running



    error_flag          One or more of these entries may be used to specify
                        variable-value pairs to be published once when a dive
                        state fails.  A short description of the dive error
                        will always be published as a string to the MOOSDB  
                        variable DIVE_ERROR. 
						
    success_flag        One or more of these entries may be used to specify
                        variable-value pairs to be published once when a dive
                        completes successfully.  The string "true" is always
                        published to the MOOSDB variable DIVE_SUCCESS.
                        					
	is_simulation		A boolean string value ("true" or "false") specifying
						whether the behavior is running within a simulation.
						If this value is "true", the dive sequence will not be
						carried out, and the behavior will complete 
						successfully right away.  If not specified, the default 
						value of "false" will be used.
						
						

==================================
 PUBLISHED VARIABLES
==================================
    The following variables are always published by this behavior:
    
    Variable:           Description:
    -----------------   -----------------------------------------------------
    DIVE_STATE          A string containing the current dive state.  Updated
                        any time the state changes. 

    DIVE_ERROR          A string published on a dive failure.  Contains a short
                        description of the dive failure.

    DIVE_SUCCESS        Contains the string "true" when a dive has completed 
                        successfully

==================================
 SUBSCRIBED VARIABLES
==================================
    Variable:           Description:
    -----------------   -----------------------------------------------------
    NAV_PITCH           The current vehicle pitch in radians
    NAV_DEPTH           The current vehicle depth in meters
             

==================================
 EXAMPLE BEHAVIOR CONFIGURATION
==================================
