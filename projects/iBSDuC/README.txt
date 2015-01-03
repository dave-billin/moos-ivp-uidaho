============================================================
README Documentation for iBSDuC

Written by: Dave Billin
============================================================


===============================================================================
<<<  TABLE OF CONTENTS  >>>
===============================================================================
	1.) Description
	2.) Usage info
	3.) MOOS variables published by iBSDuC
	4.) MOOS variables subscribed to by iBSDuC
	5.) MOOS Mission file parameters
	6.) Build Dependencies
===============================================================================	


-------------------------------------------------------------
1.) Description

	iBSDuC is a MOOS instrument for interfacing with sensors and actuator
	controls supplied by the PSOC microcontroller on the BSD CPU carrier 
	board in the University of Idaho MOOS AUV.


-------------------------------------------------------------
2.) Usage Info

USAGE:   iBSDuC MISSION_FILE [APPNAME] 

MISSION_FILE
	A MOOS mission file containing runtime configuration parameters

APPNAME
	Alternate name the application will use when registering with
	the MOOS database.


		 
-------------------------------------------------------------
3.) MOOS VARIABLES PUBLISHED BY iBSDuC

	Variable:				Description:
	--------------------	---------------------------------------------------
	BSD_IS_ONLINE           "TRUE" when a communication session with the BSD
	                        microcontroller is active, and "FALSE" otherwise.

    BSD_DEPTH_M             The current vehicle depth in meters based on the
                            reading from the pressure sensor connected to the  
                            BSD microcontroller

    BSD_DEPTH_ALARM         The string "TRUE" if the BSD microcontroller
                            indicates the depth sensor has failed; otherwise
                            the string "FALSE"
                            
    PROPELLER_RPM           Measured propeller RPM if the BSD microcontroller 
                            is online; zero otherwise

	WATER_IS_DETECTED       "TRUE" if the BSD microcontroller reports that 
	                        water is detected inside the AUV hull; "FALSE"
	                        otherwise

    PRESSURE_SENSOR_FAIL    "TRUE" if the BSD microcontroller is online and 
                            is reporting that the pressure sensor reading is
                            valid; "FALSE" otherwise
 
    RPM_VELOCITY_ESTIMATE   Estimated vehicle velocity based on propeller RPM
                            if the BSD microcontroller is online; zero 
                            otherwise

    BSD_BATTERY_VOLTS       Instantaneous battery voltage reading reported by 
                            the BSD microcontroller
                            
    BSD_BATTERY_AMPS        Instantaneous battery current reading reported by
                            the BSD microcontroller
                            
    BSD_BATTERY_DISCHARGED_AMPS       
                            Integrated battery power expended reported by the  
                            BSD microcontroller 

    BSD_BATTERY_TEMP_C      Temperature at the battery monitor reported by the
                            BSD microcontroller

    BSD_BATTERYMON_SERNUM   Battery monitor IC serial number reported by the
                            BSD microcontroller
                            
    BSD_FIRMWARE_VERSION    Firmware version reported by BSD microcontroller
    
    
-------------------------------------------------------------
3.) MOOS VARIABLES SUBSCRIBED TO BY iBSDuC

Variable:               Description:
--------------------    ---------------------------------------------------
NAV_HEADING             The current vehicle heading in degrees
NAV_SPEED               The current vehicle speed in meters/sec
NAV_DEPTH               The current vehicle depth in meters
NAV_PITCH               The current vehicle pitch in degrees
NAV_ROLL                The current vehicle roll in degrees


RUDDER_ANGLE_DEG
    Desired rudder angle in degrees (-85.0 to 85.0).  Each time a value is
    published to this variable, an actuator command is sent to the BSD
    microcontroller.  A value of 0.0 corresponds to the rudder null setting.  
    Positive values result in a positive yaw (i.e. a turn to starboard)

ELEVATOR_ANGLE_DEG    
    Desired elevator angle in degrees (-85.0 to 85.0).  Each time a value is
    published to this variable, an actuator command is sent to the BSD
    microcontroller.  A value of 0.0 corresponds to the elevator null setting.  
    Positive values result in a negative change in pitch, causing the vehicle 
    to dive.

AILERON_ANGLE_DEG
    Desired aileron angle in degrees (-85.0 to 85.0).  Each time a value is
    published to this variable, an actuator command is sent to the BSD
    microcontroller.  A value of 0.0 corresponds to the aileron null setting.
    Positive values result in a positive roll (i.e. a roll to port).

THRUST
    Desired thrust applied by the propeller as a component of full scale 
    (0.0 to 100.0).  Each time a value is published to this variable, an  
    actuator command is sent to the BSD microcontroller.  Positive values  
    result in forward thrust.  Negative values drive the propeller in reverse, 
    resulting in aft-ward thrust.


LEFT_ELEVATOR_TRIM_DEG   
    A double indicating the trim offset in degrees applied to the left 
    elevator control surface (-85.0 to +85.0). 

RIGHT_ELEVATOR_TRIM_DEG  
    A double indicating the trim offset in degrees applied to the left 
    elevator control surface (-85.0 to +85.0).
                            
RUDDER_TRIM_DEG
    A double indicating the trim offset in degrees applied to the rudder 
    control surface (-85.0 to +85.0).
                             
ACTUATOR_COUPLING_COEFFICIENT
    A double indicating the coupling coefficient applied to control surface 
    angles (0.0 to 1.0).


iBSDuC_CMD       
    A string used to issue one of the following commands to the BSD 
    microcontroller:
        "ResetBatteryMonitor"    Resets accumulated battery power
        
        "DisableProp"   Disables propeller thrust (generally useful 
                        for benchtop or out-of-water testing)
                         
        "EnableProp"    Re-enables propeller thrust after a 
                        DisableProp was issued 
	
	    "SoftReset"     Sends a command to reset the BSD microcontroller
	                    firmware (NOTE: as a safety measure, this command must 
	                    be received twice within a period of 5 seconds before
	                    the reset will be carried out)
	                    
-------------------------------------------------------------
5.) MOOS MISSION FILE PARAMETERS

	=============================
	 *** REQUIRED PARAMETERS ***
	=============================
	Parameter:					Description:
	--------------------		--------------------------------
    Port                        Serial port device connected to the BSD
                                microcontroller (e.g. "/dev/ttyS0")
								

	=============================
	 *** OPTIONAL PARAMETERS ***
	=============================

    The default MOOS variables that iBSDuC publishes to may be overridden by
    using one or more of the following mission file parameters and specifying
    an alternate value to publish to.  Alternately, the string DISABLED may be
    specified to disable publishing to any variable:

	Parameter:							Description
	------------------------------	---------------------------------------
    BSD_IS_ONLINE_PUBLISHTO         MOOS variable to get BSD microcontroller
                                    online status

    BSD_DEPTH_M_PUBLISHTO           MOOS variable to get BSD depth measurement

    PROPELLER_RPM_PUBLISHTO         MOOS variable to get propeller RPM

    WATER_IS_DETECTED_PUBLISHTO     MOOS variable to get water sensor status

    PRESSURE_SENSOR_FAIL_PUBLISHTO  MOOS variable to get depth sensor failure
     
    RPM_VELOCITY_ESTIMATE_PUBLISHTO MOOS variable to get RPM velocity estimate

    BATTERY_VOLTS_PUBLISHTO         MOOS variable to get PSU battery voltage
                            
    BATTERY_AMPS_PUBLISHTO          MOOS variable to get PSU battery amps
                            
    BATTERY_WATTS_PUBLISTO          MOOS variable to get expended power 


    The default MOOS variables that iBSDuC subscribes to may be overridden by
    using one or more of the following mission file parameters and specifying
    an alternate value to subscribe to:

    RUDDER_ANGLE_DEG_SUBSCRIBETO    MOOS variable to subscribe to for rudder
                                    angle

    ELEVATOR_ANGLE_DEG_SUBSCRIBETO  MOOS variable to subscribe to for elevator
                                    angle
                                    
    AILERON_ANGLE_DEG_SUBSCRIBETO   MOOS variable to subscribe to for aileron
                                    angle

    THRUST_SUBSCRIBETO              MOOS variable to subscribe to for thrust
    
    LEFT_ELEVATOR_TRIM_DEG_SUBSCRIBETO   MOOS variable to subscribe to for
                                         left elevator trim
                                        
    RIGHT_ELEVATOR_TRIM_DEG_SUBSCRIBETO  MOOS variable to subscribe to for
                                         right elevator trim
                                         
    RUDDER_TRIM_DEG_SUBSCRIBETO          MOOS variable to subscribe to for
                                         rudder trim
                                         
    ACTUATOR_COUPLING_COEFFICIENT_SUBSCRIBETO   MOOS variable to subscribe to
                                                for actuator coupling 
                                                coefficient
               
-------------------------------------------------------------
6.) BUILD DEPENDENCIES

    YellowSubUtils
	
