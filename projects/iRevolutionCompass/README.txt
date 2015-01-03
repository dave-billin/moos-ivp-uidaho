============================================================
README Documentation for iRevolutionCompass

Written by: Dave Billin
============================================================


===============================================================================
<<<  TABLE OF CONTENTS  >>>
===============================================================================
	1.) Description
	2.) Usage info
	3.) MOOS variables published by iRevolutionCompass
	4.) MOOS variables subscribed to by iRevolutionCompass
	5.) MOOS Mission file parameters
	6.) Build Dependencies
===============================================================================	


-------------------------------------------------------------
1.) Description

    iRevolutionCompass is a MOOS application that publishes measurements
    received from a True North Technologies Revolution Digital Compass module.


-------------------------------------------------------------
2.) Usage Info

USAGE:   iRevolutionCompass MISSION_FILE [APPNAME] 

MISSION_FILE
	A MOOS mission file containing runtime configuration parameters

APPNAME
	Alternate name the application will use when registering with
	the MOOS database.

Type 'iRevolutionCompass --help' for detailed usage information


		 
-------------------------------------------------------------
3.) MOOS VARIABLES PUBLISHED BY iRevolutionCompass

	Variable:				Description:
	--------------------	---------------------------------------------------
	REVCOMPASS_HEADING      Calculated 'true' heading received from the digital
	                        compass
	
	REVCOMPASS_DIP          Measured Dip angle in degrees
	
	REVCOMPASS_PITCH		Measured pitch (rotation about the lengthwise axis
							of the compass module) in degrees
	
	REVCOMPASS_ROLL			Measured roll (rotation about the lateral axis
							of the compass module) in degrees
							
	REVCOMPASS_MAGX         Magnetic field (in Teslas) measured along the X 
	                        (lengthwise) axis of the compass.

    REVCOMPASS_MAGY         Magnetic field (in Teslas) measured along the Y 
                            (lateral) axis of the compass.
	                        
    REVCOMPASS_MAGZ         Magnetic field (in Teslas) measured along the Z 
                            axis of the compass (perpendicular to the plane of
                            the compass module circuit board).	                        
	                        
	REVCOMPASS_ALARMS       A comma-separated list of strings indicating alarms
	 						being signaled by the compass module.  Entries in
	 						this list may include "Magnetometer", "Pitch", and
	 						"Roll".  When no alarms are being signaled, this 
	 						variable is published as an empty string.
	 						


	
-------------------------------------------------------------
5.) MOOS MISSION FILE PARAMETERS

	=============================
	 *** REQUIRED PARAMETERS ***
	=============================
	Parameter:					Description:
	--------------------		--------------------------------
    PORT                        Specifies the serial port device to use for
                                communicating with the compass module (e.g.
                                '/dev/ttyUSB2' under Linux or 'COMM1' under
                                Windows
								
 
	=============================
	 *** OPTIONAL PARAMETERS ***
	=============================
	
	Parameter:							Description
	----------------------------------	---------------------------------------
    BAUDRATE                    Specifies the baud rate used for serial 
                                communication with the compass module.  If not
                                specified, the default value of 19200 baud will
                                be used.  Baud rates supported by the compass
                                include 2400, 4800, 9600, and 19200.
                                
    DEVIATION                   A value in degrees that specifies the angle
                                between the magnetic meridian and the axis of
                                the compass.  Valid deviation angles range from  
                                -179 to 179 degrees with positive values 
                                resulting in a clockwise (Eastward) offset and 
                                negative values a counter-clockwise (Westward) 
                                offset.  If not specified, a deviation value of  
                                zero is used.

	HEADING_OFFSET				A floating-point offset in degrees that is added
								to the heading reported by the compass.  
								Positive values result in a clockwise(Eastward) 
								offset, while negative values result in a 
								counter-clockwise (Westward) offset.  If not 
								specified, a heading offset of zero is used.
								
    DECLINATION                 A value in degrees that specifies the magnetic
                                declination (angle between the geographic and
                                magnetic meridians) to use when calculating
                                heading.  The compass adds this value as an 
                                offset to its measured magnetic heading.  
                                Declination angle may range from -359.9 to 
                                359.9 degrees with positive values resulting
                                in a clockwise (Eastward) offset and negative
                                values a counter-clockwise (Westward) offset.  
                                If not specified, a value of zero is used.
                                 
    UPDATES_PER_SECOND          An integer value specifying the number of times
                                the compass should send the heading each 
                                second.  The APP_TICK parameter will 
                                automatically be adjusted to accommodate this 
                                rate to ensure timely processing of compass 
                                data.  Valid settings are 1, 2, 3, 5, 7, 10, and 
                                20.  If no rate or an invalid rate is specified, 
                                the default setting of 3 updates per second is
                                used.

    TILT_FILTER_TIME_CONSTANT   Time constant (seconds) of the filter used for 
                                smoothing tilt (pitch and roll) measurements.
                                Valid values range from 0 (no filtering) to
                                18.55.
                                
                                
    MAG_FILTER_TIME_CONSTANT    Time constant (seconds) of the filter used for 
                                smoothing the magnetic heading measurement.
                                Valid values range from 0 (no filtering) to
                                18.55.
                                
    ALARM_TIME_CONSTANT         Time constant (seconds) of the filter used to
                                smooth trigger signals for alarm conditions.
                                Valid values range from 0 (no filtering) to
                                18.55.

    MAGNETOMETER_GAIN           An integer gain index value from 0 to 255 that 
                                controls the gain applied to the compass 
                                module's magnetometer.  The actual gain value 
                                applied for index N can be calculated using the
                                relation:  G = 100 + 25600 / (2624 - 10*N)

    TILT_ALARM_THRESHOLD        Threshold angle magnitude (degrees) for pitch   
                                and roll, above which an alarm will be signaled 
                                in the corresponding field of the published
                                REVCOMPASS_STATUS variable.  When an alarm is
                                signaled, the corresponding measurement will be
                                clamped at the threshold magnitude.  Valid 
                                threshold values range from 0.0 to 45.0 degrees.

	PITCH_OFFSET				Offset (degrees) applied to pitch measurements.
								Valid settings range from 0 to 179.9 degrees.
								
	ROLL_OFFSET					Offset (degrees) applied to roll measurements.
								Valid settings range from 0 to 179.9 degrees.


	SET_BAUDRATE				If this integer value is specified, a command 
								will be sent to the compass module to configure 
								its serial baud rate.  The new baud rate will 
								not take effect until the compass module is 
								reset.  Supported baud rate settings are 2400, 
								4800, 9600, and 19200.
								

	VERBOSITY					Larger values of verbosity cause more debugging
								messages to be printed to stdio.  If not
								specified, the default setting of zero (no 
								debugging messages) will be used
								
								
    The following mission file parameters may be used to override the MOOS
    variables that pVehicleEKF publishes to

    Parameter                       Description
    -----------------------------   -------------------------------------------
    REVCOMPASS_HEADING_PUBLISHTO    Variable to publish compass heading to 
    REVCOMPASS_DIP_PUBLISHTO        Variable to publish Dip angle to
    REVCOMPASS_MAGX_PUBLISHTO       Variable to publish X-axis mag field to 
    REVCOMPASS_MAGY_PUBLISHTO       Variable to publish Y-axis mag field to
    REVCOMPASS_MAGZ_PUBLISHTO       Variable to publish Z-axis mag field to
    REVCOMPASS_ALARMS_PUBLISHTO		Variable to publish alarm status to
    
    EXAMPLE: To cause compass heading to be published to NAV_HEADING instead
             of REVCOMPASS_HEADING, the following line would be included in
             the iRevolutionCompass configuration block of a mission file.
             
             REVCOMPASS_HEADING_PUBLISHTO = NAV_HEADING


	

-------------------------------------------------------------
6.) BUILD DEPENDENCIES

   This application makes use of facilities in the YellowSubUtils library

