============================================================
README Documentation for iArchangelIMU

Written by: Dave Billin
============================================================


===============================================================================
<<<  TABLE OF CONTENTS  >>>
===============================================================================
	1.) Description
	2.) Usage info
	3.) MOOS variables published by iScotty
	4.) MOOS variables subscribed to by iScotty
	5.) MOOS Mission file parameters
	6.) Build Dependencies
===============================================================================	


-------------------------------------------------------------
1.) Description

	iArchangelIMU is a MOOS instrument to publish data from an Archangel IM^3
	Inertial Measurement Unit (IMU)


-------------------------------------------------------------
2.) Usage Info

USAGE:   iArchangelIMU MISSION_FILE [APPNAME] 

MISSION_FILE
	A MOOS mission file containing runtime configuration parameters

APPNAME
	Alternate name the application will use when registering with
	the MOOS database.


		 
-------------------------------------------------------------
3.) MOOS VARIABLES PUBLISHED BY iArchangelIMU

	Variable:				Description:
	--------------------	---------------------------------------------------
	IMU_DELTA_ROLL_ANGLE	Delta roll angle (degrees) from 'IMU' messages
	IMU_DELTA_PITCH_ANGLE	Delta pitch angle (degrees) from 'IMU' messages
	IMU_DELTA_YAW_ANGLE		Delta yaw angle (degrees) from 'IMU' messages
	IMU_DELTAV_LONGITUDINAL	Longitudinal velocity (m/s) from 'IMU' messages
	IMU_DELTAV_LATERAL		Latitudinal velocity (m/s) from 'IMU' messages
	IMU_DELTAV_NORMAL		Normal velocity (m/s) from 'IMU' messages
	
	IMU_INERTIAL_ROLL		Inertial roll angle (degrees) from 'AHRS' messages
	IMU_INERTIAL_PITCH		Inertial pitch angle (degrees) from 'AHRS' messages
	IMU_INERTIAL_YAW		Inertial yaw angle (degrees) from 'AHRS' messages
	IMU_INERTIAL_ROLLRATE	Inertial roll rate (deg/s) from 'AHRS' messages
	IMU_INERTIAL_PITCHRATE	Inertial pitch rate (deg/s) from 'AHRS' messages
	IMU_INERTIAL_YAWRATE	Inertial yaw rate (deg/s) from 'AHRS' messages

	IMU_ERRORFLAGS			A string containing one or more comma-separated
							status flags that may be set by the IMU.  Flags 
							include:
							
							"GyroX" IMU self-test error: Gyro X1
							"GyroY" IMU self-test error: Gyro Y1
							"GyroZ" IMU self-test error: Gyro Z1
							"AccX1" IMU self-test error: Accelerometer X1
							"AccY1" IMU self-test error: Accelerometer Y1
							"AccZ1" IMU self-test error: Accelerometer Z1
							"AccX1" IMU self-test error: Accelerometer X2
							"AccY1" IMU self-test error: Accelerometer Y2
							"AccZ1" IMU self-test error: Accelerometer Z2
							"TempX" IMU self-test error: TempX
							"TempY" IMU self-test error: TempY
							"TempZ" IMU self-test error: TempZ
							
							"Aligning"	IMU is running its alignment routine to
										remove sensor bias


	IMU_CONNECTED			A string value: "TRUE" if data has been received
							from the IMU3 module within the past 5 seconds;
							otherwise "FALSE".
	
	
	
-------------------------------------------------------------
5.) MOOS MISSION FILE PARAMETERS

	=============================
	 *** REQUIRED PARAMETERS ***
	=============================
	Parameter:					Description:
	--------------------		--------------------------------
	Port 						Serial port device connected to the Archangel
								IMU3 module (e.g. "/dev/ttyS0" under Linux or
								"COM1" under Windows)
								
	BaudRate					Serial baud rate (this must be set to 460800)

								

	=============================
	 *** OPTIONAL PARAMETERS ***
	=============================
	The MOOS variables that IMU measurements are published to may be overridden
	using one or more of the following mission file parameters to specify an
	alternate value to publish to:
	
	Parameter:							Description
	----------------------------------	---------------------------------------
	IMU_DELTA_ROLL_ANGLE_PUBLISHTO		MOOS variable to get delta roll angle
	IMU_DELTA_PITCH_ANGLE_PUBLISHTO		MOOS variable to get delta pitch angle
	IMU_DELTA_YAW_ANGLE_PUBLISHTO		MOOS variable to get delta yaw angle
	IMU_DELTAV_LONGITUDINAL_PUBLISHTO	MOOS variable to get Lon. velocity
	IMU_DELTAV_LATERAL_PUBLISHTO		MOOS variable to get Lat. velocity
	IMU_DELTAV_NORMAL_PUBLISHTO			MOOS variable to get Normal velocity
	
	IMU_INERTIAL_ROLL_PUBLISHTO			MOOS variable to get inertial roll angle
	IMU_INERTIAL_PITCH_PUBLISHTO		MOOS variable to get inertial pitch angle
	IMU_INERTIAL_YAW_PUBLISHTO			MOOS variable to get inertial yaw angle
	IMU_INERTIAL_ROLLRATE_PUBLISHTO		MOOS variable to get inertial roll rate
	IMU_INERTIAL_PITCHRATE_PUBLISHTO	MOOS variable to get inertial pitch rate
	IMU_INERTIAL_YAWRATE_PUBLISHTO		MOOS variable to get inertial yaw rate



	INVERT_DELTA_ROLL_ANGLE	 	If set to TRUE, the sign of measurements   
							 	published to the MOOS variable 
							 	IMU_DELTA_ROLL_ANGLE will be inverted
							
	INVERT_DELTA_PITCH_ANGLE	If set to TRUE, the sign of measurements   
                                published to the MOOS variable 
                                IMU_DELTA_PITCH_ANGLE will be inverted
							 	
	INVERT_DELTA_YAW_ANGLE		If set to TRUE, the sign of measurements   
                                published to the MOOS variable 
                                IMU_DELTA_YAW_ANGLE will be inverted


	INVERT_DELTAV_LONGITUDINAL	If set to TRUE, the sign of measurements   
                                published to the MOOS variable 
                                IMU_DELTAV_LONGITUDINAL will be inverted
							 	
	INVERT_DELTAV_LATERAL		If set to TRUE, the sign of measurements   
                                published to the MOOS variable 
                                IMU_DELTAV_LATERAL will be inverted

	INVERT_DELTAV_NORMAL		If set to TRUE, the sign of measurements   
                                published to the MOOS variable 
                                IMU_DELTAV_NORMAL will be inverted
							 	

	INVERT_INERTIAL_ROLL		If set to TRUE, the sign of measurements  
								published in the MOOS variables 
								IMU_INERTIAL_ROLL and IMU_INERTIAL_ROLLRATE 
								will be inverted
							
	INVERT_INERTIAL_PITCH		If set to TRUE, the sign of measurements  
                                published in the MOOS variables 
                                IMU_INERTIAL_PITCH and IMU_INERTIAL_PITCHRATE 
                                will be inverted

	INVERT_INERTIAL_YAW			If set to TRUE, the sign of measurements  
                                published in the MOOS variables 
                                IMU_INERTIAL_YAW and IMU_INERTIAL_YAWRATE 
                                will be inverted
							
-------------------------------------------------------------
6.) BUILD DEPENDENCIES

	YellowSubUtils, MOOS, and system libraries
	
