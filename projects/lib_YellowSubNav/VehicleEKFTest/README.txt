============================================================
README Documentation for VehicleEKFTest

Written by: Dave Billin
============================================================


===============================================================================
<<<  TABLE OF CONTENTS  >>>
===============================================================================
	1.) Description
	2.) Usage info
	6.) Build Dependencies
===============================================================================	


-------------------------------------------------------------
1.) Description

	VehicleEKFTest is a test driver application used to verify the output of
	the LibYellowSubNav VehicleEKF class matches that produced by the 
	corresponding Dynamic C code from the Rabbit-based AUV.

-------------------------------------------------------------
1.) Usage info

	USAGE: VehicleEKFTest [BINARY_LOG_FILE]

	Where BINARY_LOG_FILE is a file containing timestamped binary packets from 
	the Rabbit AUV log.

	
-------------------------------------------------------------
6.) BUILD DEPENDENCIES

	LibYellowSubNav depends on the NewMat matrix library found in the 
	Thirdparty folder in the MOOS directory and on the LibYellowSubNav
	found in the default MOOS binary folder (MOOSBin).

