============================================================
README Documentation for LibYellowSubNav

Written by: Dave Billin
============================================================


===============================================================================
<<<  TABLE OF CONTENTS  >>>
===============================================================================
	1.) Description
	2.) Usage info
	3.) MOOS variables published by pPositionAgent
	4.) MOOS variables subscribed to by pPositionAgent
	5.) Mission file parameters
	6.) Build Dependencies
===============================================================================	


-------------------------------------------------------------
1.) Description

	LibYellowSubNav is a static library containing classes and functions used
	for navigation on the U of I autonomous underwater vehicle (AUV).  The 
	library is compiled to the default MOOS binary folder (MOOSBin).  To link
	to LibYellowSubNav, simply include it in the TARGET_LINK_LIBRARIES 
	arguments of your application's CMakeLists.txt configuration, and include
	"LibYellowSubNav.h" in your source code.

	
-------------------------------------------------------------
6.) BUILD DEPENDENCIES

	LibYellowSubNav depends on the NewMat matrix library found in the 
	Thirdparty folder in the MOOS directory.

