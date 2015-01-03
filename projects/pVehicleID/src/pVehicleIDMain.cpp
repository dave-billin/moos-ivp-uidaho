//=============================================================================
/*    Copyright (C) 2012  Brandt Pedrow

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
/*! @file MoosAppMain.cpp

@brief
	Execution entry point for the MOOS application

@author Brandt Pedrow

@par Created For
	The University of Idaho Microelectronics Research and Communications
	Institute (MRCI)
*/
//=============================================================================
#include "MOOS/libMOOS/MOOSLib.h"		// MOOS core libraries
#include <stdio.h>
#include <string.h>

#include "pVehicleID.h"

//================================
// Prototypes
//================================
void PrintUsageInfo(void);





//=============================================================================
/** Creates an instance of the application and runs it */
int main(int argc, char * argv[])
{
	const char * sMissionFile = "pVehicleID.moos";
	const char * sMOOSName = "pVehicleID";
	switch(argc)
	{
		// DB: This application is implemented as a singleton!  Don't support
		// running multiple instances under different process names!
		//case 3:
		//	sMOOSName = argv[2];

		case 2:
			sMissionFile = argv[1];
			break;

		default:
			PrintUsageInfo();
			return 0;
	}

	// Create and launch the application
	pVehicleID AppObject;
	AppObject.Run((char *) sMOOSName, (char *) sMissionFile);

	return 0;
}


void PrintUsageInfo( void )
{
	printf(	"\n\npMVehicleID version %0.2f\n"
			"Written by Brandt Pedrow\n"
			"USAGE:   pMissionManager MISSION_FILE\n"
			"\n"
			"MISSION_FILE\n"
			"\tMOOS mission file containing application parameters\n"
			"\n",
			APPLICATION_SOFTWARE_VERSION );
}
