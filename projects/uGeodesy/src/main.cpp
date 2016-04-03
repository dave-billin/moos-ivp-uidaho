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
/** @file uGeodesy/main.cpp
 *
 * @brief
 *  Execution entry point for the uGeodesy application
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================
#include "config.h"
#include <MOOS/libMOOS/MOOSLib.h>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <iostream>

using std::cout;
using std::endl;
using std::string;

static std::string APPLICATION_VERSION(APP_VERSION_TUPLE);


/** @defgroup uGeodesy
 * @{
 */

//================================
// Function Prototypes
//================================
void PrintUsageInfo(void);
void PrintExampleConfig(void);
bool ParseMissionFileParameters( CProcessConfigReader& MissionReader );


//=============================================================================
/** Creates an instance of the application and runs it */
int main(int argc, char* argv[])
{
	const char* szMissionFile = NULL;
	const char* szMOOSName = "uGeodesy";

	// Print usage info and exit if no command line arguments were supplied
	if (argc == 1)
	{
		cout << "\nUSAGE:   uGeodesy [OPTIONS] MISSION_FILE [APPNAME]\n"
             << "For more info, use 'uGeodesy --help' or 'man "
                "uGeodesy'\n"
             << endl;

		return -1;
	}

	// Parse command line arguments for options or mission file
	if (argc >= 2)
	{
        //--------------------------------
		// Parse command-line switches
        //--------------------------------
		if ( MOOSStrCmp(argv[1], "-e") || MOOSStrCmp(argv[1], "--example") )
		{
            PrintExampleConfig();
			return 0;
		}
		else if ( MOOSStrCmp(argv[1], "-h") || MOOSStrCmp(argv[1], "--help") )
		{
            // Print usage info
			PrintUsageInfo();
			return 0;
		}
        else if ( MOOSStrCmp(argv[1], "-v") || MOOSStrCmp(argv[1], "--version") )
        {
            // Print application version
            cout << "uGeodesy v" << APPLICATION_VERSION << "\n" << endl;
            return 0;
        }
		else	// No command line options, must be a mission file path
		{
			szMissionFile = argv[1];
		}
	}

	// Grab alternate name to use when registering if specified
	if (argc >= 3)
	{
		szMOOSName = argv[2];
	}

	CProcessConfigReader MissionReader;
	MissionReader.SetFile( string(szMissionFile) );
	MissionReader.SetAppName( string(szMOOSName) );

	return (ParseMissionFileParameters(MissionReader)) ? 0 : -1;
}




//=============================================================================
/** Prints the application's command-line usage info */
void PrintUsageInfo( void )
{
	cout <<
    "\nuGeodesy v" << APPLICATION_VERSION << "\n"
    "Written by Dave Billin (david.billin@vandals.uidaho.edu)\n"
    "\n"
    "USAGE:   uGeodesy [OPTIONS] MISSION_FILE\n"
    "\n"
    "OPTIONS\n"
    "   -e,--example  Prints an example mission file configuration block\n"
    "   -h,--help     Prints application usage information\n"
    "   -v,--version  Displays application version\n"
    "\n"
    "MISSION_FILE\n"
    "   MOOS mission file containing geodesy origin and points to operate on\n"
    << endl;
}



//=============================================================================
/** Prints an example MOOS mission file configuration block */
void PrintExampleConfig(void)
{
    // Print an example mission file configuration
    cout << "\n\n"
    	 << "** Example Mission File Configuration for uGeodesy **\n"
    	 << "\n"
    	 << "// Origin of geodesy must be in global scope of mission file\n"
    	 << "OriginLat = 47.978100\n"
    	 << "OriginLon = -116.537103\n"
    	 << endl
         << "ProcessConfig = uGeodesy\n"
         << "{\n"
         << "    // Declare a point specified using local coordinates\n"
         << "    POINT:NAME=LocalCoordPoint,North=20,East=72\n"
         << endl
         << "    // Declare a point specified using earth coordinates\n"
    	 << "    POINT:NAME=EarthCoordPoint,Lon=-116.537718,Lat=47.979337\n"
    	 << "}\n"
         << endl;
}



//=============================================================================
bool ParseMissionFileParameters( CProcessConfigReader& MissionReader )
{
	double OriginLatitude, OriginLongitude;
	CMOOSGeodesy Geo;
	double TargetLatitude, TargetLongitude;	// Target point's lat/long
	double TargetEastings, TargetNorthings;	// Target point's X/Y
	string sErrorReport;	// Used to report point specification errors

    //------------------------------------------
    // LOAD REQUIRED MISSION FILE PARAMETERS
    //------------------------------------------
	// Make sure a geodesy origin is specified in the app configuration block
	if ( MissionReader.GetValue("LatOrigin", OriginLatitude) &&
		 MissionReader.GetValue("LongOrigin", OriginLongitude)  )
	{
		Geo.Initialise(OriginLatitude, OriginLongitude);
	}
	else
	{
		return MOOSFail("Missing one or more origin coordinates: "
						"LatOrigin, LongOrigin!\n");
	}


	// Load all parameters from the app's mission file block
    STRING_LIST sAppParams;
    MissionReader.GetConfiguration("uGeodesy", sAppParams);
    STRING_LIST::reverse_iterator p;


    // Print geodesy parameters
    string sBar = string(80, '-') + "\n";
    MOOSTrace("\n\n" + sBar);
    MOOSTrace("<<Geodesy Origin>>  "
    		  "Longitude: %-10.8f    "
    		  "Latitude:  %-10.8f\n",
    		  OriginLongitude, OriginLatitude);
    MOOSTrace(sBar);

    MOOSTrace("\n\n" + sBar);
    MOOSTrace( "|        NAME          |  Longitude, Latitude (°)   |  "
    		   "East(x), North(y) (m)\n", "NAME");
    MOOSTrace(sBar);
    // Parse lines containing strings that specify a location using the format:
    // "POINT:"NAME=abcd,NORTH=1234,EAST=1234" or
    // "POINT:"NAME=abcd,LAT=1234,LONG=1234"
    for ( p = sAppParams.rbegin(); p != sAppParams.rend(); p++ )
    {
    	// Parse parameter/statement on the current line
        string sLine = *p;		// Get the current line of the app params
        string sPoint = MOOSChomp(sLine,":");	// "POINT" for a point spec
        string sValueString = sLine;	// RHS of "Point:<PARAMS>" spec
        MOOSToUpper(sPoint);

        // If the current line doesn't start with "POINT:", don't process it.
        if ( !MOOSStrCmp(sPoint, "POINT") )
        {
        	continue;
        }

        string sName = "<Unnamed Point>";
        bool PointSpecIsValid = false;

        if ( !MOOSValFromString( sName, sValueString, "NAME", true ) )
        {
        	sErrorReport += "  " + *p + "\n";
        	continue;
        }

        if ( MOOSValFromString(TargetEastings, sValueString, "NORTH", true) &&
        	 MOOSValFromString(TargetNorthings, sValueString, "EAST", true) )
        {
        	Geo.LocalGrid2LatLong( TargetEastings, TargetNorthings,
        						   TargetLatitude, TargetLongitude );
        	PointSpecIsValid = true;
        }
        else if ( MOOSValFromString(TargetLatitude, sValueString, "LAT", true) &&
        		  MOOSValFromString(TargetLongitude, sValueString, "LONG", true) )
        {
        	Geo.LatLong2LocalGrid( TargetLatitude, TargetLongitude,
        						   TargetNorthings, TargetEastings );
        	PointSpecIsValid = true;
        }

        if (PointSpecIsValid)
        {
			// Print coordinates of the given point
			MOOSTrace("| %-20s | %-12f, %-12f | %-10f, %-10f\n",
					  sName.c_str(), TargetLongitude, TargetLatitude,
					  TargetEastings, TargetNorthings);
        }
        else
        {
        	sErrorReport += "  " + *p + "\n";
        }
    }
    // Loop through application parameters and process them

    MOOSTrace("\n\n");

    int Rc;

    if (!sErrorReport.empty())
    {
    	MOOSTrace("** The following point specifications in the mission file "
    			  "contain errors **\n" +
    			  sErrorReport);
    	MOOSTrace("\n\n");
    	Rc = false;
    }
    else
    {
    	Rc = true;
    }

	return Rc;
}


/** @} */   // END @defgroup uGeodesy

