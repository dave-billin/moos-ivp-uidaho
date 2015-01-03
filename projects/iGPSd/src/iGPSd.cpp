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
/** @file iGPSd.cpp
 *
 * @brief
 *  Implementation of the iGPSd application class
 *
 * @author Dave Billin
 *         david.billin@vandals.uidaho.edu
 */
//=============================================================================
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSAssert.h"

#include "iGPSd.h"
#include "AppVersion.h"

using std::string;
using std::cout;

#ifdef USE_GOOGLE_PROTOCOL_BUFFERS
#include "iGPSd_FixData.pb.h"
using namespace iGPSdProtobuf;
#endif

// Default variable names IMU values will be published to
const char* iGPSd::sm_DefaultPublishedVarNames[NUM_PUBLISHED_VARIABLES] =
            {
              "FIX_IS_VALID",              /* FIX_IS_VALID */
              "FIX_MODE",                  /* FIX_MODE */
              "LATITUDE_DEG",              /* LATITUDE_DEG */
              "LONGITUDE_DEG",             /* LONGITUDE_DEG */
              "HEADING_DEG",               /* HEADING_DEG */
              "GROUND_SPEED_MPS",          /* GROUND_SPEED_MPS */
              "ALTITUDE_M",                /* ALTITUDE_M */
              "VERTICAL_SPEED_MPS",        /* VERTICAL_SPEED_MPS */
              #ifdef USE_GOOGLE_PROTOCOL_BUFFERS
              "PBREPORT",/* UNCERTAINTY_VERTICAL_SPEED*/
              #endif
            };



// Character string containing an example mission file configuration
// block for the application
const char iGPSd::ExampleMissionFileConfiguration[]=
    "ProcessConfig = iGPSd\n"
    "{\n"
    "    // REQUIRED PARAMETERS\n"
    "\n"
    "    // OPTIONAL PARAMETERS\n"
    "    AppTick   = 100    // Frequency (Hz) at which Iterate() runs\n"
    "    CommsTick = 100    // Frequency (Hz) of MOOS database updates\n"
    "\n"
    "    VERBOSITY = 0      // Verbosity of debugging messages\n"
    "\n"
    "    GPSD_HOSTNAME = localhost  // Hostname of GPSd server\n"
    "    GPSD_PORT     = 2947       // TCP port (default is 2947)\n"
    "}\n"
    "\n";



//=============================================================================
/* 
    This function is called when the application object is created in main().
    This is a good place to initialize application variables to default
    values, and set up dynamic memory resources.  

    NOTE: This function executes as the application object is being created, 
          so it's best to save activities that might require error handling
          (e.g. opening files or I/O devices) for the OnStartup() method.
*/
iGPSd::iGPSd( void )
:   m_gpsdclient(NULL),
    m_Verbosity(0),
    m_OnStartupIsDone(false),
    m_ServerHostname("127.0.0.1"),
    m_ServerPort(2947)
{

}



//=============================================================================
/* 
    This function is called when the application object is exiting.  Make sure
    you release any resources your program allocated!

    NOTE: There is no need to explicitly delete Dynamic Variables.  They get
          cleaned up automatically.
*/
iGPSd::~iGPSd()
{
    DaemonDisconnect();
}




//=============================================================================
/* 
    This is where your application will probably do most of its work.  Some
    handy methods you might want to use include:

    Get the current system time:
        MOOSTime()

    Check whether the MOOS database is connected:
        m_Comms.IsConnected()

    Publish a value directly to the MOOS database:
        m_Comms.Notify()

    Set or get the value of a Dynamic Variable:
        SetMOOSVar() and GetMOOSVar()

    Publish Dynamic Variable values that have changed to the MOOS database:
        PublishFreshMOOSVariables()

    Publish a debugging message to the MOOS database variable MOOS_DEBUG:
        MOOSDebugWrite()

    Set the error state and message in the iGPSd_STATUS variable:
        SetAppError()

    NOTE: returning false will cause the application to exit.
*/
bool iGPSd::Iterate( void )
{
    // If no connection to the GPS daemon exists, attempt to open one
    if ( !DaemonIsConnected() )
    {
        if ( !DaemonConnect() )
        {
            // Pause for 1 sec between connection attempts
            MOOSPause(1000, false);
            return true;
        }
    }


    // Receive and process reports from the GPS daemon
    struct gps_data_t* GpsData;

    // Block for up to 10 milliseconds waiting for GPS data
    if ( !m_gpsdclient->waiting(10000) )
    {
        return true;    // Do nothing if no data was received.
    }

    // STATUS:
    //  Received data from the GPS daemon is available
    //  Read and process it
    if ( (GpsData = m_gpsdclient->read()) == NULL)
    {
        DaemonDisconnect();
    }
    else
    {
        // Publish GPS fix data received from the GPS daemon
        PublishGpsData( *GpsData );
    }

	return true;
}






//=============================================================================
/*
    Here is where the application should handle mail received from the MOOS 
    database.  This mail takes the form of CMOOSMsg objects - each of which
    contains the name and contents of a MOOS variable the application has
    subscribed to.

    NOTE: returning false will cause the application to exit.
*/
bool iGPSd::OnNewMail(MOOSMSG_LIST & NewMail)
{
    //-------------------------------------------
    // Update dynamic variables with values 
    // received from the MOOS database
    //-------------------------------------------
    UpdateMOOSVariables(NewMail);


    // If your application isn't using dynamic variables, or if 
    // you would prefer to handle messages individually, un-comment
    // the following code to iterate through the received mail
    /*
    MOOSMSG_LIST::iterator iter;
    double t = MOOSTime();      // Current system time
    for (iter = NewMail.begin(); iter != NewMail.end(); iter++)
    {
	    CMOOSMsg& Msg = *iter;

        // STATUS: Msg contains a mail message from the MOOS database.
        // Note that this message may be skewed (i.e. it may have been 
        // published to the database more than a few seconds ago)

	    // Here we process only messages that are not skewed
	    if( !RxMsg.IsSkewed(t) )
	    {
		    // <<< Process only un-skewed messages here >>>

		    continue;
	    }

        // <<< Process skewed messages here >>>

    }
    */

	return true;
}









//=============================================================================
/*
    This function is called once when the application starts running.  The 
    first thing it does is call to LoadMissionParameters() to read parameters
    from the (.moos) mission file.

    This function is a good spot to open files, connect to I/O devices, or 
    perform other actions that might result in an error condition that must be
    handled gracefully.

    This is also a good place to create Dynamic Variables (which can simplify 
    the process of publishing and subscribing to the MOOS database) using 
    AddMOOSVariable().

    NOTE: returning false will cause the application to exit.
*/
bool iGPSd::OnStartUp( void )
{

    //---------------------------------
	// Print a startup banner
    //---------------------------------
	string sAppName = GetAppName();
	string sBar = string(40, '=') + "\n";
	MOOSTrace(sBar +
			  MOOSFormat("iGPSd version %s\n", 
                         APP_VERSION_STRING) +
			  "Written by Dave Billin\n" + sBar + "\n\n");


    //---------------------------------
	// Load mission file parameters.
    //---------------------------------
	if ( LoadMissionFileParameters() == false )
	{
		return false;
	}

    //----------------------------------------------------
    // Display re-mapped and disabled published variables
    //----------------------------------------------------
	string sRemapped, sDisabled;
	for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
	{
	    string s = MOOSFormat( "GPS_%s", sm_DefaultPublishedVarNames[i] );

	    if (s != m_PublishedVarNames[i])
	    {
	        // Disabled variables
	        if ( m_PublishedVarNames[i].empty() )
	        {
	            sDisabled += s + "\n";
	        }
	        else
	        {
	            sRemapped += s + " ==> " + m_PublishedVarNames[i];
	        }
	    }
	}

    if ( !sRemapped.empty() )
    {
        MOOSTrace( sBar +
                   "\nPublished variables that have been re-mapped:\n" +
                   sBar + "\n" +
                   sRemapped + "\n\n" );
    }

    if ( !sDisabled.empty() )
    {
        MOOSTrace( sBar +
                   "\nPublished variables that have been disabled:\n" +
                   sBar + "\n" +
                   sDisabled + "\n\n" );
    }

    // Register Dynamic Variables to receive 
    // updates from the MOOS database
    m_OnStartupIsDone = true;
    
    return true;
}







//=============================================================================
/*
    This function gets called from OnStartup() to load parameters from the
    (.moos) mission file.  Call: 

    m_MissionReader.GetConfigurationParam()
        to read the value of a mission file parameter occurring in the 
        application's "ProcessConfig = iGPSd" block

    m_MissionReader.GetValue()
        to load a global scope parameter (i.e. one occuring outside a
        ProcessConfig block)

    NOTE: There is no need to load the AppTick and CommsTick parameters;  
          they are automatically loaded.

    Returning false from this function causes the application to exit.
*/
bool iGPSd::LoadMissionFileParameters( void )
{
    string s;
    
    //------------------------------------------
    // LOAD REQUIRED MISSION FILE PARAMETERS
    //------------------------------------------


    //------------------------------------------
    // LOAD OPTIONAL MISSION FILE PARAMETERS
    //------------------------------------------

	// Get the verbosity level used for debugging messages
	m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);

    // Load the hostname of the GPSd server
    m_MissionReader.GetConfigurationParam("GPSD_HOSTNAME", m_ServerHostname);

    // Load the TCP port to connect to on the GPSd server
    uint32_t UintVal;
    if ( m_MissionReader.GetConfigurationParam("GPSD_PORT", UintVal) )
    {
        m_ServerPort = static_cast<uint16_t>(UintVal);
    }

    //------------------------------------------
    // Load published variable re-mapping
    //------------------------------------------
    for (int i = 0; i < NUM_PUBLISHED_VARIABLES; i++)
    {
        // Load default published variable name
        m_PublishedVarNames[i] = MOOSFormat( "GPS_%s",
                                             sm_DefaultPublishedVarNames[i] );

        // Load alternate name.  If no "_PUBLISHTO" parameter is present,
        // the default variable name will remain unchanged
        s = MOOSFormat("%s_PUBLISHTO", sm_DefaultPublishedVarNames[i] );
        m_MissionReader.GetConfigurationParam(s, m_PublishedVarNames[i]);

        if ( MOOSStrCmp( m_PublishedVarNames[i], "NOT_PUBLISHED") )
        {
            m_PublishedVarNames[i].clear();
        }
    }

	return true;
}






//=============================================================================
/*
    This function gets called when the application connects to the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iGPSd::OnConnectToServer( void )
{
    // If OnStartup has already run, (re-)register Dynamic Variables
    // to receive updates from the MOOS database
    if (m_OnStartupIsDone)
    {
        RegisterMOOSVariables();
    }

    // << Perform any tasks associated with MOOSDB connection >>

	return true;
}



//=============================================================================
/*
    This function gets called when the application disconnects from the MOOS 
    database.  BE CAREFUL! this function gets called on a different thread 
    than your Iterate() and OnNewMail() handlers!
*/
bool iGPSd::OnDisconnectFromServer( void )
{
    // << Perform any tasks associated with MOOSDB disconnection >>

	return true;
}


//=============================================================================
bool iGPSd::DaemonConnect( void )
{
    // If no connection to the GPS daemon exists, attempt to open one
    if ( m_gpsdclient == NULL )
    {
        // Attempt to open a connection to the GPS daemon
        m_gpsdclient = new gpsmm( "localhost", DEFAULT_GPSD_PORT );

        if ( m_gpsdclient->stream(WATCH_ENABLE|WATCH_JSON) == NULL )
        {
            // No GPSd running.  Delete the gpsmm instance to release
            // connection resources (kinda lame to be mucking around
            // on the heap, but that's the way that gpsmm is written)...
            delete m_gpsdclient;
            m_gpsdclient = NULL;
        }
    }

    return (m_gpsdclient != NULL);
}


//=============================================================================
void iGPSd::DaemonDisconnect( void )
{
    if ( m_gpsdclient != NULL )
    {
        delete m_gpsdclient;
        m_gpsdclient = NULL;
    }
}


//=============================================================================
void iGPSd::PublishGpsData( struct gps_data_t& GpsData )
{
    static const char* szFixModes[] = { "not seen", /* MODE_NOT_SEEN */
                                        "none",     /* MODE_NO_FIX */
                                        "2D",       /* MODE_2D */
                                        "3D" };     /* MODE_3D */

    gps_fix_t& GpsFix = GpsData.fix;
    string s;

    // Publish GPS_FIX_IS_VALID
    s = (GpsData.status < STATUS_FIX) ? "TRUE" : "FALSE";
    NotifyIfEnabled( m_PublishedVarNames[FIX_IS_VALID], s, GpsFix.time );

    // If fix data is available, publish it
    if ( GpsData.status != STATUS_NO_FIX )
    {
        // Publish GPS_FIX_MODE
        s = ( (GpsFix.mode >= MODE_NOT_SEEN) && (GpsFix.mode < MODE_3D) ) ?
                                szFixModes[GpsFix.mode] : szFixModes[MODE_NO_FIX];
        NotifyIfEnabled( m_PublishedVarNames[FIX_MODE], s, GpsFix.time );

        // Publish GPS_LATITUDE_DEG
        NotifyIfEnabled( m_PublishedVarNames[LATITUDE_DEG],GpsFix.latitude,
                         GpsFix.time );

        // Publish GPS_LONGITUDE_DEG
        NotifyIfEnabled( m_PublishedVarNames[LONGITUDE_DEG],GpsFix.longitude,
                         GpsFix.time );

        // Publish GPS_HEADING_DEG
        NotifyIfEnabled( m_PublishedVarNames[HEADING_DEG],GpsFix.track,
                         GpsFix.time );

        // Publish GPS_GROUND_SPEED_MPS
        NotifyIfEnabled( m_PublishedVarNames[GROUND_SPEED_MPS],GpsFix.track,
                         GpsFix.time );

        // Publish 3D fix variables
        if ( GpsFix.mode == MODE_3D )
        {
            // Publish GPS_ALTITUDE_M
            NotifyIfEnabled( m_PublishedVarNames[ALTITUDE_M],GpsFix.altitude,
                             GpsFix.time );

            // Publish GPS_VERTICAL_SPEED_MPS
            NotifyIfEnabled( m_PublishedVarNames[VERTICAL_SPEED_MPS],
                             GpsFix.climb, GpsFix.time );
        }
    }

    //----------------------------------
    // Publish serialized Fix report
    //----------------------------------
    #ifdef USE_GOOGLE_PROTOCOL_BUFFERS
    iGPSd_Report pbReport;

    if ( GpsData.status != STATUS_NO_FIX )
    {
        pbReport.set_havefix( true );
        pbReport.clear_fixdata();
    }
    else
    {
        pbReport.set_havefix( false );
        iGPSd_Report::GPSFixData* FixData = pbReport.mutable_fixdata();

        FixData->set_fixmode(
           static_cast<iGPSd_Report::GPSFixData::e_GPS_Fix_Mode>(GpsFix.mode));

        FixData->set_numsatellites( GpsData.satellites_used );
        FixData->set_unixtimestamp( GpsFix.time );
        FixData->set_latitude_deg( GpsFix.latitude );
        FixData->set_longitude_deg( GpsFix.longitude );
        FixData->set_heading_deg( GpsFix.track );
        FixData->set_groundspeed_mps( GpsFix.speed );
        FixData->set_altitude_deg( GpsFix.altitude );
        FixData->set_verticalspeed_mps( GpsFix.climb );

        FixData->set_eposition( GpsData.epe );
        FixData->set_etime( GpsFix.ept );
        FixData->set_elatitude( GpsFix.epy );
        FixData->set_elongitude( GpsFix.epx );
        FixData->set_eheading( GpsFix.epd );
        FixData->set_egroundspeed( GpsFix.eps );
        FixData->set_ealtitude( GpsFix.epv );
        FixData->set_everticalspeed( GpsFix.epc );
    }

    // Serialize the report object and publish it to GPS_PBREPORT
    pbReport.SerializeToString( &s );
    NotifyIfEnabled( m_PublishedVarNames[PROTOBUF_REPORT], s, GpsFix.time );

    #endif
}
