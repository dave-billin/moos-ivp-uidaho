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
/** @file iArchangelIMU.cpp

@brief
	Implementation of the iArchangelIMU application object

@author Dave Billin

*/
//=============================================================================

#include "iArchangelIMU.h"
#include "config.h"

#include <YellowSubUtils.h>
#include <MOOS/libMOOS/Utils/MOOSFileReader.h>
#include <MOOS/libMOOS/Utils/MOOSAssert.h>

#include <iostream>
#include <fstream>


using namespace std;
using YellowSubUtils::LinuxSerialPortEx;
using YellowSubUtils::DebugFileSerialPort;



// Default variable names IMU values will be published to
const char* iArchangelIMU::sm_DefaultPublishedVarNames[NUM_IMU3_SENSOR_VALUES]=
			{
				"IMU_DELTA_ROLL_ANGLE",	    /* IMUSENSOR_DeltaAngle_Roll */
				"IMU_DELTA_PITCH_ANGLE",	/* IMUSENSOR_DeltaAngle_Pitch */
				"IMU_DELTA_YAW_ANGLE",	    /* IMUSENSOR_DeltaAngle_Yaw */
				"IMU_DELTAV_LONGITUDINAL",	/* IMUSENSOR_DeltaVLongitudinal */
				"IMU_DELTAV_LATERAL",       /* IMUSENSOR_DeltaVLateral */
				"IMU_DELTAV_NORMAL",		/* IMUSENSOR_DeltaVNormal */
				"IMU_INERTIAL_ROLL",		/* IMUSENSOR_AHRS_Angle_Roll */
				"IMU_INERTIAL_PITCH",		/* IMUSENSOR_AHRS_Angle_Pitch */
				"IMU_INERTIAL_YAW",			/* IMUSENSOR_AHRS_Angle_Yaw */
				"IMU_INERTIAL_ROLLRATE",	/* IMUSENSOR_AHRS_Rate_Roll */
				"IMU_INERTIAL_PITCHRATE",	/* IMUSENSOR_AHRS_Rate_Pitch */
				"IMU_INERTIAL_YAWRATE"		/* IMUSENSOR_AHRS_Rate_Yaw */
			};



enum e_StatusStringIDs  // Used to index strings in szImuStatusStringTable
{
    GyroX1Fail = 0,
    GyroY1Fail,
    GyroZ1Fail,
    AccX1Fail,
    AccY1Fail,
    AccZ1Fail,
    AccX2Fail,
    AccY2Fail,
    AccZ2Fail,
    TempXFail,
    TempYFail,
    TempZFail,
    Aligning
};

const char* szImuStatusStringTable[] = { "GyroX",
                                         "GyroY",
                                         "GyroZ",
                                         "AccX1",
                                         "AccY1",
                                         "AccZ1",
                                         "AccX2",
                                         "AccY2",
                                         "AccZ2",
                                         "TempX",
                                         "TempY",
                                         "TempZ",
                                         "Aligning"
                                          };




//=============================================================================
/** Creates an instance of the object */
iArchangelIMU::iArchangelIMU( void )
:   m_SerialPort(NULL),
    m_UseSimDataFile(false),
    m_Imu(0),
    m_Verbosity(0),
    m_MOOSDBIsConnected(false),
    m_DeltaRoll_Polarity(1.0),
    m_DeltaPitch_Polarity(1.0),
    m_DeltaYaw_Polarity(1.0),
    m_DeltaVLongitudinal_Polarity(1.0),
    m_DeltaVLateral_Polarity(1.0),
    m_DeltaVNormal_Polarity(1.0),
    m_InertialPitchPolarity(1.0),
    m_InertialRollPolarity(1.0),
    m_InertialYawPolarity(1.0)
{
	// Load default published variable names and zero last sensor values
	for (int i = 0; i < NUM_IMU3_SENSOR_VALUES; i++)
	{
		m_PublishedVarNames[i] = sm_DefaultPublishedVarNames[i];
	}
}



//=============================================================================
/** Called when the object goes out of scope */
iArchangelIMU::~iArchangelIMU()
{
    if (m_SerialPort != NULL)
    {
        delete m_SerialPort;
    }
}




//=============================================================================
bool iArchangelIMU::Iterate( void )
{
    static double tLast = 0;
	static bool ImuConnectedLastTime = false;
	char RxBuffer[1024];

	// Read serial data from the serial port (or data file)
	double d;
	int NumBytesRead = m_SerialPort->ReadNWithTimeOut(RxBuffer, /* Buffer */
	                                                  140,  /* Buffer Size */
	                                                  0.05, /* Timeout (sec)*/
	                                                  &d);  /* Timestamp */

	m_Imu.ProcessSerialData(RxBuffer, NumBytesRead);

    bool ImuConnectedThisTime = m_Imu.IsConnected();
    double t = HPMOOSTime();


	//-----------------------------------------
	// Publish changes to IMU connection state
	//-----------------------------------------
	if (ImuConnectedThisTime != ImuConnectedLastTime)
	{
		string s = (ImuConnectedThisTime) ? "TRUE" : "FALSE";
		m_Comms.Notify("IMU_CONNECTED", s, t);

		if (m_Verbosity >= 1)
		{
			MOOSTrace("\n\n<<< The IM3 module is %s >>>\n\n",
			          ((ImuConnectedThisTime) ? "connected" : "disconnected") );
		}
	}
	ImuConnectedLastTime = ImuConnectedThisTime;


	//-----------------------------------------
	// Publish IMU measurements if they have
	// changed since the last call to Iterate()
	//-----------------------------------------
	if (ImuConnectedThisTime)
	{
	    bool IMUDataIsFresh = m_Imu.ImuDataIsFresh();
	    bool AHRSDataIsFresh = m_Imu.AhrsDataIsFresh();

		if ( IMUDataIsFresh )
		{
			double DeltaRollAngle =  m_DeltaRoll_Polarity
			                             * m_Imu.DeltaRollAngle();
			double DeltaPitchAngle = m_DeltaPitch_Polarity
			                             * m_Imu.DeltaPitchAngle();
			double DeltaYawAngle =   m_DeltaYaw_Polarity
			                             * m_Imu.DeltaYawAngle();

			double dVLongitudinal = m_DeltaVLongitudinal_Polarity
			                            * m_Imu.DeltaVLongitudinal();
			double dvLateral = m_DeltaVLateral_Polarity
			                            * m_Imu.DeltaVLateral();
			double dvNormal = m_DeltaVNormal_Polarity
			                            * m_Imu.DeltaVNormal();

			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaAngle_Roll],
							DeltaRollAngle, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaAngle_Pitch],
							DeltaPitchAngle, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaAngle_Yaw],
							DeltaYawAngle, t );

			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaVLongitudinal],
			                dVLongitudinal, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaVLateral],
			                dvLateral, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_DeltaVNormal],
			                dvNormal, t );
		}


	    //-----------------------------------------
	    // Publish AHRS measurements if they have
	    // changed since the last call to Iterate()
	    //-----------------------------------------
		if ( AHRSDataIsFresh )
		{
			double Roll =  m_InertialRollPolarity * m_Imu.InertialRoll();
			double Pitch = m_InertialPitchPolarity * m_Imu.InertialPitch();
			double Yaw =   m_InertialYawPolarity * m_Imu.InertialYaw();
			double RollRate= m_InertialRollPolarity * m_Imu.InertialRollRate();
			double PitchRate = m_InertialPitchPolarity
			                        * m_Imu.InertialPitchRate();
			double YawRate = m_InertialYawPolarity * m_Imu.InertialYawRate();

			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_Roll],
							Roll, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_Pitch],
							Pitch, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_Yaw],
							Yaw, t );

			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_RollRate],
							RollRate, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_PitchRate],
							PitchRate, t );
			m_Comms.Notify( m_PublishedVarNames[IMUSENSOR_AHRS_YawRate],
							YawRate, t );
		}

		//--------------------------------------------
        // Publish any IMU status flags that are set
		// at a rate of 2 Hz
		//--------------------------------------------
		double dt = t - tLast;
		if (dt > 0.5)
		{
	        tLast = t;

		    string sFlags;
            int Flags = m_Imu.GetSelfTestErrorFlags();
            if (Flags != 0)
            {
                for (int i = 0; i < 12; i++)
                {
                    if ((Flags & 1) == 0)
                    {
                        AppendToCSVList(szImuStatusStringTable[i], sFlags);
                    }
                }
            }

            if (m_Imu.IsAligning())
            {
                AppendToCSVList(szImuStatusStringTable[Aligning], sFlags);
            }

            m_Comms.Notify("IMU_FLAGS", sFlags, t);
		}
	}

	return true;
}






//=============================================================================
bool iArchangelIMU::OnNewMail(MOOSMSG_LIST & NewMail)
{
	// DB: This app never subscribes to anything, so we don't parse mail.

	return true;
}









//=============================================================================
bool iArchangelIMU::OnStartUp( void )
{
	string sBar50, s;
	string sAppName = GetAppName();
	string sEmpty = "";

	// Print a cute banner
	sBar50 = string(50, '=') + "\n";
	MOOSTrace(s +
			  MOOSFormat("iArchangelIMU version %s\n", APP_VERSION_TUPLE) +
			  "Written by Dave Billin\n" +
			  sBar50 + "\n\n");


	// Load mission file parameters.  These settings may or may not overwrite
	// the default variable names we just assigned.
	if ( !LoadMissionFileParameters() )
	{
		return false;
	}

	//-------------------------------------------------
	// Display variables sensors will be published to
	//-------------------------------------------------
	const char* szSensor[] = { "Delta Body Roll Angle",
							   "Delta Body Pitch Angle",
							   "Delta Body Yaw Angle",
							   "Longitudinal Velocity", "Lateral Velocity",
							   "Normal Velocity", "Inertial Roll Angle",
							   "Inertial Pitch Angle", "Inertial Yaw Angle",
							   "Inertial Roll Rate", "Inertial Pitch Rate",
							   "Inertial Yaw Rate" };

	MOOSTrace(sBar50 +
	          "\nPublishing IMU3 measurements to MOOS Variables:\n" +
	          sBar50 + "\n\n");
	for (int i = 0; i < NUM_IMU3_SENSOR_VALUES; i++)
	{
		MOOSTrace( "%24s  -->  %s\n",
				   szSensor[i], m_PublishedVarNames[i].c_str() );
	}
	MOOSTrace("\n\n");

	SetAppFreq(100);
	SetCommsFreq(20);

	//-------------------------------------
    // Wait until the MOOSDB is connected
	// before enabling Iterate()
	//-------------------------------------
    while (!m_MOOSDBIsConnected)
    {
        MOOSPause(200);
    }

    return true;
}



//=============================================================================
bool iArchangelIMU::OnConnectToServer( void )
{
    m_MOOSDBIsConnected = true;
	return true;
}



//=============================================================================
bool iArchangelIMU::OnDisconnectFromServer( void )
{
	return true;
}



//=============================================================================
bool iArchangelIMU::LoadMissionFileParameters( void )
{
	string s, sVal;

	//------------------------------------------
	// Load optional mission file parameters
	//------------------------------------------
	// Load variable re-mapping
	for (int i = 0; i < NUM_IMU3_SENSOR_VALUES; i++)
	{
		s = sm_DefaultPublishedVarNames[i] + string("_PUBLISHTO");
		m_MissionReader.GetConfigurationParam(s, m_PublishedVarNames[i]);
	}

	// Load verbosity level of debugging messages
	m_MissionReader.GetConfigurationParam("VERBOSITY", m_Verbosity);

	// Load option to use simulated IMU data
    m_MissionReader.GetConfigurationParam( "IMUDATAFILE", m_UseSimDataFile);
    if ( m_UseSimDataFile )
    {
        if ( m_MissionReader.GetConfigurationParam("PORT", s) )
        {
            MOOSTrace("Simulating IMU with data from:\n%s\n\n", s.c_str() );
        }
    }

    // Load optional measurement inversions
    bool InvertEnabled;
    if (m_MissionReader.GetConfigurationParam("INVERT_DELTA_ROLL_ANGLE",
                                              InvertEnabled) )
    {
        m_DeltaRoll_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_DELTA_PITCH_ANGLE",
                                              InvertEnabled) )
    {
        m_DeltaPitch_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_DELTA_YAW_ANGLE",
                                              InvertEnabled) )
    {
        m_DeltaYaw_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_DELTAV_LONGITUDINAL",
                                              InvertEnabled) )
    {
        m_DeltaVLongitudinal_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_DELTAV_LATERAL",
                                              InvertEnabled) )
    {
        m_DeltaVLateral_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_DELTAV_NORMAL",
                                              InvertEnabled) )
    {
        m_DeltaVNormal_Polarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_INERTIAL_ROLL",
                                              InvertEnabled) )
    {
        m_InertialRollPolarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_INERTIAL_PITCH",
                                              InvertEnabled) )
    {
        m_InertialPitchPolarity = (InvertEnabled) ? -1.0f : 1.0f;
    }

    if (m_MissionReader.GetConfigurationParam("INVERT_INERTIAL_YAW",
                                              InvertEnabled) )
    {
        m_InertialYawPolarity = (InvertEnabled) ? -1.0f : 1.0f;
    }


    //----------------------------------------
    // Load required mission file parameters
    // and configure the serial port
    //----------------------------------------
    if ( !ConfigureSerialPort() )
    {
        return false;
    }

	return true;
}





//=============================================================================
bool iArchangelIMU::ConfigureSerialPort( void )
{
    STRING_LIST sParams;

    //-------------------------------------------------
    // Create and configure a serial port for the IMU
    //-------------------------------------------------
    string sPort;
    if ( m_MissionReader.GetConfigurationParam("PORT", sPort) )
    {
        MOOSTrace("<< Connecting to IMU module using " + sPort + " >>\n\n");
    }
    else
    {
        MOOSTrace("Missing required mission file parameter: PORT\n");
        return false;
    }

    sParams.push_back("PORT=" + sPort);
    sParams.push_back("VERBOSE=FALSE");
    sParams.push_back("BAUDRATE=460800");
    sParams.push_back("STREAMING=FALSE");

    // Create the serial port object
    if (m_UseSimDataFile)
    {
        m_SerialPort = new DebugFileSerialPort();   // Simulated data from file
    }
    else
    {
        m_SerialPort = new LinuxSerialPortEx(); // Actual serial port comms
    }

    // Configure the serial port
    if(!m_SerialPort->Configure(sParams))
    {
        MOOSTrace("Failed to create serial port!\n");
        return false;
    }


    return true;
}




string& iArchangelIMU::AppendToCSVList( const char* szStringToAppend,
                                        string& sTarget )
{
    if ( !sTarget.empty() )
    {
        sTarget.push_back(',');
    }
    sTarget += szStringToAppend;
    return sTarget;
}


