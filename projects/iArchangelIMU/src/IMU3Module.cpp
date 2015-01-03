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
/** @file IMU3Module.cpp

 @brief
 *** Add a description of your source file here ***

 @author Dave Billin
 */
//=============================================================================

#include <cstring>
#include <cmath>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "IMU3Module.h"


using namespace::std;
using namespace::YellowSubUtils;




#define _VERBOSE1(_expr_) if (m_Verbosity > 0) _expr_;
#define _VERBOSE2(_expr_) if (m_Verbosity > 1) _expr_;


//===========================
// FILE-SCOPE CONSTANTS::PolyphaseDecimator
//===========================
#define MESSAGE_ID_OFFSET 6		/* Byte offset of the message ID field in IMU3
								   serial messages */


// Serial data message ID constants
#define	MSGID_IMU	(uint8_t)0xaa	/* IMU Message ID */
#define	MSGID_AHRS	(uint8_t)0x55	/* AHRS Message ID */


// Byte offsets of message fields
#define STATUS_BYTE_OFFSET 	7	/* Byte offset of the status Byte fields */
#define DATA_RATE_OFFSET 	8	/* Byte offset of the DATA RATE field */
#define IMU_SENSOR_OFFSET	12	/* Byte offset of the floating-point gyro and
								   accelerometer sensor data fields */
#define AHRS_SENSOR_OFFSET	7	/* Byte offset of sensor data fields in an AHRS
								   message */

#define ROLL_ANGLE_INDEX	0	/* Index of Body roll field in IMU and AHRS*/
#define PITCH_ANGLE_INDEX	1	/* Index of Body pitch field in IMU and AHRS */
#define YAW_ANGLE_INDEX		2	/* Index of Body pitch field in IMU and AHRS */
#define BODY_VLONGITUDINAL	3	/* Index of longitudinal velocity in IMU */
#define BODY_VLATERAL		4	/* Index of latitudinal velocity in IMU */
#define BODY_VNORMAL		5	/* Index of normal velocity field in IMU */


#define ROLL_RATE_INDEX		3	/* Index of roll rate field in AHRS message */
#define PITCH_RATE_INDEX	4	/* Index of pitch rate field in AHRS message */
#define YAW_RATE_INDEX		5	/* Index of yaw rate field in AHRS message */

#define IM3_DECIMATION_FACTOR 10 /* Decimation factor applied to IMU and AHRS
									measurements */

/** @enum e_ImuMsgParserStates
 * @brief States used by the IMU message parser state machine
 */
enum e_ImuMsgParserStates
{
	SYNC = 0,	/**< Initial state.  Reads serial data until five consecutive
					 Bytes with value 0xff (the IM3 serial message delimiter
					 sequence) are received */

	READ_MESSAGE	/**< Reads a full 70-Byte message */
};


/** @enum e_LPFIndices
 * @brief
 * 	Indices of LPF channels used to decimate incoming measurements from the IMU
 */
enum e_LPFIndices
{
	LPF_GYRO_X = 0, 		/**< LPF for X-axis gyro */
	LPF_GYRO_Y,				/**< LPF for Y-axis gyro */
	LPF_GYRO_Z,				/**< LPF for Z-axis gyro */

	LPF_ACCELEROMETER_X,	/**< LPF for X-axis accelerometer */
	LPF_ACCELEROMETER_Y,	/**< LPF for Y-axis accelerometer */
	LPF_ACCELEROMETER_Z,	/**< LPF for Z-axis accelerometer */

	LPF_AHRS_ROLL_ANGLE,	/**< LPF for IMU AHRS roll angle */
	LPF_AHRS_PITCH_ANGLE,	/**< LPF for IMU AHRS pitch angle */
	LPF_AHRS_YAW_ANGLE,		/**< LPF for IMU AHRS yaw angle */

	LPF_AHRS_ROLL_RATE,		/**< LPF for IMU AHRS roll rate */
	LPF_AHRS_PITCH_RATE,	/**< LPF for IMU AHRS roll rate */
	LPF_AHRS_YAW_RATE,		/**< LPF for IMU AHRS roll rate */

	LPF_NUM_CHANNELS	/**< Number of LPF's - not a valid index */
};







//=============================================================================
IMU3Module::IMU3Module(int Verbosity)
 : m_Verbosity(Verbosity),
   m_NumMessageBytesReceived(0),
   m_RxIsSynchronized(false),
   m_ImuIsConnected(false),
   m_ImuDataIsFresh(false),
   m_AhrsDataIsFresh(false),
   m_ImuSelfTestFlags(0),
   m_ImuIsAligning(false),
   m_DeltaRoll_deg(0.0),
   m_DeltaPitch_deg(0.0),
   m_DeltaYaw_deg(0.0),
   m_dVLongitudinal(0.0),
   m_dVLateral(0.0),
   m_dVNormal(0.0),
   m_InertialRoll_deg(0.0),
   m_InertialPitch_deg(0.0),
   m_InertialYaw_deg(0.0),
   m_InertialRollRate_dps(0.0),
   m_InertialPitchRate_dps(0.0),
   m_InertialYawRate_dps(0.0)
{
	// Initialize IMU status Bytes
	m_ImuStatusBytes[0] = 0;
	m_ImuStatusBytes[1] = 0;
	m_ImuStatusBytes[2] = 0;

	//-----------------------------------------------------
	// Set up parameters for decimating low-pass filters
	//-----------------------------------------------------
    DSPFilterParameters<float> LPFParams;
    LPFParams.PreGain = 1.0f;
    LPFParams.PostGain = 1.0f;
    LPFParams.Numerator.clear();
    LPFParams.ProcessingIsEnabled = true;
    for (int i = 0; i < sm_NumLPFCoefficients; i++)
    {
        LPFParams.Numerator.push_back(sm_LPFCoefficients[i]);
    }


    //-----------------------------------------------------
    // Create decimating low-pass filters
    //-----------------------------------------------------
	for (int i = 0; i < LPF_NUM_CHANNELS; i++)
	{
		m_pLPF[i] = new PolyphaseFIRDecimator<float>(LPFParams,
		                                             IM3_DECIMATION_FACTOR);
	}
}



//=============================================================================
IMU3Module::~IMU3Module()
{
	// Delete all low-pass filter objects
	for (int i = 0; i < 12; i++)
	{
		if (m_pLPF[i] != NULL)
		{
			delete m_pLPF[i];
		}
	}
}



//=============================================================================
void IMU3Module::ProcessSerialData( char* pRxData, uint32_t NumRxBytes )
{
	// If five seconds has elapsed since the last IMU data was received, then
	// reset the parser and flag the IMU as disconnected.
    static double LastMessageRxTime = 0;
    double MsgTime = MOOSTime(false); // Get the current message time

	if ( (MsgTime - LastMessageRxTime) > 5.0 )
	{
		m_ImuIsConnected = false;
		m_RxIsSynchronized = false;
		m_NumMessageBytesReceived = 0;
	}

    // Validate parameters
    if ( (pRxData == NULL) || (NumRxBytes < 1) )
    {
        return;
    }

    char* pRxByte = pRxData;

    //------------------------------------------
    // Synchronize to message signature Bytes
    //------------------------------------------
    if (!m_RxIsSynchronized)
    {
        // Search for the first 0xff Byte in Rx data
        for (uint32_t i = 0; i < NumRxBytes; i++)
        {
            char c = *pRxByte;
            pRxByte++;
            m_MessageBuffer[m_NumMessageBytesReceived] = c;

            if ( static_cast<uint8_t>(c) == 0xff )
            {
                m_NumMessageBytesReceived++;
                if (m_NumMessageBytesReceived == 6)
                {
                    m_RxIsSynchronized = true;
                    break;
                }
            }
            else
            {
                m_NumMessageBytesReceived = 0;
            }

        }

        NumRxBytes -= pRxByte - pRxData;    // Calculate remaining Bytes
                                            // in buffer

        // If we haven't yet accumulated six signature Bytes, return to try
        // again with the next buffer of received data.
        if (m_NumMessageBytesReceived != 6)
        {
            return;
        }
    }


	// Run the parser state machine with received data
	while (NumRxBytes > 0)
	{
	    int NumBytesNeeded = IMU3_PACKET_SIZE - m_NumMessageBytesReceived;

		// Copy up to the needed number of RxBytes into the message buffer
	    uint32_t NumBytesThisTime =
	            ( NumRxBytes < (uint32_t)NumBytesNeeded ) ?
										        NumRxBytes : NumBytesNeeded;

	    memcpy( (m_MessageBuffer + m_NumMessageBytesReceived),
	            pRxByte, NumBytesThisTime);
		/*uint8_t* pWrite = m_MessageBuffer + m_NumMessageBytesReceived;
		int i = NumBytesThisTime;
		do
		{
			*pWrite++ = static_cast<uint8_t>(*pRxByte++);
		} while (--i > 0);
        */

		m_NumMessageBytesReceived += NumBytesThisTime;
        pRxByte += NumBytesThisTime;
		NumRxBytes -= NumBytesThisTime;

		if (m_NumMessageBytesReceived == IMU3_PACKET_SIZE)
		{

		    // Verify that the first six Bytes of a message are
		    // signature Bytes
		    for (int i = 0; i < 6; i++)
		    {
		        if (static_cast<uint8_t>(m_MessageBuffer[i]) != 0xff)
		        {
		            m_RxIsSynchronized = false;
		            break;
		        }
		    }

            //----------------------------------
            // Dispatch the received packet
            //----------------------------------
            uint8_t MessageID =
                    static_cast<uint8_t>(m_MessageBuffer[MESSAGE_ID_OFFSET]);

            if (MessageID == MSGID_IMU)
            {
                HandleIMUMessage(m_MessageBuffer);
            }
            else if (MessageID == MSGID_AHRS)
            {
                HandleAHRSMessage(m_MessageBuffer);
            }
            else
            {
                if (m_Verbosity >= 1)
                {
                    _VERBOSE1( MOOSTrace("\n[IMU3 parser] Invalid message "
                                         "ID: %d\n", MessageID) );
                }
            }

            LastMessageRxTime = MsgTime;

            // Flag the IMU as connected
            m_ImuIsConnected = true;

            // Reset counter to receive the next message
            m_NumMessageBytesReceived = 0;
		}

	}

}





//=============================================================================
bool IMU3Module::ImuDataIsFresh( void )
{
	bool b = m_ImuDataIsFresh;
	m_ImuDataIsFresh = false;
	return b;
}




//=============================================================================
int IMU3Module::GetSelfTestErrorFlags( void )
{
    int Flags = m_ImuSelfTestFlags;
    m_ImuSelfTestFlags = 0;
    return Flags;
}



//=============================================================================
bool IMU3Module::AhrsDataIsFresh( void )
{
	bool b = m_AhrsDataIsFresh;
	m_AhrsDataIsFresh = false;
	return b;
}





//=============================================================================
void IMU3Module::HandleIMUMessage(const uint8_t* pMessageBytes)
{
    const char* szSensorName[] = { "Delta Body Roll Angle",
                                   "Delta Body Pitch Angle",
                                   "Delta Body Yaw Angle",
                                   "Delta Body Longitudinal Velocity",
                                   "Delta Body Lateral Velocity",
                                   "Delta Body Normal Velocity"
                                 };

	const uint8_t* pStatus = pMessageBytes + STATUS_BYTE_OFFSET;

	//------------------------------------------
	// IMU Message Status Byte 0
	// Gyro/Accelerometer status bits
	//------------------------------------------
	int NewSelfTestFlags = ((pStatus[1] << 8) | pStatus[0]) & 0xfff;
	m_ImuSelfTestFlags |= NewSelfTestFlags;  // Latch self-test error flags


	//-----------------------------------
	// IMU Message Status Byte 1
	// Alignment status
	//-----------------------------------
	m_ImuIsAligning = (pStatus[1] & (1<<4)) ? true : false;
	m_ImuStatusBytes[1] = pStatus[1];


	//----------------------------------------------
	// DB: IMU Message Status Byte 3 is ignored
	//----------------------------------------------


	//----------------------------------------------
	// Apply accelerometer & gyro measurements to
	// anti-aliasing LPF's
	//----------------------------------------------
    const float* pSensorData =
            reinterpret_cast<const float*>(pMessageBytes + IMU_SENSOR_OFFSET);


    // NOTE:
    //  Do not allow NAN's in IMU data values to enter the filter!!
    //  If a NAN is found, process a sample value of zero
	float DecimatedSamples[6] = {0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 6; i++)
	{
	    if ( isnan(pSensorData[i]) == false )
	    {
	        DecimatedSamples[i] = m_pLPF[i]->ProcessSample(pSensorData[i]);
	    }
	    else
	    {
	        MOOSTrace("WARNING: IMU reported %s as NAN!!\n", szSensorName[i]);
	        DecimatedSamples[i] = 0.0f;
	    }
	}


	// Report new decimated measurements
	if ( m_pLPF[LPF_DeltaBodyRollAngle]->OutputIsFresh() )
	{
		m_DeltaRoll_deg = DecimatedSamples[LPF_DeltaBodyRollAngle];
		m_DeltaPitch_deg = DecimatedSamples[LPF_DeltaBodyPitchAngle];
		m_DeltaYaw_deg = DecimatedSamples[LPF_DeltaBodyYawAngle];
		m_dVLongitudinal= DecimatedSamples[LPF_DeltaBodyLongitudinalVelocity];
		m_dVLateral = DecimatedSamples[LPF_DeltaBodyLateralVelocity];
		m_dVNormal = DecimatedSamples[LPF_DeltaBodyNormalVelocity];

		m_ImuDataIsFresh |= true;
	}

}




//=============================================================================
void IMU3Module::HandleAHRSMessage(const uint8_t* pMessageBytes)
{
	const float* pSensorData =
	        reinterpret_cast<const float*>(pMessageBytes + AHRS_SENSOR_OFFSET);

	//-------------------------------------------------
	// Apply inertial measurements to decimating LPF's
	//-------------------------------------------------
	float DecimatedSamples[6];
	for (int i = 0; i < 6; i++)
	{
	    DecimatedSamples[i] =
	          m_pLPF[i + LPF_InertialRollAngle]->ProcessSample(pSensorData[i]);
	}

	// If decimation yielded new samples, report them.
	if ( m_pLPF[LPF_InertialRollAngle]->OutputIsFresh() )
	{
		m_InertialRoll_deg = DecimatedSamples[0];
		m_InertialPitch_deg = DecimatedSamples[1];
		m_InertialYaw_deg = DecimatedSamples[2];

		m_InertialRollRate_dps = DecimatedSamples[3];
		m_InertialPitchRate_dps = DecimatedSamples[4];
		m_InertialYawRate_dps = DecimatedSamples[5];

		m_AhrsDataIsFresh |= true;
	}

	//--------------------------------------------
	// Translate and register the new sensor values
	//--------------------------------------------

}


const int IMU3Module::sm_NumLPFCoefficients = 154;

const float IMU3Module::sm_LPFCoefficients[] = {
    -0.00072639466758498718, -0.00057129884609960588, -0.00076065469005151889,
    -0.00095852338219948536, -0.0011532728760419646,  -0.0013308146727472317,
    -0.0014757652662652795,  -0.001570184100135631,   -0.001600280208655903,
    -0.001551257907456605,   -0.0014125500755020437,  -0.0011799570212111883,
    -0.00085195370262711124, -0.00043623084891244508,  5.4452440598246716e-005,
     0.00059938858254971427,  0.0011722661186659644,   0.0017404156141629756,
     0.0022691584258445118,   0.0027207131752677795,   0.0030596257474830536,
     0.0032532436136157765,   0.0032754286484462762,   0.0031086661066087812,
     0.0027465515439528917,   0.0021949083523114584,   0.0014733705049077075,
     0.00061486558150202083, -0.0003353242187118413,  -0.0013210295848735088,
    -0.0022787857944730526,  -0.0031407835123891895,  -0.0038395571747709698,
    -0.0043124227438043738,  -0.0045064707396573591,  -0.0043827566837598914,
    -0.0039208594543415441,  -0.0031213936846968267,  -0.002008321731916209,
    -0.00062927414871503105,  0.00094558908190513604,  0.0026261595856572717,
     0.0043066852983388009,   0.0058714804886586596,   0.0072017509153179896,
     0.0081831476737058793,   0.0087140650498339217,   0.0087131533679414546,
     0.0081269282146743706,   0.00693554819910244,     0.0051574327959368433,
     0.0028515055486491433,   0.00011749260880081344, -0.002906729450921572,
    -0.0060497104704044825,  -0.0091138832392109355,  -0.011884708733872582,
    -0.014141607255216841,   -0.015669913972915478,   -0.016273266562607257,
    -0.015785640733078761,   -0.014082478156880521,   -0.01109008229903663,
    -0.0067926154419346157,  -0.0012364852738471399,   0.0054687145453040792,
     0.013152379648198556,    0.021588584835367852,    0.030504767871030555,
     0.039593309047049823,    0.048525071854562452,    0.056964694167965083,
     0.064586332430655771,    0.07108943167520744,     0.07621338759601462,
     0.079750457488700799,    0.081556058469147624,    0.081556058469147624,
     0.079750457488700799,    0.07621338759601462,     0.07108943167520744,
     0.064586332430655771,    0.056964694167965083,    0.048525071854562452,
     0.039593309047049823,    0.030504767871030555,    0.021588584835367852,
     0.013152379648198556,    0.0054687145453040792,  -0.0012364852738471399,
    -0.0067926154419346157,  -0.01109008229903663,    -0.014082478156880521,
    -0.015785640733078761,   -0.016273266562607257,   -0.015669913972915478,
    -0.014141607255216841,   -0.011884708733872582,   -0.0091138832392109355,
    -0.0060497104704044825,  -0.002906729450921572,    0.00011749260880081344,
     0.0028515055486491433,   0.0051574327959368433,   0.00693554819910244,
     0.0081269282146743706,   0.0087131533679414546,   0.0087140650498339217,
     0.0081831476737058793,   0.0072017509153179896,   0.0058714804886586596,
     0.0043066852983388009,   0.0026261595856572717,   0.00094558908190513604,
    -0.00062927414871503105, -0.002008321731916209,   -0.0031213936846968267,
    -0.0039208594543415441,  -0.0043827566837598914,  -0.0045064707396573591,
    -0.0043124227438043738,  -0.0038395571747709698,  -0.0031407835123891895,
    -0.0022787857944730526,  -0.0013210295848735088,  -0.0003353242187118413,
     0.00061486558150202083,  0.0014733705049077075,   0.0021949083523114584,
     0.0027465515439528917,   0.0031086661066087812,   0.0032754286484462762,
     0.0032532436136157765,   0.0030596257474830536,   0.0027207131752677795,
     0.0022691584258445118,   0.0017404156141629756,   0.0011722661186659644,
     0.00059938858254971427,  5.4452440598246716e-5,-  0.00043623084891244508,
    -0.00085195370262711124, -0.0011799570212111883,  -0.0014125500755020437,
    -0.001551257907456605,   -0.001600280208655903,   -0.001570184100135631,
    -0.0014757652662652795,  -0.0013308146727472317,  -0.0011532728760419646,
    -0.00095852338219948536, -0.00076065469005151889, -0.00057129884609960588,
    -0.00072639466758498718
};
