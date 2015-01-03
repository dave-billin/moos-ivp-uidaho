//=============================================================================
/** @file EKFTest.cpp
 *
 * @brief
 * 	A module used for testing the EKF class against EKF source code from the
 *  Rabbit-based AUV
 *
 * @details
 *	This application performs the following tasks:
 *		-# Loads EKF-related packets from a KIRK (binary) log file into memory.
 *		-# Initializes Kalman.lib code and an EKF object using LBL beacon
 *		   locations and data from the KIRK log.
 *		-# Applies data from all EKF-related packets to the Kalman.lib and EKF
 *		   code using a time delta from packet time stamps.
 *		-# Checks for divergence between Kalman.lib and EKF matrices after each
 *		   operation.
 *
 * @author Dave Billin
 */
//=============================================================================

#include <string>
#include <set>
#include <vector>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "LibYellowSubNav.h"	// VehicleEKF class under test
#include "Kalman_lib.h"	// Dynamic C library functions to test against
#include "UAV_PACKET_TYPES.h"	// UAVNet packet type definitions
#include "UAVNetPacket.h"
#include "RunningStatistics.h"	// Statistics object

using namespace std;
using namespace UAVnet;
using namespace NEWMAT;
using namespace YellowSubNav;

//=======================
// LOCAL CONSTANTS
//=======================
//#define VERBOSE

#ifdef VERBOSE
	#define _VERBOSE( _expr_ ) _expr_
#else
	#define _VERBOSE( _expr_ )
#endif


//constants for speed control
//these 2 constants are for the linearized RPM to velocity conversion
#define MAGNETOMETER_PRESENT
#ifdef MAGNETOMETER_PRESENT // With Magnetometer	Updated Aug. 12 2009
	#define VELPERRPM 0.001138//slope of the linear conversion
	#define VELOFFSET -0.2533//y intercept of the linear conversion
#else // Without Magnetometer
	#define VELPERRPM 0.001167//slope of the linear conversion
	#define VELOFFSET -0.1988//y intercept of the linear conversion
#endif



#define MAX_EKF_ERROR	1.0




//=======================
// DATA TYPES
//=======================
/** @struct PacketComparer
 * @brief Provides timestamp comparison function for STL set class
 */
struct TimeStampComparison
{
  bool operator() (const UAVNetPacket& lhs, const UAVNetPacket& rhs) const
  { return lhs > rhs; }
};

typedef struct
{
   float N;
   float E;
} R2Coordinate_t;





//=======================
// FUNCTION PROTOTYPES
//=======================
void PrintUsage( void );
string PacketTypeToString( uint16_t PacketTypeId );
string MsTimeStampToString( uint32_t TimeStampMs );
void SplitIMUPacket( const UAVNetPacket& SourcePacket,
					 UAVNetPacket OUT_SingleMsgPackets[] );

void DispatchLogPacket( const UAVNetPacket& ThePacket );
void HandleSensorPacket( const UAVNetPacket& ThePacket );
void HandleControlsPacket( const UAVNetPacket& ThePacket );
void HandleNavPingPacket( const UAVNetPacket& ThePacket );
void HandleWhoiMsgPacket( const UAVNetPacket& ThePacket );
void HandleBeaconLocationPacket( const UAVNetPacket& ThePacket );
void HandleSingleImuPacket( const UAVNetPacket& ThePacket );
void HandleSynchRangePacket( const UAVNetPacket& ThePacket );
void HandleShipEkfPacket( const UAVNetPacket& ThePacket );
void HandleDepthPacket( const UAVNetPacket& ThePacket );

bool TestEkfError( double MaxPercentError );
//void PrintFloatMatrix( float* pData[], int NumRows, int NumColumns );
void PrintNewmatMatrix( const Matrix& TheMatrix );
void PrintEkfMatrices( void );
//void CompareMatrices( const float* pFloatData[], const GeneralMatrix& Matrix );

//=======================
// GLOBAL VARIABLES
//=======================
// STL multiset containing packets from the log ordered by time stamp
multiset<UAVNetPacket, TimeStampComparison> LogPackets;			// packets from log

UAVNetPacket GLastSensorPacket;		// Stores the last sensor packet 'received'

float GRpmVelocityEstimate = 0;		// Estimated velocity based on motor RPM (m/s)

R2Coordinate_t GEstimatedCoord;

double GLastPropagateTime = 0;	/* Simulation time when the kalman.lib EKF was
								   last propagated */

double GSimulationTimeSec = 0;	/* Time (in seconds) derived from packet time
								   stamps and used for 'playback' of log
								   packets */

CMOOSGeodesy GReferenceGeodesy;	// Reference Geodesy object used by EKF class

VehicleEKF GNavEKF(GReferenceGeodesy);	// EKF object under test

RunningStatistics<Real> GXStats[VehicleEKF::NUM_STATES];


//=============================================================================
/** Application entry point */
int main( int argc, const char* argv[] )
{
	string sInputFileName;
	UAVNetPacket LogPacket;
	uint32_t PacketCounter, TotalPackets;
	UAVNetPacket SingleMsgImuPackets[4];
	uint16_t TypeIds[]=
			{ UAVnet::TYPE_SENSOR_LOG, UAVnet::TYPE_LBL_POS_LOG,
			  UAVnet::TYPE_WHM_MSG_LOG, UAVnet::TYPE_BUOY_POS_LOG,
			  UAVnet::TYPE_IMU_LOG, UAVnet::TYPE_AHRS_LOG,
			  UAVnet::TYPE_SYNCH_RANGE_LOG, UAVnet::TYPE_KALMSHIP_LOG,
			  UAVnet::TYPE_DEPTH_LOG, UAVnet::TYPE_CONTROLS_LOG };
	set<uint16_t> PacketTypesOfInterest(TypeIds,
										TypeIds + sizeof(TypeIds)/sizeof(uint16_t) );

	// *** DEBUG ***
	/*
	MsTimeStampToString(86397123UL);
	MOOSTrace("Packet Header Size: %d\n", sizeof(PacketHeader_t));
	MOOSTrace("GenericPacket_t Bytes: %d\n", sizeof(GenericPacket_t));
	MOOSTrace("SensorPacket_t Bytes: %d\n", sizeof(SensorPacket_t));
	MOOSTrace("ControlsPacket_t Bytes: %d\n", sizeof(ControlsPacket_t));
	MOOSTrace("CommandPacket_t Bytes: %d\n", sizeof(CommandPacket_t));
	MOOSTrace("LblEstPositionPacket_t Bytes: %d\n", sizeof(LblEstPositionPacket_t));
	MOOSTrace("WhoiMsgPacket_t Bytes: %d\n", sizeof(WhoiMsgPacket_t));
	MOOSTrace("GpsGPGGAPacket_t Bytes: %d\n", sizeof(GpsGPGGAPacket_t));
	MOOSTrace("LblBeaconLocationPacket_t Bytes: %d\n", sizeof(LblBeaconLocationPacket_t));
	MOOSTrace("VehicleEKFPacket_t Bytes: %d\n", sizeof(VehicleEKFPacket_t));
	MOOSTrace("ShipEKFPacket_t Bytes: %d\n", sizeof(ShipEKFPacket_t));
	MOOSTrace("DepthPacket_t Bytes: %d\n", sizeof(DepthPacket_t));
	MOOSTrace("ImuPacket_t Bytes: %d\n", sizeof(ImuPacket_t));
	MOOSTrace("DspTelemetryPacket_t Bytes: %d\n", sizeof(DspTelemetryPacket_t));
	MOOSTrace("SynchronousRangePacket_t Bytes: %d\n", sizeof(SynchronousRangePacket_t));
	*/
	// *** END DEBUG ***


	// Handle command line parameters
	if (argc < 2)
	{
		PrintUsage();
		exit(0);
	}
	else
	{
		sInputFileName = argv[1];
	}

	//--------------------------------------
	// Open the specified file for input
	//--------------------------------------
	ifstream Fin(sInputFileName.c_str(), ifstream::binary);
	if (!Fin.good())
	{
		MOOSTrace("Failed to open input file '%s'\n\n", sInputFileName.c_str());
	}
	else
	{
		MOOSTrace("Opened input file '%s'\n\n", sInputFileName.c_str());
	}

	//--------------------------------------
	// Extract packets of interest from the
	// log file
	//--------------------------------------
	TotalPackets = 0;
	while( Fin.good() )
	{
		try { Fin >> LogPacket; }	// Read one packet
		catch (UAVNetPacket::UAVNetPacketException& e)
		{
			break;	// On failure to read a packet, stop reading
		}

		TotalPackets++;

		// Print the packet type if debugging
		/*#ifdef VERBOSE
		MOOSTrace( MOOSFormat("(%d)  ", TotalPackets) +
				   MsTimeStampToString(LogPacket.Header()->MsTimeStamp) + " " +
				   PacketTypeToString(LogPacket.Header()->Type) + "\n" );
		#endif
		*/

		// Handle IMU packets separately.  They need to be split into packets
		// containing individual IMU messages
		if (LogPacket.Header()->Type == TYPE_IMU_LOG)
		{
			SplitIMUPacket(LogPacket, SingleMsgImuPackets);
			for (int i = 0; i < 4; i++)
			{
				LogPackets.insert(SingleMsgImuPackets[i]);
			}
		}
		else
		{
			// Add all other EKF-related packets to the LogPacket multiset
			// for later 'playback'
			if (PacketTypesOfInterest.find(LogPacket.Header()->Type) !=
				PacketTypesOfInterest.end())
			{
				LogPackets.insert(LogPacket);
			}
		}

	}
	Fin.close();	// Close the input file

	// Print the packets we gathered for debugging
	/*#ifdef VERBOSE
	PacketCounter = 1;
	for (multiset<UAVNetPacket>::iterator iter = LogPackets.begin();
		 iter != LogPackets.end(); iter++)
	{
		MOOSTrace( MOOSFormat("(%d)  ", PacketCounter) +
				   MsTimeStampToString(iter->Header()->MsTimeStamp) + " " +
				   PacketTypeToString(iter->Header()->Type) + "\n" );

		PacketCounter++;
	}
	#endif
	*/

	MOOSTrace("Gathered %u EKF-related packets from log data.\n\n",
			  LogPackets.size() );

	whoi.ping_interval = 15;	/* Using default ping interval from line 350 of
								   mission.lib */

	MOOSPause(1000);

	//--------------------------------------
	// Play back log packets into the EKF and
	// Kalman.lib code
	//--------------------------------------

	// Reset statistics on state error
	for (int i = 0; i < VehicleEKF::NUM_STATES; i++)
	{
		GXStats[i].Initialize();
	}

	PacketCounter = 0;
	set<UAVNetPacket>::iterator iter;
	for (iter = LogPackets.begin(); iter != LogPackets.end(); iter++)
	{
		// Convert packet time stamp to seconds and use this to set
		// simulation time
		GSimulationTimeSec = iter->Header()->MsTimeStamp * 0.001;

		// Apply the packet
		DispatchLogPacket(*iter);

		PacketCounter++;

		// Calculate EKF differences and notify on divergence
		if ( !TestEkfError(MAX_EKF_ERROR) )
		{
			PrintEkfMatrices();
			return -2;
		}
	}

	MOOSTrace("\n\n*** Playback: %u of %u packets complete ***\n\n",
			  PacketCounter, LogPackets.size());

	MOOSTrace("EKF state error statistics:\n\n");

	for (int i = 0; i < 5; i++)
	{
		const char* szStateNames[5] = { "East Coordinate", "North Coordinate",
										"Speed", "Yaw", "Yaw Bias" };

		MOOSTrace("State(%d): %s:\n"
				  "  Peak(+) Error:  %10.5f %%\n"
				  "  Peak(-) Error:  %10.5f %%\n"
				  "  Mean Error: %10.5f %%\n"
				  "  Error Variance:  %10.5f\n"
				  "  Error Standard Deviation: %10.5f\n\n",
				  i, szStateNames[i], GXStats[i].MostPositive(),
				  GXStats[i].MostNegative(), GXStats[i].Mean(),
				  GXStats[i].Variance(), GXStats[i].StdDev());
	}
	exit(0);
}




//=============================================================================
/** Prints usage info for the application */
void PrintUsage( void )
{
	MOOSTrace("\nEKFTest\n"
			  "USAGE: EKFTest [INPUTFILE]\n\n"
			  );
}



//=============================================================================
/** Reads a packet from the input file and calls the appropriate handler
 *  function to pass it to the EKF object and Rabbit library code. */
void DispatchLogPacket( const UAVNetPacket& ThePacket )
{
	_VERBOSE( MOOSTrace( "\n" + MsTimeStampToString(GSimulationTimeSec * 1000.0) +
						 " Dispatching " +
						 PacketTypeToString(ThePacket.Header()->Type) + " ") );


	//--------------------------------------------
	// Handle packets we care about
	switch (ThePacket.Header()->Type)
	{
	case UAVnet::TYPE_SENSOR_LOG:
		HandleSensorPacket( ThePacket );
		break;

	case UAVnet::TYPE_CONTROLS_LOG:
		HandleControlsPacket( ThePacket );
		break;

	case UAVnet::TYPE_LBL_POS_LOG:
		HandleNavPingPacket( ThePacket );
		break;

	case UAVnet::TYPE_WHM_MSG_LOG:
		HandleWhoiMsgPacket( ThePacket );
		break;

	case UAVnet::TYPE_BUOY_POS_LOG:
		HandleBeaconLocationPacket( ThePacket );
		break;

	case UAVnet::TYPE_EKFTEST_IMU_MSG:
		HandleSingleImuPacket( ThePacket );
		break;

	case UAVnet::TYPE_AHRS_LOG:
		MOOSTrace("Found an AHRS packet in the log and ignored it...\n");
		break;

	case UAVnet::TYPE_SYNCH_RANGE_LOG:
		HandleSynchRangePacket( ThePacket );
		break;

	case UAVnet::TYPE_KALMSHIP_LOG:
		HandleShipEkfPacket( ThePacket );
		break;

	case UAVnet::TYPE_DEPTH_LOG:
		HandleDepthPacket( ThePacket );
		break;

	default:
		MOOSTrace("\n");
		break;
	}
}




//=============================================================================
/** Applies a sensor packet read from the log file */
void HandleSensorPacket( const UAVNetPacket& ThePacket )
{
	SensorPacket_t* pSensorPacket = const_cast<SensorPacket_t*>(ThePacket.AsSensorPacket());
	float DeltaT_sec = GSimulationTimeSec - GLastPropagateTime;
	float ActiveVelocity = ((pSensorPacket->GPS_Velocity > 0) && (pSensorPacket->GPS_Velocity < 1.5) ) ?
							  pSensorPacket->GPS_Velocity : GRpmVelocityEstimate;

	//------------------------------
	// Apply to Kalman.lib EKF
	//------------------------------
	// Propagate the EKF
	PropagateKalman(DeltaT_sec);
	GLastPropagateTime = GSimulationTimeSec;

	_VERBOSE( MOOSTrace("  Velocity: GPS=%3.2f  RPM=%3.2f   Heading: %3.2f deg\n",
						pSensorPacket->GPS_Velocity, GRpmVelocityEstimate,
						pSensorPacket->CompassHeading) );

	_VERBOSE( MOOSTrace("*** PROPAGATING ***\n") );

	// Update sensor info
	KalmanUpdate( MEASURE_HEADING,
				  ActiveVelocity,
				  pSensorPacket->CompassHeading,
				  0, 0, 0, 0,
				  pSensorPacket->Depth_cm );


	//------------------------------
	// Apply to VehicleEKF object
	//------------------------------
	// Propagate the EKF
	GNavEKF.Propagate(GSimulationTimeSec);

	HeadingMeasurement HeadingMeas( pSensorPacket->CompassHeading );
	SpeedMeasurement SpeedMeas(ActiveVelocity);

	GNavEKF.UpdateMeasurement(HeadingMeas);
	GNavEKF.UpdateMeasurement(SpeedMeas);

	// Store this sensor packet as the 'last' sensor packet
	GLastSensorPacket = ThePacket;
}



//=============================================================================
/** Stores desired RPM from a controls packet read from the log file */
void HandleControlsPacket( const UAVNetPacket& ThePacket )
{
	ControlsPacket_t* pPacket = const_cast<ControlsPacket_t*>(ThePacket.AsControlsPacket());
	GRpmVelocityEstimate = ( (float)pPacket->DesiredThrust * VELPERRPM ) + VELOFFSET;

	_VERBOSE( MOOSTrace("  Desired RPM=%d   RPM->Velocity=%3.2f m/s\n",
			  pPacket->DesiredThrust, GRpmVelocityEstimate) );
}



//=============================================================================
/** Applies an LBL estimated position packet read from the log file */
void HandleNavPingPacket( const UAVNetPacket& ThePacket )
{
	LblEstPositionPacket_t* pLblPacket = const_cast<LblEstPositionPacket_t*>(ThePacket.AsLblEstPositionPacket());
	SensorPacket_t* pLastSensorPacket = const_cast<SensorPacket_t*>(GLastSensorPacket.AsSensorPacket());

	_VERBOSE( MOOSTrace("  Ranges: A=%3.2f  B=%3.2f  C=%3.2f  D=%3.2f   Depth: %3.2f cm\n",
					    pLblPacket->Range_A,
					    pLblPacket->Range_B,
					    pLblPacket->Range_C,
					    pLblPacket->Range_D,
					    pLastSensorPacket->Depth_cm) );

	//--------------------------------
	// Apply beacon ranges to Kalman.lib
	//--------------------------------
	KalmanUpdate( MEASURE_BUOY,
				  GRpmVelocityEstimate,
				  pLastSensorPacket->CompassHeading,
				  pLblPacket->Range_A,
				  pLblPacket->Range_B,
				  pLblPacket->Range_C,
				  pLblPacket->Range_D,
				  pLastSensorPacket->Depth_cm );

	//--------------------------------
	// Apply to VehicleEKF object
	//--------------------------------
	NEWMAT::Real BeaconRanges[4];
	BeaconRanges[0] = pLblPacket->Range_A;
	BeaconRanges[1] = pLblPacket->Range_B;
	BeaconRanges[2] = pLblPacket->Range_C;
	BeaconRanges[3] = pLblPacket->Range_D;

	// Create a beacon measurement object and update the EKF object
	LblBeaconMeasurement BeaconMeasurement( BeaconRanges,
											pLastSensorPacket->Depth_cm * 0.01);

	GNavEKF.UpdateMeasurement(BeaconMeasurement);

}


//=============================================================================
/** Applies a WHOI message packet read from the log file */
void HandleWhoiMsgPacket( const UAVNetPacket& ThePacket )
{
	_VERBOSE( MOOSTrace("\n") );
}


//=============================================================================
/** Applies an LBL beacon location packet read from the log file */
void HandleBeaconLocationPacket( const UAVNetPacket& ThePacket )
{
	LblBeaconLocationPacket_t* pBeaconPacket = const_cast<LblBeaconLocationPacket_t*>(ThePacket.AsLblBeaconLocationPacket());
	SensorPacket_t* pLastSensorPacket = const_cast<SensorPacket_t*>(GLastSensorPacket.AsSensorPacket());

	_VERBOSE( MOOSTrace("\n") );

	//--------------------------------
	// Apply to Kalman.lib
	//--------------------------------
	KalmanGetBuoyLocations( pBeaconPacket->BeaconA_N,
							pBeaconPacket->BeaconA_E,
							pBeaconPacket->BeaconA_Depth,
							pBeaconPacket->BeaconB_N,
							pBeaconPacket->BeaconB_E,
							pBeaconPacket->BeaconB_Depth,
							pBeaconPacket->BeaconC_N,
							pBeaconPacket->BeaconC_E,
							pBeaconPacket->BeaconC_Depth,
							pBeaconPacket->BeaconD_N,
							pBeaconPacket->BeaconD_E,
							pBeaconPacket->BeaconD_Depth,
							pBeaconPacket->OriginLatitude,
							pBeaconPacket->OriginLongitude );

	// Set local coordinate system origin
	GReferenceGeodesy.Initialise(pBeaconPacket->OriginLatitude, pBeaconPacket->OriginLongitude);

	// Initialize current estimated position in local coordinates
	GEstimatedCoord.N = (float)( (long)(pLastSensorPacket->GPS_Latitude*10000000L)
							  - pBeaconPacket->OriginLatitude ) * 0.0111311;
	GEstimatedCoord.E = (float)( (long)(pLastSensorPacket->GPS_Longitude*10000000L)
							  - pBeaconPacket->OriginLongitude) * 0.0111311
							  * cos(pLastSensorPacket->GPS_Latitude * PI/180.0 );

	// From line 732 in mission.lib:
	InitializeKalman( GEstimatedCoord.N,
					  GEstimatedCoord.E,
					  GRpmVelocityEstimate,
					  pLastSensorPacket->CompassHeading );

	// Set last EKF propagation time to simulation time for kalman.lib code
	// In the sub, the current MS_TIMER is captured for this purpose.
	GLastPropagateTime = GSimulationTimeSec;

	//--------------------------------
	// Apply packet to EKF object
	//--------------------------------

	// Set the location of all four navigation beacons in the vehicle EKF
	float* pBeaconCoord = &pBeaconPacket->BeaconA_N;
	for (int i = 0; i < 4; i++)
	{
		double BeaconN, BeaconE, BeaconDepth_m;
		LblBeacon& NavBeacon = GNavEKF.NavBeacon('A' + i);

		// Set beacon location
		BeaconN = *pBeaconCoord++;	// North (Y) coordinate
		BeaconE = *pBeaconCoord++;	// East (X) coordinate
		BeaconDepth_m = *pBeaconCoord++;	// Beacon depth (meters)

		NavBeacon.SetLocation_LocalGrid(BeaconE, BeaconN, BeaconDepth_m );
	}

	// Set sound speed in water
	GNavEKF.SetH20SoundVelocity(pBeaconPacket->SoundSpeedInH2O);

	GNavEKF.Initialize( GEstimatedCoord.E, GEstimatedCoord.N,
						GRpmVelocityEstimate,
						pLastSensorPacket->CompassHeading,
						whoi.ping_interval,
						GSimulationTimeSec );

}


//=============================================================================
/** Applies the contents of a single IMU message read from the log file */
void HandleSingleImuPacket( const UAVNetPacket& ThePacket )
{
	ImuMessage_t* pImuReading = const_cast<ImuMessage_t*>(&ThePacket.AsSingleImuPacket()->Reading);
	SensorPacket_t* pLastSensorPacket = const_cast<SensorPacket_t*>(GLastSensorPacket.AsSensorPacket());

	_VERBOSE( MOOSTrace("  GyroB=%3.2f   GyroC=%3.2f  AccelX=%3.2f  AccelY=%3.2f\n",
						pImuReading->GyroC, pImuReading->GyroB,
						pLastSensorPacket->Accelerometer_Y,
						pLastSensorPacket->Accelerometer_X) );

	//--------------------------------
	// Apply to Kalman.lib code
	//--------------------------------
	GiveZGyro( pImuReading->GyroC, pImuReading->GyroB,
			   pLastSensorPacket->Accelerometer_Y,
			   pLastSensorPacket->Accelerometer_X );


	//--------------------------------
	// Apply to EKF object
	//--------------------------------
	ImuMeasurement ImuMeasurement( pImuReading->GyroC,	/* wz */
								   pImuReading->GyroB,	/* wy */
								   MOOSDeg2Rad(pLastSensorPacket->Accelerometer_Y),	 /* phi */
								   MOOSDeg2Rad(pLastSensorPacket->Accelerometer_X) ); /* theta */

	GNavEKF.ApplyImuMeasurement(ImuMeasurement);
}



//=============================================================================
/** Applies a synchronous range packet read from the log file */
void HandleSynchRangePacket( const UAVNetPacket& ThePacket )
{
	SynchronousRangePacket_t* range_pp = const_cast<SynchronousRangePacket_t*>(ThePacket.AsSynchronousRangePacket());
	SensorPacket_t* pLastSensorPacket = const_cast<SensorPacket_t*>(GLastSensorPacket.AsSensorPacket());

	float ranges[4];

	_VERBOSE( MOOSTrace("\n") );

    if (range_pp->BeaconId > 0)
    {
		for (int i = 0; i < 4; i++)
		{
			ranges[i] = 0.0;
		}

		ranges[range_pp->BeaconId - 1] = range_pp->Range_m;
		KalmanUpdate( MEASURE_BUOY,
					  GRpmVelocityEstimate,
					  pLastSensorPacket->CompassHeading,
					  ranges[0],
					  ranges[1],
					  ranges[2],
					  ranges[3],
					  pLastSensorPacket->Depth_cm );

		//GEstimatedCoord.n = GetKalmanNorthPos(last_sensor_packet.lat);
		//GEstimatedCoord.e = GetKalmanEastPos(last_sensor_packet.lat,last_sensor_packet.lon);
    }

}


//=============================================================================
/** Applies a ship EKF packet read from the log file */
void HandleShipEkfPacket( const UAVNetPacket& ThePacket )
{
	_VERBOSE( MOOSTrace("\n") );
}


//=============================================================================
/** Applies a depth packet read from the log file */
void HandleDepthPacket( const UAVNetPacket& ThePacket )
{
	_VERBOSE( MOOSTrace("\n") );
}





//=============================================================================
/** Splits a TYPE_IMU_LOG packet into three TYPE_SINGLE_IMU_LOG packets
 *
 * @param [in] SourcePacket
 *	The TYPE_IMU packet to split
 *
 * @param [out] OUT_SingleMsgPacket
 *	An array of four UAVNetPacket objects to populate with the individual IMU
 *	messages contained in SourcePacket
 */
void SplitIMUPacket( const UAVNetPacket& SourcePacket,
					 UAVNetPacket OUT_SingleMsgPackets[] )
{
	const ImuPacket_t* pImuSource = SourcePacket.AsImuPacket();
	const ImuMessage_t* pSourceReading = pImuSource->Reading;
	SingleImuPacket_t SingleMsgPacket;

	if (OUT_SingleMsgPackets != NULL)
	{
		// Populate single message packet header
		memcpy(&SingleMsgPacket.Header, &pImuSource->Header, sizeof(PacketHeader_t));
		SingleMsgPacket.Header.Type = TYPE_EKFTEST_IMU_MSG;

		// For each IMU message in the source packet:
		//	- Copy the message's time stamp into the time stamp of the single
		//	  message packet's header.
		//	- Copy the message data into the single message packet's data
		for (int i = 0; i < 4; i++)
		{
			SingleMsgPacket.Header.MsTimeStamp = pImuSource->Reading[i].TimeStampMs;
			memcpy(&SingleMsgPacket.Reading, &pSourceReading[i], sizeof(ImuMessage_t));
			OUT_SingleMsgPackets[i].FromRawBytes(&SingleMsgPacket);
		}
	}

}




//=============================================================================
string PacketTypeToString( uint16_t PacketTypeId )
{
	string s;

	// A lookup table for packet type ID's
	static const char* szTypeStringTable[] = {
			"TYPE_GENERIC", "TYPE_GENERIC_LOG",
			"TYPE_SENSOR", "TYPE_SENSOR_LOG",
			"TYPE_CONTROLS", "TYPE_CONTROLS_LOG",
			"TYPE_COMMAND", "TYPE_COMMAND_LOG",
			"TYPE_LBL_POS", "TYPE_LBL_POS_LOG",
			"TYPE_WHM_MSG", "TYPE_WHM_MSG_LOG",
			"TYPE_GPGGA", "TYPE_GPGGA_LOG",
			"TYPE_BUOY_POS", "TYPE_BUOY_POS_LOG",
			"TYPE_KALMAN", "TYPE_KALMAN_LOG",
			"TYPE_IMU", "TYPE_IMU_LOG",
			"TYPE_TELEMETRY", "TYPE_TELEMETRY_LOG",
			"TYPE_AHRS", "TYPE_AHRS_LOG",
			"TYPE_SYNCH_RANGE", "TYPE_SYNCH_RANGE_LOG",
			"TYPE_KALMSHIP", "TYPE_KALMSHIP_LOG",
			"TYPE_DEPTH", "TYPE_DEPTH_LOG", "TYPE_EKFTEST_IMU_MSG"
		};

	if (PacketTypeId < UAVnet::NUM_PACKET_TYPEIDS)
	{
		s = string(szTypeStringTable[PacketTypeId]);
	}
	else
	{
		s = MOOSFormat("** Unknown Type: (%d) **", PacketTypeId);
	}

	return s;
}


//=============================================================================
string MsTimeStampToString( uint32_t TimeStampMs )
{
	int Hours, Minutes, Seconds, Ms;

	Seconds = TimeStampMs / 1000;			// Total whole seconds
	Ms = TimeStampMs - (Seconds * 1000);	// Fractional seconds
	Hours = Seconds / (60 * 60);			// Hours
	Minutes = (Seconds / 60) % 60;			// Minute in the hour
	Seconds = Seconds % 60;					// Second in the minute

	string s = MOOSFormat("[%02d:%02d:%02d.%03d]",
						  Hours, Minutes, Seconds, Ms);
	return s;
}



//=============================================================================
/** Calculates the percent error of the EKF object's state values with respect
 * to the state values calculated by Kalman.lib
 *
 * @return
 * 	false if the Kalman.lib and EKF object's state values have diverged beyond
 * 	a specified maximum percent error; else true if the state values are
 * 	within the specified error bound (i.e. EKF's are not diverging)
 */
bool TestEkfError( double MaxPercentError )
{
	// Calculate difference in states
	Real PercentError[VehicleEKF::NUM_STATES];
	Real KLibVal, EkfObjVal, Difference;
	bool ReturnVal = true;
	const ColumnVector& EkfObjStates = GNavEKF.GetStates();
	string s;

	// Calculate error in state values between Kalman.lib and EKF object.
	// Update statistics with these values
	for (int i = 0; i < VehicleEKF::NUM_STATES; i++)
	{
		KLibVal = static_cast<Real>(X[i][0]);
		EkfObjVal = EkfObjStates(i+1);

		// DB: the Kalman.lib implementation does not always bound the Yaw
		// estimate, so we bound both values being compared to the range +/- PI
		if (i == VehicleEKF::EST_YAW - 1)
		{
			KLibVal = MOOS_ANGLE_WRAP(KLibVal);
			EkfObjVal = MOOS_ANGLE_WRAP(EkfObjVal);
		}

		PercentError[i] = (KLibVal == 0.0) ? 0.0 : (EkfObjVal - KLibVal) / KLibVal * 100.0;
		GXStats[i].Update(PercentError[i]);
		Difference = fabs(EkfObjVal - KLibVal);

		// Flag divergence
		if (PercentError[i] > MaxPercentError)
		{
			if (Difference < 1e-5)
			{
				//MOOSTrace("Ignoring very small difference of %3.4e in state %d", Difference, i+1);
			}
			else
			{
				// Return false to indicate that error in state values exceeds
				// the specified maximum value
				ReturnVal = false;
			}
		}
	}

	/*
	MOOSTrace("States:\n");
	MOOSTrace("Kalman.lib: East: %-6.3f   North: %-6.3f   Speed: %-6.3f   "
			  "Yaw: %-6.3f   Bias: %-6.3f \n",
			  X[0][0], X[1][0], X[2][0], MOOS_ANGLE_WRAP(X[3][0]), X[4][0]);
	MOOSTrace("EKF Object: East: %-6.3f   North: %-6.3f   Speed: %-6.3f   "
			  "Yaw: %-6.3f   Bias: %-6.3f \n",
			  EkfObjStates(1), EkfObjStates(2), EkfObjStates(3),
			  MOOS_ANGLE_WRAP(EkfObjStates(4)), EkfObjStates(5) );
	MOOSTrace("Error(%):   East: %-6.3e%%  North: %-6.3e%%  Speed: %-6.3e%%  "
			  "Yaw: %-6.3e%%  Bias: %-6.3e%%\n\n",
			  PercentError[0], PercentError[1], PercentError[2],
			  PercentError[3], PercentError[4]);
	*/

	// Print Kalman.lib states as:  E, N, Speed, Yaw, YawBias
	//s = MsTimeStampToString( static_cast<uint32_t>(GSimulationTimeSec * 1000.0) ) + ",";
	s = MOOSFormat("%f,", GSimulationTimeSec);
	MOOSTrace( s + MOOSFormat("Kalman.lib,%e,%e,%e,%e,%e\n",
				  X[0][0], X[1][0], X[2][0],
				  MOOS_ANGLE_WRAP(X[3][0]), MOOS_ANGLE_WRAP(X[4][0])) );

	MOOSTrace( s + MOOSFormat("VehicleEKF,%e,%e,%e,%e,%e\n",
			  EkfObjStates(1), EkfObjStates(2), EkfObjStates(3),
			  MOOS_ANGLE_WRAP(EkfObjStates(4)),
			  MOOS_ANGLE_WRAP(EkfObjStates(5))) );

	return ReturnVal;	// Return true if the error in EKF states is within
						// the specified maximum range
}





//=============================================================================
void PrintFloatMatrix( float pData[][6], int NumRows, int NumColumns )
{
	for (int i = 0; i < NumRows; i++)
	{
		MOOSTrace("[ ");
		for (int j = 0; j < NumColumns; j++)
		{
			MOOSTrace("%-6.3f  ", pData[i][j]);
		}
		MOOSTrace(" ]\n");
	}
}




//=============================================================================
void PrintNewmatMatrix( const Matrix& TheMatrix )
{
	int NumRows = TheMatrix.Nrows();
	int NumColumns = TheMatrix.Ncols();

	for (int i = 1; i <= NumRows; i++)
	{
		MOOSTrace("[ ");
		for (int j = 1; j <= NumColumns; j++)
		{
			MOOSTrace("%-6.3f  ", TheMatrix(i,j));
		}
		MOOSTrace(" ]\n");
	}
}




void PrintEkfMatrices( void )
{
	MOOSTrace("<<<<  EKF Matrices  >>>>\n" );

	//-------------------
	// Print X
	//-------------------
	MOOSTrace("X (Kalman.lib)\n");
	PrintFloatMatrix(X, 5, 1);
	MOOSTrace("\n");

	MOOSTrace("X (EKF object)\n");
	PrintNewmatMatrix( GNavEKF.GetStates() );
	MOOSTrace("\n");


	//-------------------
	// Print P
	//-------------------
	MOOSTrace("P (Kalman.lib)\n");
	PrintFloatMatrix(P, 5, 5);
	MOOSTrace("\n");

	MOOSTrace("P (EKF object)\n");
	PrintNewmatMatrix( GNavEKF.GetP() );
	MOOSTrace("\n");


	//-------------------
	// Print Q
	//-------------------
	MOOSTrace("Q (Kalman.lib)\n");
	PrintFloatMatrix(Q, 5, 5);
	MOOSTrace("\n");

	MOOSTrace("Q (EKF object)\n");
	PrintNewmatMatrix( GNavEKF.GetQ() );
	MOOSTrace("\n");


	//-------------------
	// Print R
	//-------------------
	MOOSTrace("R (Kalman.lib)\n");
	PrintFloatMatrix(R, 6, 6);
	MOOSTrace("\n");

	MOOSTrace("R (EKF object)\n");
	PrintNewmatMatrix( GNavEKF.GetR() );
	MOOSTrace("\n");

}


/** Compares a float matrix with a Newmat matrix
 * @return
 * 	true if the contents of the two matrices are equal; else false
 */
/*bool CompareMatrices( const float* pFloatData[], const GeneralMatrix& Matrix )
{
	int NumRows = Matrix.Nrows();
	int NumColumns = Matrix.Ncols();

	for (int i = 0; i < NumRows; i++)
	{
		for (int j = 0; j < NumColumns; j++)
		{
			if ( pFloatData)
		}
	}
}
*/

