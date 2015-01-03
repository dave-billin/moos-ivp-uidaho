//=============================================================================
/** @file Kalman_lib.h
 *
 * @brief
 * 	Header file for the adapted Kalman.lib emulation module
 *
 * @author Dave Billin
 */
//=============================================================================

#ifndef _KALMAN_LIB_H_
#define _KALMAN_LIB_H_



typedef enum
{
	MEASURE_HEADING,
	MEASURE_BUOY,
	MEASURE_SHIP_E,
	MEASURE_SHIP_N,
	MEASURE_SHIP_S,
	MEASURE_SHIP_H,
} e_kalman_Measurement_Types;

typedef struct
{
	float State1;
	float State2;
	float State3;
	float State4;
	float State5;
	float P11;
	float P22;
	float P33;
	float P44;
	float P55;
	float DT;
	float Zgyro;
	float P12;
	float P13;
	float P14;
	float P15;
	float P23;
	float P24;
	float P25;
	float P34;
	float P35;
	float P45;
} KalmanParamStruct;

typedef struct
{
	float State1;
	float State2;
	float State3;
	float State4;
	float P11;
	float P22;
	float P33;
	float P44;
	float DT;
	float P12;
	float P13;
	float P14;
	float P23;
	float P24;
	float P34;
} KalmShipParamStruct;


//--------------------------------------------
// Definitions from acoustic_comms.lib
//--------------------------------------------
//list of possible communication modes
typedef enum {
	SILENT,          //  No Comms No Pings
	FLEET_COMMS,          //  3 vehicle 20 second comms cycle
	TRACKING_NAV_2,          //  2 Sec. Tracking and Nav pings
	TRACKING_NAV_16,          //  2 Sec. Tracking Pings 16 Sec. Nav
	SYNCH_NAV_6,          //  Synchronous Navigation 1sec
	SYNCH_NAV_12,          //  Synchronous Navigation 2sec
	SYNCH_FLEET_COMMS_30,          //  3 vehicle 30 second Synchronous Navigation fleet comms
	MLBL_COMMS_AUV_12,          //  MLBL test AUV comms cycle

   NUM_NAV_MODES,//Always the last one!!!
}acoustic_comms_mode_t;

// List of modem operational states for state machine.
typedef enum {
   WHM_INIT, WHM_CONFIG, WHM_MEAS_NOISE, WHM_SET_CLOCK, WHM_GPS_POS, WHM_SCHEDULE
   } whm_op_state_t;


//WHOI comms state machine data structure
// These variables must be initialized by Uhura before enabling whoi
typedef struct{
   whm_op_state_t op_state;     // WHOI communications state
   int ready;        // WHOI not available
   int src_addr;
   int ping_interval;// ping interval in seconds
   acoustic_comms_mode_t comms_mode; //
} whoi_control;





//============================
// EXPORTED FUNCTIONS:
//============================
void InitializeKalman( float North_Pos, float East_Pos, float Veh_Speed, float Veh_Heading );
void PropagateKalman( float TS );
void KalmanUpdate( e_kalman_Measurement_Types Meas_Type,float Meas1,float Meas2,float Meas3,float Meas4,float Meas5,float Meas6,float Depth );
void UpdateEquations(float H[][6], float M[][6], int N, float re[][6], float XEN[2], float uX[][6], float uP[][6], float uR[][6], e_kalman_Measurement_Types Meas_Type);
int Inv( int N , float A[][6] );
void Mult( float A[][6] , float B[][6] , int a , int b , int c , float C[][6] , int d);
void Add(  float A[][6] , float B[][6] , int a , int b , float C[][6] , int c);
void Equate(  float A[][6] , float B[][6] , int a , int b );
float GetKalmanEastPos( float lat, float lon );
float GetKalmanNorthPos( float lat );
float GetKalmanVelocity( float gps_vel );
float GetKalmanHeading( float Compass_Heading );
void KalmanInitializeFlag( void );
void KalmanStopPropagate( void );
void KalmanGetBuoyLocations( float BUOYA_North, float BUOYA_East, float BUOYA_Depth, float BUOYB_North,
							 float BUOYB_East, float BUOYB_Depth, float BUOYC_North, float BUOYC_East,
                             float BUOYC_Depth, float BUOYD_North, float BUOYD_East, float BUOYD_Depth,
                             long BUOYZ_North, long BUOYZ_East );
void GetKalmanParameters( KalmanParamStruct *Kalman_Parameters );
void GetKalmShipParameters( KalmShipParamStruct *KalmShip_Parameters );
int GetKalmanFlagStatus( void );
int GetKalmShipFlagStatus( void );
int GetBadJumpFlag( void );
int GetBadRangesFlag( void );
int GetFailedInvertFlag( void);
void GiveZGyro( float wz , float wy , float phi , float theta );
void InitializeKalmanShip( float North_Pos, float East_Pos, float Veh_Speed, float Veh_Heading );
void PropagateKalmanShip( float TS );
void ShipTransLoc(void);
void KalmanUpdateShip( e_kalman_Measurement_Types Meas_Type,float Meas1 );

/*** EndHeader */

//Macro Definitions
#define STATE_SIZE 5   //Length of State Vector
#ifndef PI
#define PI 3.14159
#endif
#define METERS_PER_CENTIMETER 0.01
#ifndef RADPERDEG
#define RADPERDEG (PI/180.0)
#endif

//Measurement Check Cutoffs
#define SPEED_CHECK 5.0		//If difference between measured and estimated value is greater than check, state is not updated, MUST be larger than "real" value to account for range related jumping
#define JUMP_CHECK 15.0		// Jump (post lbl update) which triggers mission abort


//-----------------------------
// GLOBAL VARIABLES
//-----------------------------
extern float X[6][6];
extern float P[6][6];
extern float Q[6][6];
extern float R[6][6];

//Ship EKF Variables
extern float X_s[6][6];
extern float P_s[6][6];
extern float Q_s[6][6];
extern float R_s[6][6];

extern float F[6][6];	//& DB: added for debugging purposes

//Safety Variables
extern float range_check;
extern int INITIALIZE_FLAG;
extern int INITIALIZE_FLAG_s;
extern int badjumpflag;//added12.3.08
extern int badrangesflag; //added 7.27.09
extern int badrangescondition; //added3.31.10
extern int failedinvertflag; //added 2.16.10

//Transponder Locations
extern float BUOYA_N;
extern float BUOYA_E;
extern float BUOYA_Z;
extern float BUOYB_N;
extern float BUOYB_E;
extern float BUOYB_Z;
extern float BUOYC_N;
extern float BUOYC_E;
extern float BUOYC_Z;
extern float BUOYD_N;
extern float BUOYD_E;
extern float BUOYD_Z;
extern long  BUOY_ZERO_N;
extern long  BUOY_ZERO_E;
//extern float tsploc[3][4];
extern float tsploc[6][6];	//& DB: changed to 6x6 to fix build error

//Current Time Step Value
extern float Time_Step;
extern float Time_Step_s;

//Current zgyro
extern float last_zgyro;
extern float last_used_zgyro;

extern whoi_control whoi;



#endif	// END #ifndef _KALMAN_LIB_H_
