//=============================================================================
/** @file Kalman_lib.cpp
 *
 * @brief
 * 	An adaptation of the Kalman.lib Dynamic C library used to test the MOOS EKF
 * 	class.
 */

#include <stdio.h>
#include <math.h>
#include "Kalman_lib.h"



//=============================================================================
/* Kalman.lib
 * Author: 	Ben Armstrong		arms7186@gmail.com
 *	Date:		2008/9/04
 * Description:  This Library contains all of the functions necessary for
 		an Extened Kalman Filter
 */
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// 					Revision History
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//
// July 1 2010 -- Ben Armstrong Amanda Folk --  Altered all 4 GetKalmanSTATE() functions for use in MLBL.
//							Functions now are passed in a value which is used if EKF is not running
//							BUOYP GPS zero point added.
//
// December 9 2010 -- Ben Armstrong Amanda Folk -- Alterations were done for a more robust ship ekf initialization
//								The ship ekf also now has its own logging packet
//								The transponders can now be passed in and aren't hard coded
//								Bug's have been significantly cleaned from the previous version
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

/*** BeginHeader InitializeKalman, PropagateKalman, KalmanUpdate, GetKalmanEastPos, GetKalmanNorthPos, GetKalmanVelocity, GetKalmanHeading, KalmanInitializeFlag, KalmanGetBuoyLocations, GiveZGyro, GetKalmanFlagStatus, GetBadJumpFlag, GetBadRangesFlag, GetFailedInvertFlag, InitializeKalmanShip, PropagateKalmanShip, ShipTransLoc, KalmanUpdateShip,GetKalmShipParameters,GetKalmShipFlagStatus*/

//Global Variables
float X[6][6];
float P[6][6];
float Q[6][6];
float R[6][6];

//Ship EKF Variables
float X_s[6][6];
float P_s[6][6];
float Q_s[6][6];
float R_s[6][6];

//float F[6][6];	//& DB: added for debugging purposes

//Safety Variables
float range_check;
int INITIALIZE_FLAG;
int INITIALIZE_FLAG_s;
int badjumpflag;//added12.3.08
int badrangesflag; //added 7.27.09
int badrangescondition; //added3.31.10
int failedinvertflag; //added 2.16.10

//Transponder Locations
float BUOYA_N;
float BUOYA_E;
float BUOYA_Z;
float BUOYB_N;
float BUOYB_E;
float BUOYB_Z;
float BUOYC_N;
float BUOYC_E;
float BUOYC_Z;
float BUOYD_N;
float BUOYD_E;
float BUOYD_Z;
long  BUOY_ZERO_N;
long  BUOY_ZERO_E;
//float tsploc[3][4];	//& DB: changed to 6x6 to fix compile error
float tsploc[6][6];

//Current Time Step Value
float Time_Step;
float Time_Step_s;

//Current zgyro
float last_zgyro;
float last_used_zgyro;

whoi_control whoi;

/************************************************************
*	Function Definition: InitializeKalman
*	Inputs: GPS_N, GPS_E, GPS_U, Heading
*	Outputs: void
*	Purpose: Initialize the state vector to the last known
*			GPS coordinates received before diving.
*			Initialize the error covariance matrix to ones
*			on diagonal so the filter will run.
*************************************************************/
void InitializeKalman( float North_Pos, float East_Pos, float Veh_Speed, float Veh_Heading )
{
	//Variable Definitions
	float sigma_range;
	float sigma_u;
	float sigma_h;
	float sigma_e_sys;
	float sigma_n_sys;
	float sigma_u_sys;
	float sigma_h_sys;
	float sigma_b_sys;
	float p_init_en;
	float p_init_u;
	float p_init_h;
	float p_init_b;
	//NOTE THAT range_check is a global variable as it is needed in Kalman Update
	//These variables aren't needed as their values are recorded in R, Q and P which are already global

	//Switch to select EKF coefficients depending on CommMode
	switch( whoi.ping_interval )
   {
   case 0:  //"Normal" Synchronous Mission
		//Measurement Check Cutoff
		range_check = 10.0;  // Range will be rejected if larger than this value, will count toward badrangesflag++

		//Measurement Noises (R values)
		sigma_range = 0.1289*0.1289;
		sigma_u     = 0.0212*0.0212;
		sigma_h     = 17.5128*RADPERDEG*17.5128*RADPERDEG;

		//System Noises (Q values)
		sigma_e_sys = 0.1269*0.1269;
		sigma_n_sys = 0.1269*0.1269;
		sigma_u_sys = 1.0490*1.0490;
		sigma_h_sys = 0.4220*RADPERDEG*0.4220*RADPERDEG;
		sigma_b_sys = 0.0217*RADPERDEG*0.0217*RADPERDEG;

		//Initial P Values
		p_init_en = 20.4849;
		p_init_u  = 2.7525;
		p_init_h  = 17.4712;
		p_init_b  = 3.3400;

		//Number of Rejected Ranges to Cause Abort
		badrangescondition = 4;
		break;
	case 2:  //"normal" 2 second navigation
		//Measurement Check Cutoff
		range_check = 10.2570;  // Range will be rejected if larger than this value, will count toward badrangesflag++

		//Measurement Noises (R values)
		sigma_range = 0.7051*0.7051;
		sigma_u     = 0.9856*0.9856;
		sigma_h     = 1.8855*RADPERDEG*1.8855*RADPERDEG;

		//System Noises (Q values)
		sigma_e_sys = 0.9723*0.9723;
		sigma_n_sys = 0.9723*0.9723;
		sigma_u_sys = 0.6819*0.6819;
		sigma_h_sys = 0.1586*RADPERDEG*0.1586*RADPERDEG;
		sigma_b_sys = 1.5667*RADPERDEG*1.5667*RADPERDEG;

		//Initial P Values
		p_init_en = 0.1;
		p_init_u  = 7.0;
		p_init_h  = 32.5;
		p_init_b  = 9.5;

		//Number of Rejected Ranges to Cause Abort
		badrangescondition = 10;
		break;
	case 3:  //MLBL Case
		//Measurement Check Cutoff
		range_check = 20;  // Range will be rejected if larger than this value, will count toward badrangesflag++

		//Measurement Noises (R values)
		sigma_range = 1.0000;
		sigma_u     = 0.0101;
		sigma_h     = 0.0101;

		//System Noises (Q values)
		sigma_e_sys = 10.077;
		sigma_n_sys = 10.071;
		sigma_u_sys = 0.1007;
		sigma_h_sys = 0.1054;
		sigma_b_sys = 0.1008;

		//Initial P Values
		p_init_en = 10.1;
		p_init_u  = 10.1;
		p_init_h  = 10.1;
		p_init_b  = 10.1;

		//Number of Rejected Ranges to Cause Abort
		badrangescondition = 10;
		break;
	case 15: //"Fleet"
	default://unrecognized CommMode will run 15 second EKF
		//Measurement Check Cutoff
		range_check = 18.9740;  // Range will be rejected if larger than this value, will count toward badrangesflag++

		//Measurement Noises (R values)
		sigma_range = 0.4462*0.4462;
		sigma_u     = 5.8860*5.8860;
		sigma_h     = 0.3462*RADPERDEG*0.3462*RADPERDEG;

		//System Noises (Q values)
		sigma_e_sys = 1.0803*1.0803;
		sigma_n_sys = 1.0803*1.0803;
		sigma_u_sys = 0.1082*0.1082;
		sigma_h_sys = 1.1936*RADPERDEG*1.1936*RADPERDEG;
		sigma_b_sys = 0.5516*RADPERDEG*0.5516*RADPERDEG;

		//Initial P Values
		p_init_en = 0.3023;
		p_init_u  = 0.0838;
		p_init_h  = 1.5912;
		p_init_b  = 0.0085;

		//Number of Rejected Ranges to Cause Abort
		badrangescondition = 4;
	break;
	}//switch( CommMode )

	//Set Initial State
	X[0][0] = East_Pos;
	X[1][0] = North_Pos;
	X[2][0] = Veh_Speed;
	X[3][0] = Veh_Heading * RADPERDEG; //In Radians (
	X[4][0] = 0; //Initial Compass Bias in radians
	//Initialize P matrix
	P[0][0] = p_init_en;   	P[0][1] = 0.0;   		P[0][2] = 0.0;   	P[0][3] = 0.0;   	P[0][4] = 0.0;
	P[1][0] = 0.0;   		P[1][1] = p_init_en;   	P[1][2] = 0.0;   	P[1][3] = 0.0;   	P[1][4] = 0.0;
	P[2][0] = 0.0;   		P[2][1] = 0.0;   		P[2][2] = p_init_u; P[2][3] = 0.0;   	P[2][4] = 0.0;
	P[3][0] = 0.0;   		P[3][1] = 0.0;   		P[3][2] = 0.0;   	P[3][3] = p_init_h; P[3][4] = 0.0;
	P[4][0] = 0.0;   		P[4][1] = 0.0;   		P[4][2] = 0.0;   	P[4][3] = 0.0;   	P[4][4] = p_init_b;
	//Setup Q matrix
	Q[0][0] = sigma_e_sys;	Q[0][1] = 0.0;				Q[0][2] = 0.0;				Q[0][3] = 0.0;				Q[0][4] = 0.0;
	Q[1][0] = 0.0;			   Q[1][1] = sigma_n_sys;	Q[1][2] = 0.0;				Q[1][3] = 0.0;				Q[1][4] = 0.0;
	Q[2][0] = 0.0;				Q[2][1] = 0.0;				Q[2][2] = sigma_u_sys;	Q[2][3] = 0.0;				Q[2][4] = 0.0;
	Q[3][0] = 0.0;				Q[3][1] = 0.0;				Q[3][2] = 0.0;				Q[3][3] = sigma_h_sys;	Q[3][4] = 0.0;
	Q[4][0] = 0.0;				Q[4][1] = 0.0;				Q[4][2] = 0.0;				Q[4][3] = 0.0;				Q[4][4] =  sigma_b_sys;
	//Setup R matrix
   R[0][0] = sigma_range;	R[0][1] = 0.0;				R[0][2] = 0.0;				R[0][3] = 0.0;				R[0][4] = 0.0;			R[0][5] = 0.0;
	R[1][0] = 0.0;				R[1][1] = sigma_range;	R[1][2] = 0.0;				R[1][3] = 0.0;				R[1][4] = 0.0; 		R[1][5] = 0.0;
	R[2][0] = 0.0;				R[2][1] = 0.0;				R[2][2] = sigma_range;	R[2][3] = 0.0;				R[2][4] = 0.0; 		R[2][5] = 0.0;
	R[3][0] = 0.0;				R[3][1] = 0.0;				R[3][2] = 0.0;				R[3][3] = sigma_range;	R[3][4] = 0.0; 		R[3][5] = 0.0;
	R[4][0] = 0.0;				R[4][1] = 0.0;				R[4][2] = 0.0;				R[4][3] = 0.0;				R[4][4] = sigma_u; 	R[4][5] = 0.0;
	R[5][0] = 0.0;				R[5][1] = 0.0;				R[5][2] = 0.0;				R[5][3] = 0.0;				R[5][4] = 0.0;			R[5][5] = sigma_h;

	INITIALIZE_FLAG = 1;
    badjumpflag = 0;//added12.3.08
    badrangesflag = 0; //added 7.27.09
	failedinvertflag = 0;//added 2.16.10
   #ifdef KALMAN_LIB_VERBOSE
		printf("\n INITIALIZE FLAG = %d \n",INITIALIZE_FLAG);
   #endif
}



/************************************************************
*	Function Definition: PropagateKalman
*	Inputs: void
*	Outputs: void
*	Purpose: Propagate the state forward using the system
*			model. Propagate the error covariance forward.
*************************************************************/
void PropagateKalman( float TS )
{
	float F[6][6];
	float A[6][6];
	float B[6][6];
	Time_Step = TS;
   if(INITIALIZE_FLAG == 1)
	{
   	#ifdef KALMAN_LIB_VERBOSE
      	//printf("\n\t\t\t\t\t PROPOGATING \n");
      	//printf("\n\t\t\t\t\t E: %.3f N: %.3f U: %.3f H: %.3f B: %.3f \n",X[0][0],X[1][0],X[2][0],X[3][0],X[4][0]);
      #endif
		// ENSPB
		F[0][0] = 1.0;	F[0][1] = 0.0;	F[0][2] = TS*sin(X[3][0]);	F[0][3] =  TS*X[2][0]*cos(X[3][0]);		F[0][4] = 0.0;
		F[1][0] = 0.0;	F[1][1] = 1.0;	F[1][2] = TS*cos(X[3][0]);	F[1][3] =  -TS*X[2][0]*sin(X[3][0]);	F[1][4] = 0.0;
		F[2][0] = 0.0;	F[2][1] = 0.0;	F[2][2] = 1.0;				F[2][3] = 0.0;							F[2][4] = 0.0;
		F[3][0] = 0.0;	F[3][1] = 0.0;	F[3][2] = 0.0;				F[3][3] = 1.0;							F[3][4] = 0.0;
		F[4][0] = 0.0;	F[4][1] = 0.0;	F[4][2] = 0.0;				F[4][3] = 0.0;							F[4][4] = 1.0;

		//& *** DEBUG ***
/*		printf("\nF (Kalman.lib):\n");
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				printf("%10.5f   ", F[i][j]);
			}
			printf("\n");
		}
*/		//& *** END DEBUG ***


		//NOTE: Zgyro IS ADDED AS CONTROL
		//ENSPB
		X[0][0] = X[0][0] + X[2][0]*TS*sin(X[3][0]);
		X[1][0] = X[1][0] + X[2][0]*TS*cos(X[3][0]);
		X[2][0] = X[2][0];
		X[3][0] = X[3][0] + TS*last_zgyro;
		X[4][0] = X[4][0];

		last_used_zgyro = last_zgyro; //for logging ensures logged zgyro was from the current step

		//P = F*P*F' + L*Q*L'
		//F*P
		Mult( F , P , 5 , 5 , 5 , A , 0 ); // A = FP 5x5

		//A*F'
		Mult( A , F , 5 , 5 , 5 , B , 1 ); // B = FPF' 5x5

		//B+Q
		Add( B , Q , 5 , 5 , P , 1 ); // P = F*P*F + L*Q*L w/ L=eye(5)

      #ifdef KALMAN_LIB_VERBOSE
      	//printf("\n\t\t\t\t\t E: %.3f N: %.3f U: %.3f H: %.3f B: %.3f \n",X[0][0],X[1][0],X[2][0],X[3][0],X[4][0]);
      #endif
   }
}


/************************************************************
*	Function Definition: KalmanUpdate
*	Inputs: Meas_Type, Measurement (Heading and Speed or Ranges Up to 4 ranges),Depth
*	Outputs: void
*	Purpose: Using Meas_Type, decide which measurement is
*			sent in Meas. Using measurement data calculate
*			the measurement residual, kalman gain, and
*			state error. Use these values to update the
*			state vector and the error covariance matrix.
*			can handle up to 6x6 matrix,but currently is
*			setup for a heading and speed OR Ranges
*			with up to 4 ranges
*
*		Heading Update:
*			Meas1 = velocity estimate (from GPS or prop RPM)
*			Meas2 = heading
*************************************************************/
void KalmanUpdate( e_kalman_Measurement_Types Meas_Type,float Meas1,float Meas2,float Meas3,float Meas4,float Meas5,float Meas6,float Depth )
{
	float H[6][6];		//Observation matrix
	float M[6][6];		//Measurement Noise Map
	float re[6][6];		//measurement residual
	float XEN[2];       //added12.3.08 moved 7.2.10
	int i, j, N;		//Number of measurements (dynamic with range rejection)
	float Range_Est;	//Estimated range (used to find residual on range)

	//---------------------------------------------------------------------
	// DB: Added code to initialize float matrices declared on the stack
	/*
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			H[i][j] = 0.0;
			M[i][j] = 0.0;
			re[i][j] = 0.0;
		}
	}

	XEN[0] = 0.0;
	XEN[1] = 0.0;
	*/
	//---------------------------------------------------------------------


   #ifdef KALMAN_LIB_VERBOSE
      	printf("\n\t\t\t\t\t\t\t\t\t\t UPDATE  ");
         if(Meas_Type == MEASURE_HEADING) printf("HS \n");
         else if(Meas_Type == MEASURE_BUOY) printf("B \n");
         //printf("\n\t\t\t\t\t\t\t\t\t\t E: %.3f N: %.3f U: %.3f H: %.3f B: %.3f \n",X[0][0],X[1][0],X[2][0],X[3][0],X[4][0]);
   #endif

    M[0][0] = 0.0;  M[0][1] = 0.0; M[0][2] = 0.0;  M[0][3] = 0.0; M[0][4] = 0.0;  M[0][5] = 0.0;
	M[1][0] = 0.0;  M[1][1] = 0.0; M[1][2] = 0.0;  M[1][3] = 0.0; M[1][4] = 0.0;  M[1][5] = 0.0;
	M[2][0] = 0.0;  M[2][1] = 0.0; M[2][2] = 0.0;  M[2][3] = 0.0; M[2][4] = 0.0;  M[2][5] = 0.0;
	M[3][0] = 0.0;  M[3][1] = 0.0; M[3][2] = 0.0;  M[3][3] = 0.0; M[3][4] = 0.0;  M[3][5] = 0.0;
	M[4][0] = 0.0;  M[4][1] = 0.0; M[4][2] = 0.0;  M[4][3] = 0.0; M[4][4] = 0.0;  M[4][5] = 0.0;
  	M[5][0] = 0.0;  M[5][1] = 0.0; M[5][2] = 0.0;  M[5][3] = 0.0; M[5][4] = 0.0;  M[5][5] = 0.0;

	Depth *= METERS_PER_CENTIMETER;

	if( INITIALIZE_FLAG == 1 )
	{
		switch (Meas_Type)
		{
			  case MEASURE_HEADING:
				N = 2;
				H[0][0] = 0.0; H[0][1] = 0.0; H[0][2] = 1.0; H[0][3] = 0.0; H[0][4] = 0.0;
				H[1][0] = 0.0; H[1][1] = 0.0; H[1][2] = 0.0; H[1][3] = 1.0; H[1][4] = 1.0;

				M[0][4] = 1.0;
				M[1][5] = 1.0;

				re[0][0] = ( Meas1 -  X[2][0] );

				if(( re[0][0] * re[0][0] ) > ( SPEED_CHECK * SPEED_CHECK))
				{
					re[0][0] = 0.0;
				}

				re[1][0] = ( (Meas2 * RADPERDEG) - (X[3][0] + X[4][0]) ); // compass bias
				//re[1][0] = ( (Meas2 * RADPERDEG) - X[3][0] ); // no compass bias

				while( re[1][0] > +PI ) re[1][0] -= (2 * PI);
				while( re[1][0] < -PI ) re[1][0] += (2 * PI);

				//& *** DEBUG ***
/*				printf("\nH (kalman.lib)\n");
				for (int i = 0; i < 2; i++)
				{
					for (int j = 0; j < 5; j++)
					{
						printf("%10.5f   ", H[i][j]);
					}
					printf("\n");
				}

				printf("\nM (kalman.lib)\n");
				for (int i = 0; i < 2; i++)
				{
					for (int j = 0; j < 6; j++)
					{
						printf("%10.5f   ", M[i][j]);
					}
					printf("\n");
				}

				printf("\nre (kalman.lib)\n");
				for (int i = 0; i < 2; i++)
				{
					printf("%10.5f", re[i][0]);
				}

				printf("\n\n");
*/
				//& *** END DEBUG ***

				break;

			  case MEASURE_BUOY:
				XEN[0]=X[0][0];//added12.3.08
				XEN[1]=X[1][0];//added12.3.08
				N = 0;

				//& *** DEBUG ***
/*				printf("\nX (kalman.lib)\n");
				for (int i = 0; i < 5; i++)
				{
					printf("%10.5f\n", X[i][0]);
				}
				printf("\nDepth: %10.5f\n", Depth);
*/				//& *** END DEBUG ***

				//LBL
				 if( Meas3!=0.0 )
					{
					Range_Est = sqrt((X[0][0]-BUOYA_E) * (X[0][0]-BUOYA_E) + (X[1][0]-BUOYA_N) * (X[1][0]-BUOYA_N) + (Depth-BUOYA_Z) * (Depth-BUOYA_Z));
					//& *** DEBUG ***
/*						printf("\nkalman.lib [Beacon A]\n");
						printf("   dEast:     %10.5f\n", X[0][0]-BUOYA_E);
						printf("   dNorth:    %10.5f\n", X[1][0]-BUOYA_N);
						printf("   dDepth:    %10.5f\n", Depth-BUOYA_Z);
						printf("   Range_Est: %10.5f\n", Range_Est);
*/					//& *** END DEBUG ***

					re[N][0] = Meas3 - Range_Est;
					if(re[N][0]*re[N][0]<(range_check * range_check))
						{
						H[N][0] = (X[0][0] - BUOYA_E)/Range_Est;
						H[N][1] = (X[1][0] - BUOYA_N)/Range_Est;
						H[N][2] = 0.0;
						H[N][3] = 0.0;
						H[N][4] = 0.0;

						M[N][0] = 1.0;

						N++;
						badrangesflag = -4;
						}
					else
						{
						badrangesflag++;
						}
					}
				if( Meas4!=0.0 )
					{
					Range_Est = sqrt((X[0][0]-BUOYB_E) * (X[0][0]-BUOYB_E) + (X[1][0]-BUOYB_N) * (X[1][0]-BUOYB_N) + (Depth-BUOYB_Z) * (Depth-BUOYB_Z));
					//& *** DEBUG ***
/*						printf("\nkalman.lib [Beacon B]\n");
						printf("   dEast:     %10.5f\n", X[0][0]-BUOYB_E);
						printf("   dNorth:    %10.5f\n", X[1][0]-BUOYB_N);
						printf("   dDepth:    %10.5f\n", Depth-BUOYB_Z);
						printf("   Range_Est: %10.5f\n", Range_Est);
*/					//& *** END DEBUG ***
					re[N][0] = Meas4 - Range_Est;
					if( re[N][0]*re[N][0]<(range_check * range_check) )
						{
						H[N][0] = (X[0][0] - BUOYB_E)/Range_Est;
						H[N][1] = (X[1][0] - BUOYB_N)/Range_Est;
						H[N][2] = 0.0;
						H[N][3] = 0.0;
						H[N][4] = 0.0;

						M[N][1] = 1.0;

						N++;
						badrangesflag = -4;
						}
					else
						{
						badrangesflag++;
						}
					}
				if( Meas5!=0.0 )
					{
					Range_Est = sqrt((X[0][0]-BUOYC_E) * (X[0][0]-BUOYC_E) + (X[1][0]-BUOYC_N) * (X[1][0]-BUOYC_N) + (Depth-BUOYC_Z) * (Depth-BUOYC_Z));
					//& *** DEBUG ***
/*						printf("\nkalman.lib [Beacon C]\n");
						printf("   dEast:     %10.5f\n", X[0][0]-BUOYC_E);
						printf("   dNorth:    %10.5f\n", X[1][0]-BUOYC_N);
						printf("   dDepth:    %10.5f\n", Depth-BUOYC_Z);
						printf("   Range_Est: %10.5f\n", Range_Est);
*/					//& *** END DEBUG ***
					re[N][0] = Meas5 - Range_Est;
					if( re[N][0]*re[N][0]<(range_check * range_check) )
						{
						H[N][0] = (X[0][0] - BUOYC_E)/Range_Est;
						H[N][1] = (X[1][0] - BUOYC_N)/Range_Est;
						H[N][2] = 0.0;
						H[N][3] = 0.0;
						H[N][4] = 0.0;

						M[N][2] = 1.0;

						N++;
						badrangesflag = -4;
						}
					else
						{
						badrangesflag++;
						}
					}
				if( Meas6!=0.0 )
					{
					Range_Est = sqrt((X[0][0]-BUOYD_E) * (X[0][0]-BUOYD_E) + (X[1][0]-BUOYD_N) * (X[1][0]-BUOYD_N) + (Depth-BUOYD_Z) * (Depth-BUOYD_Z));
					//& *** DEBUG ***
/*						printf("\nkalman.lib [Beacon D]\n");
						printf("   dEast:     %10.5f\n", X[0][0]-BUOYD_E);
						printf("   dNorth:    %10.5f\n", X[1][0]-BUOYD_N);
						printf("   dDepth:    %10.5f\n", Depth-BUOYD_Z);
						printf("   Range_Est: %10.5f\n", Range_Est);
*/					//& *** END DEBUG ***
					re[N][0] = Meas6 - Range_Est;
					if( re[N][0]*re[N][0]<(range_check * range_check) )
						{
						H[N][0] = (X[0][0] - BUOYD_E)/Range_Est;
						H[N][1] = (X[1][0] - BUOYD_N)/Range_Est;
						H[N][2] = 0.0;
						H[N][3] = 0.0;
						H[N][4] = 0.0;

						M[N][3] = 1.0;

						N++;
						badrangesflag = -4;
						}
					else
						{
						badrangesflag++;
						}
					}
				if( badrangesflag < 0 )
					{
					badrangesflag=0;
					}

				//& *** DEBUG ***
/*				if (N == 0)	break;
				printf("\nH (kalman.lib)\n");
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < 5; j++)
					{
						printf("%10.5f   ", H[i][j]);
					}
					printf("\n");
				}

				printf("\nM (kalman.lib)\n");
				for (int i = 0; i < N; i++)
				{
					for (int j = 0; j < 6; j++)
					{
						printf("%10.5f   ", M[i][j]);
					}
					printf("\n");
				}

				printf("\nre (kalman.lib)\n");
				for (int i = 0; i < N; i++)
				{
					printf("%10.5f\n", re[i][0]);
				}

				printf("\n\n");

*/				//& *** END DEBUG ***
				break;
			}//End Switch

		#ifdef KALMAN_LIB_VERBOSE
		if( N==0 && Meas_Type==MEASURE_BUOY) printf("\n\t\t\t\t\t\t\t\t\t\t ZERO RANGES  \n"); // RANGE UPDATE WITH ZERO ACCEPTED RANGES
		#endif

		if( N==0 && Meas_Type==MEASURE_BUOY) return; // RANGE UPDATE WITH ZERO ACCEPTED RANGES

		UpdateEquations(H, M, N, re, XEN, X, P, R, Meas_Type); //Update Equations with AUV X P R
	}//End if initialized
}	//End function

/************************************************************
*	Function Definition: UpdateEquations
*	Inputs: H M N re XEN X P R Meas_Type
*	Outputs: void
*	Purpose: Performs the Update Equations for an EKF Update
*************************************************************/
void UpdateEquations(float H[][6], float M[][6], int N, float re[][6], float XEN[2], float uX[][6], float uP[][6], float uR[][6], e_kalman_Measurement_Types Meas_Type)
{
	float A[6][6];		//Intermediate Solution Variable
	float B[6][6];		//Intermediate Solution Variable
	float C[6][6];		//Intermediate Solution Variable
	float K[6][6];		//Kalman Gain
	float I[6][6];		//Identity matrix

	I[0][0] = 1.0;  I[0][1] = 0.0; I[0][2] = 0.0;  I[0][3] = 0.0; I[0][4] = 0.0;  I[0][5] = 0.0;
	I[1][0] = 0.0;  I[1][1] = 1.0; I[1][2] = 0.0;  I[1][3] = 0.0; I[1][4] = 0.0;  I[1][5] = 0.0;
	I[2][0] = 0.0;  I[2][1] = 0.0; I[2][2] = 1.0;  I[2][3] = 0.0; I[2][4] = 0.0;  I[2][5] = 0.0;
	I[3][0] = 0.0;  I[3][1] = 0.0; I[3][2] = 0.0;  I[3][3] = 1.0; I[3][4] = 0.0;  I[3][5] = 0.0;
	I[4][0] = 0.0;  I[4][1] = 0.0; I[4][2] = 0.0;  I[4][3] = 0.0; I[4][4] = 1.0;  I[4][5] = 0.0;
  	I[5][0] = 0.0;  I[5][1] = 0.0; I[5][2] = 0.0;  I[5][3] = 0.0; I[5][4] = 0.0;  I[5][5] = 1.0;

      #ifdef KALMAN_LIB_VERBOSE
      	//printf("\n\t\t\t\t\t\t\t\t\t\t MATRIX  \n");
      #endif

		// B = (HxPxtranspose(H)+MxRxtranspose(M))

		//HxP
		Mult( H , uP , N , 5 , 5 , A , 0 ); // A = HP Nx5
		//AxH'
		Mult( A , H , N , 5 , N , B , 1 ); // B = HPH' NxN
		//MxR
		Mult( M , uR , N , 6 , 6 , A , 0 ); // A = MR Nx6
		//AxM'
		Mult( A , M , N , 6 , N , C , 1 ); // C = MRM' NxN
		//B+C
		Add( B , C , N , N , A , 1 ); // A = HPH' + MRM'

   	#ifdef KALMAN_LIB_VERBOSE
      	//printf("\n\t\t\t\t\t\t\t\t\t\t START INVERT  \n");
      #endif

		//invert HPH' + MRM'
		if( Inv( N , A ) == 1 ) // A = (HPH' + MRM')^(-1)
      {
   		#ifdef KALMAN_LIB_VERBOSE
	         //printf("\n\t\t\t\t\t\t\t\t\t\t INVERT SUCCESSFUL  \n");
	      #endif

	      // K = Pxtranspose(H)xA

	      //PxH'
	      Mult( uP , H , 5 , 5 , N , B , 1 ); // B = PH' 5xN
	      //BxA
	      Mult( B , A , 5 , N , N , K , 0 ); // K = PH'A 5xN

	      #ifdef KALMAN_LIB_VERBOSE
	         //printf("\n\t\t\t\t\t\t\t\t\t\t FOUND K  \n");
	      #endif

	      // Pb = (I-KxH)xP

	      //KxH
	      Mult( K , H , 5 , N , 5 , B , 0 ); // B = KH 5x5
	      //I-B
	      Add( I , B , 5 , 5 , C , -1 ); // C = I-KH 5x5
	      //CxP
	      Mult( C , uP , 5 , 5 , 5 , B , 0 ); // B = (I-KH)P 5x5

	      // P = B
	      Equate(  uP , B , 5 ,  5 ); // P = P 5x5

	      #ifdef KALMAN_LIB_VERBOSE
	         //printf("\n\t\t\t\t\t\t\t\t\t\t FOUND P  \n");
	      #endif

	      // X = X + Kxre
	      //Kxre
	      Mult( K , re , 5 , N , 1 , B , 0 ); // B = Kre 5x1
	      //X+B
	      Add( uX , B , 5 , 1 , C , 1 ); // C = X + Kre 5x1
	      //X=C
	      Equate(  uX , C , 5 ,  1 ); // X = X 5x1

	      #ifdef KALMAN_LIB_VERBOSE
	         //printf("\n\t\t\t\t\t\t\t\t\t\t FOUND X  \n");
	      #endif

	      while( uX[3][0] > (2 * PI) ) uX[3][0] -= (2 * PI);
	      while( uX[3][0] < 0 ) uX[3][0] += (2 * PI);

	      if(Meas_Type==MEASURE_BUOY)//added12.3.08
	      {
	         if( ((XEN[0] - X[0][0])*(XEN[0] - X[0][0]) > (JUMP_CHECK*JUMP_CHECK)) || ((XEN[1] - X[1][0])*(XEN[1] - X[1][0]) > (JUMP_CHECK*JUMP_CHECK)))//added12.3.08
	         {
	            badjumpflag=1;//added12.3.08
	         }
	      }
      }
      else
      {
       	failedinvertflag = 1;//failed invert; changed on 2.16.10 [BA] ; there is no update performed abrot occurs NEXT sensor packet
      }
      #ifdef KALMAN_LIB_VERBOSE
      	printf("\n\t\t\t\t\t\t\t\t\t\t SUCCESS  \n");
        printf("\n\t\t\t\t\t\t\t\t\t\t E: %.3f N: %.3f U: %.3f H: %.3f B: %.3f \n",X[0][0],X[1][0],X[2][0],X[3][0],X[4][0]);
      #endif



}//end function
/************************************************************
*	Function Definition: Equate()
*	Inputs: A, B, a, b
*	Outputs: sets B == A for (0->i,0->j)
*	Purpose: Equate to matrices axb (both A and B)
*************************************************************/
void Equate(  float A[][6] , float B[][6] , int a , int b )
{
	int i;
	int j;

	for ( i = 0; i<=a-1; i++ )
	{
		for ( j = 0; j<=b-1; j++ )
		{
			A[i][j] = B[i][j];
		}
	}
}
/************************************************************
*	Function Definition: Add()
*	Inputs: A, B, a, b, C, c
*	Outputs: sets C = A+c*B so that C=A+B or C=A-B
*	Purpose: Add or Subtract two matrices
*************************************************************/
void Add(  float A[][6] , float B[][6] , int a , int b , float C[][6] , int c)
{
	int i;
	int j;

	for ( i = 0; i<=a-1; i++ )
	{
		for ( j = 0; j<=b-1; j++ )
		{
			C[i][j] = A[i][j] + c * B[i][j];
		}
	}
}
/************************************************************
*	Function Definition: Mult()
*	Inputs: A, B, a, b, c, C, d
*	Outputs: C = A*B where A(axb) and B(bxc) if d=1 then transpose B
*	Purpose: Multiply two matrices.
*************************************************************/
void Mult(  float A[][6] , float B[][6] , int a , int b , int c , float C[][6] , int d)
{
	int i;
	int j;
	int k;
	float D[6][6];

	if( d==1 ) //transpose
	{
		for ( i = 0; i<=b-1; i++ )
		{
			for ( j = 0; j<=c-1; j++ )
			{
				D[i][j] = B[j][i];
			}
		}
	}

	for ( i = 0; i<=a-1; i++ )
	{
		for ( j = 0; j<=c-1; j++ )
		{
			C[i][j] = 0;
			for ( k = 0; k<=b-1; k++ )
			{
			if( d==0 ) C[i][j] += A[i][k]*B[k][j];
			else C[i][j] += A[i][k]*D[k][j];
			}
		}
	}
}
/************************************************************
*	Function Definition: Inv()
*	Inputs: N I where N is the size of the matrix to invert and I is the input and output matrix
*	Outputs: I inverted, where N is the size of I NxN and an integer of success or failure
*	Purpose: invert a matrix.
*************************************************************/
int Inv( int N , float I[][6])
{
	//A and D-C*iA*B must be non-singular
	float A[6][6];
	float B[6][6];
	float C[6][6];
	float D[6][6];
	float E[6][6];
	float F[6][6];
	float G[6][6];
	float H[6][6];
	int flag;


	if(N==6)
	{
		A[0][0] = I[0][0];  A[0][1] = I[0][1]; A[0][2] = I[0][2];
		A[1][0] = I[1][0];  A[1][1] = I[1][1]; A[1][2] = I[1][2];
		A[2][0] = I[2][0];  A[2][1] = I[2][1]; A[2][2] = I[2][2];

		B[0][0] = I[0][3];  B[0][1] = I[0][4]; B[0][2] = I[0][5];
		B[1][0] = I[1][3];  B[1][1] = I[1][4]; B[1][2] = I[1][5];
		B[2][0] = I[2][3];  B[2][1] = I[2][4]; B[2][2] = I[2][5];

		C[0][0] = I[3][0];  C[0][1] = I[3][1]; C[0][2] = I[3][2];
		C[1][0] = I[4][0];  C[1][1] = I[4][1]; C[1][2] = I[4][2];
		C[2][0] = I[5][0];  C[2][1] = I[5][1]; C[2][2] = I[5][2];

		D[0][0] = I[3][3];  D[0][1] = I[3][4]; D[0][2] = I[3][5];
		D[1][0] = I[4][3];  D[1][1] = I[4][4]; D[1][2] = I[4][5];
		D[2][0] = I[5][3];  D[2][1] = I[5][4]; D[2][2] = I[5][5];

		if( Inv( 3 , A )==0 ) //now A is inv(A)
		return 0;
		//D-C*A*B
		Mult( C , A , 3 , 3 , 3 , E , 0 ); // E = C*A 3x3
		Mult( E , B , 3 , 3 , 3 , F , 0 ); // F = C*A*B 3x3
		Add( D , F , 3 , 3 , E , -1 ); // E = D-C*A*B 3x3
		if( Inv( 3 , E )==0 ) //now E = inv(D-C*A*B) 3x3
		return 0;

		//Ca = A + A*B*E*C*A;
		Mult( A , B , 3 , 3 , 3 , F , 0 ); //F = A*B 3x3
		Mult( F , E , 3 , 3 , 3 , G , 0 ); //G = A*B*E 3x3
		Mult( G , C , 3 , 3 , 3 , F , 0 ); //F = A*B*E*C 3x3
		Mult( F , A , 3 , 3 , 3 , H , 0 ); //H = A*B*E*C*A 3x3
		Add( A , H , 3, 3, I, 1 ); //I = A + A*B*E*C*A 3x3

		//Cb = -A*B*E;
		//but G = A*B*E already
		I[0][3] = -G[0][0];  I[0][4] = -G[0][1]; I[0][5] = -G[0][2];
		I[1][3] = -G[1][0];  I[1][4] = -G[1][1]; I[1][5] = -G[1][2];
		I[2][3] = -G[2][0];  I[2][4] = -G[2][1]; I[2][5] = -G[2][2];

		//Cc = -E*C*A;
		Mult( E , C , 3 , 3 , 3 , F , 0 ); //F = E*C 3x3
		Mult( F , A , 3 , 3 , 3 , G , 0 ); //G = E*C*A 3x3

		I[3][0] = -G[0][0];  I[3][1] = -G[0][1]; I[3][2] = -G[0][2];
		I[4][0] = -G[1][0];  I[4][1] = -G[1][1]; I[4][2] = -G[1][2];
		I[5][0] = -G[2][0];  I[5][1] = -G[2][1]; I[5][2] = -G[2][2];

		//Cd = E;
		I[3][3] = E[0][0];  I[3][4] = E[0][1]; I[3][5] = E[0][2];
		I[4][3] = E[1][0];  I[4][4] = E[1][1]; I[4][5] = E[1][2];
		I[5][3] = E[2][0];  I[5][4] = E[2][1]; I[5][5] = E[2][2];
	}
	else if(N==5)
	{
		A[0][0] = I[0][0];  A[0][1] = I[0][1];
		A[1][0] = I[1][0];  A[1][1] = I[1][1];

		B[0][0] = I[0][2];  B[0][1] = I[0][3]; B[0][2] = I[0][4];
		B[1][0] = I[1][2];  B[1][1] = I[1][3]; B[1][2] = I[1][4];

		C[0][0] = I[2][0];  C[0][1] = I[2][1];
		C[1][0] = I[3][0];  C[1][1] = I[3][1];
		C[2][0] = I[4][0];  C[2][1] = I[4][1];

		D[0][0] = I[2][2];  D[0][1] = I[2][3]; D[0][2] = I[2][4];
		D[1][0] = I[3][2];  D[1][1] = I[3][3]; D[1][2] = I[3][4];
		D[2][0] = I[4][2];  D[2][1] = I[4][3]; D[2][2] = I[4][4];

		if( Inv( 2 , A )==0 ) //now A is inv(A)
		return 0;
		//D-C*A*B
		Mult( C , A , 3 , 2 , 2 , E , 0 ); // E = C*A 3x2
		Mult( E , B , 3 , 2 , 3 , F , 0 ); // F = C*A*B 3x3
		Add( D , F , 3 , 3 , E , -1 ); // E = D-C*A*B 3x3
		if( Inv( 3 , E )==0 ) //now E = inv(D-C*A*B) 3x3
		return 0;

		//Ca = A + A*B*E*C*A;
		Mult( A , B , 2 , 2 , 3 , F , 0 ); //F = A*B 2x3
		Mult( F , E , 2 , 3 , 3 , G , 0 ); //G = A*B*E 2x3
		Mult( G , C , 2 , 3 , 2 , F , 0 ); //F = A*B*E*C 2x2
		Mult( F , A , 2 , 2 , 2 , H , 0 ); //H = A*B*E*C*A 2x2
		Add( A , H , 2, 2, I, 1 ); //I = A + A*B*E*C*A 2x2

		//Cb = -A*B*E;
		//but G = A*B*E already
		I[0][2] = -G[0][0];  I[0][3] = -G[0][1]; I[0][4] = -G[0][2];
		I[1][2] = -G[1][0];  I[1][3] = -G[1][1]; I[1][4] = -G[1][2];

		//Cc = -E*C*A;
		Mult( E , C , 3 , 3 , 2 , F , 0 ); //F = E*C 3x2
		Mult( F , A , 3 , 2 , 2 , G , 0 ); //G = E*C*A 3x2

		I[2][0] = -G[0][0];  I[2][1] = -G[0][1];
		I[3][0] = -G[1][0];  I[3][1] = -G[1][1];
		I[4][0] = -G[2][0];  I[4][1] = -G[2][1];

		//Cd = E;
		I[2][2] = E[0][0];  I[2][3] = E[0][1]; I[2][4] = E[0][2];
		I[3][2] = E[1][0];  I[3][3] = E[1][1]; I[3][4] = E[1][2];
		I[4][2] = E[2][0];  I[4][3] = E[2][1]; I[4][4] = E[2][2];
	}
	else if(N==4)
	{
		A[0][0] = I[0][0];  A[0][1] = I[0][1];
		A[1][0] = I[1][0];  A[1][1] = I[1][1];

		B[0][0] = I[0][2];  B[0][1] = I[0][3];
		B[1][0] = I[1][2];  B[1][1] = I[1][3];

		C[0][0] = I[2][0];  C[0][1] = I[2][1];
		C[1][0] = I[3][0];  C[1][1] = I[3][1];

		D[0][0] = I[2][2];  D[0][1] = I[2][3];
		D[1][0] = I[3][2];  D[1][1] = I[3][3];

		if( Inv( 2 , A )==0 ) //now A is inv(A)
		return 0;
		//D-C*A*B
		Mult( C , A , 2 , 2 , 2 , E , 0 ); // E = C*A 2x2
		Mult( E , B , 2 , 2 , 2 , F , 0 ); // F = C*A*B 2x2
		Add( D , F , 2 , 2 , E , -1 ); // E = D-C*A*B 2x2
		if( Inv( 2 , E )==0 ) //now E = inv(D-C*A*B) 2x2
		return 0;

		//Ca = A + A*B*E*C*A;
		Mult( A , B , 2 , 2 , 2 , F , 0 ); //F = A*B 2x2
		Mult( F , E , 2 , 2 , 2 , G , 0 ); //G = A*B*E 2x2
		Mult( G , C , 2 , 2 , 2 , F , 0 ); //F = A*B*E*C 2x2
		Mult( F , A , 2 , 2 , 2 , H , 0 ); //H = A*B*E*C*A 2x2
		Add( A , H , 2, 2, I, 1 ); //I = A + A*B*E*C*A 2x2

		//Cb = -A*B*E;
		//but G = A*B*E already
		I[0][2] = -G[0][0];  I[0][3] = -G[0][1];
		I[1][2] = -G[1][0];  I[1][3] = -G[1][1];

		//Cc = -E*C*A;
		Mult( E , C , 2 , 2 , 2 , F , 0 ); //F = E*C 2x2
		Mult( F , A , 2 , 2 , 2 , G , 0 ); //G = E*C*A 2x2

		I[2][0] = -G[0][0];  I[2][1] = -G[0][1];
		I[3][0] = -G[1][0];  I[3][1] = -G[1][1];

		//Cd = E;
		I[2][2] = E[0][0];  I[2][3] = E[0][1];
		I[3][2] = E[1][0];  I[3][3] = E[1][1];
	}
	else if(N==3)
	{
		A[0][0] = I[0][0];

		B[0][0] = I[0][1];  B[0][1] = I[0][2];

		C[0][0] = I[1][0];
		C[1][0] = I[2][0];

		D[0][0] = I[1][1];  D[0][1] = I[1][2];
		D[1][0] = I[2][1];  D[1][1] = I[2][2];

		if( A[0][0]==0.00001 )
		return 0;
		A[0][0] = 1.0/A[0][0];//now A is inv(A)
		//D-C*A*B
		Mult( C , A , 2 , 1 , 1 , E , 0 ); // E = C*A 2x1
		Mult( E , B , 2 , 1 , 2 , F , 0 ); // F = C*A*B 2x2
		Add( D , F , 2 , 2 , E , -1 ); // E = D-C*A*B 2x2
		if( Inv( 2 , E )==0 ) //now E = inv(D-C*A*B) 2x2
		return 0;

		//Ca = A + A*B*E*C*A;
		Mult( A , B , 1 , 1 , 2 , F , 0 ); //F = A*B 1x2
		Mult( F , E , 1 , 2 , 2 , G , 0 ); //G = A*B*E 1x2
		Mult( G , C , 1 , 2 , 1 , F , 0 ); //F = A*B*E*C 1x1
		Mult( F , A , 1 , 1 , 1 , H , 0 ); //H = A*B*E*C*A 1x1
		Add( A , H , 1, 1, I, 1 ); //I = A + A*B*E*C*A 1x1

		//Cb = -A*B*E;
		//but G = A*B*E already
		I[0][1] = -G[0][0];  I[0][2] = -G[0][1];

		//Cc = -E*C*A;
		Mult( E , C , 2 , 2 , 1 , F , 0 ); //F = E*C 2x1
		Mult( F , A , 2 , 1 , 1 , G , 0 ); //G = E*C*A 2x1

		I[1][0] = -G[0][0];
		I[2][0] = -G[1][0];

		//Cd = E;
		I[1][1] = E[0][0];  I[1][2] = E[0][1];
		I[2][1] = E[1][0];  I[2][2] = E[1][1];
	}
	else if(N==2)
	{
	B[0][0] = I[0][0]*I[1][1]-I[0][1]*I[1][0]; //det I
	 if( (B[0][0]*B[0][0]) < (0.00001*0.00001) )
	 return 0;
	 E[1][1] = I[0][0]/B[0][0];

	 I[0][0] = I[1][1]/B[0][0];
	 I[0][1] = -I[0][1]/B[0][0];
	 I[1][0] = -I[1][0]/B[0][0];
	 I[1][1] = E[1][1];
	}
	else if(N==1)
	{
	 if( (I[0][0]*I[0][0]) < (0.00001*0.00001) ) //watch for divide by zero
	 return 0;

	 I[0][0] = 1.0/I[0][0];

	}
   return 1;
}

/************************************************************
*	Function Definition: GetKalmanEastPos()
*	Inputs: void
*	Outputs: X[0] - Vehicle East Position
*	Purpose: Return the current east position from state.
*************************************************************/
float GetKalmanEastPos( float lat, float lon )
{
if( INITIALIZE_FLAG == 0 )
   {
    lat = (float)((long)(lon*10000000L) - BUOY_ZERO_E ) * 0.0111311 * cos((lat) * PI / 180.0 );
   	return lat;
   }
   else
   {
		return X[0][0];
   }
}




/************************************************************
*	Function Definition: GetKalmanNorthPos()
*	Inputs: void
*	Outputs: X[1] - Vehicle North Position
*	Purpose: Return the current north position from state.
*************************************************************/
float GetKalmanNorthPos( float lat )
{
   if( INITIALIZE_FLAG == 0 )
   {
    lat = (float)((long)(lat*10000000L) - BUOY_ZERO_N) * 0.0111311;
   	return lat;
   }
   else
   {
		return X[1][0];
   }
}




/************************************************************
*	Function Definition: GetKalmanVelocity)
*	Inputs: void
*	Outputs: X[2] - Vehicle Velocity
*	Purpose: Return the current velocity from state.
*************************************************************/
float GetKalmanVelocity( float gps_vel )
{
if( INITIALIZE_FLAG == 0 )
   {
   	return gps_vel;
   }
   else
   {
		return X[2][0];
   }

}





/************************************************************
*	Function Definition: GetKalmanHeading()
*	Inputs: void
*	Outputs: X[3] - Vehicle Heading
*	Purpose: Return the current heading from state.
*************************************************************/
float GetKalmanHeading( float Compass_Heading )
{
	if( INITIALIZE_FLAG == 0 )
   {
   	return Compass_Heading;
   }
   else
   {
		return ( X[3][0] / RADPERDEG );
   }
}




/************************************************************
*	Function Definition:Kalman_GetBuoyLocations()
*	Inputs: X,Y,Z locations of all the LBL buoys.
*	Outputs: void
*	Purpose: This function is meant to get buoy locations
				from Kirk so Kalman filter can see them.
*************************************************************/
void KalmanGetBuoyLocations( float BUOYA_North, float BUOYA_East, float BUOYA_Depth, float BUOYB_North,
									  float BUOYB_East, float BUOYB_Depth, float BUOYC_North, float BUOYC_East,
                             float BUOYC_Depth, float BUOYD_North, float BUOYD_East, float BUOYD_Depth,
							 long BUOYZ_North, long BUOYZ_East)
{
	BUOYA_E = BUOYA_East;
	BUOYA_N = BUOYA_North;
	BUOYA_Z = BUOYA_Depth;
	BUOYB_E = BUOYB_East;
	BUOYB_N = BUOYB_North;
	BUOYB_Z = BUOYB_Depth;
	BUOYC_E = BUOYC_East;
	BUOYC_N = BUOYC_North;
	BUOYC_Z = BUOYC_Depth;
	BUOYD_E = BUOYD_East;
	BUOYD_N = BUOYD_North;
	BUOYD_Z = BUOYD_Depth;
	BUOY_ZERO_N = BUOYZ_North;
	BUOY_ZERO_E = BUOYZ_East;
	tsploc[0][0] = BUOYA_N; tsploc[0][1] = BUOYB_N; tsploc[0][2] = BUOYC_N; tsploc[0][3] = BUOYD_N; //North of each buoy relative to the ship
	tsploc[1][0] = BUOYA_E; tsploc[1][1] = BUOYB_E; tsploc[1][2] = BUOYC_E; tsploc[1][3] = BUOYD_E; // East of each buoy relative to the ship
	tsploc[2][0] = 1;       tsploc[2][1] = 1;       tsploc[2][2] = 1;       tsploc[2][3] = 1; // Used for matrix multiplication

}

/************************************************************
*	Function Definition: KalmanInitializeFlag()
*	Inputs: void
*	Outputs: void
*	Purpose: This function initializes the flag to zero when the sub
*				is turned on.
*************************************************************/
void KalmanInitializeFlag( void )
{
	INITIALIZE_FLAG = 0;
   INITIALIZE_FLAG_s=0;
	badjumpflag = 0;//added12.3.08
	badrangesflag=0;//added2.12.10
	failedinvertflag = 0; //added 2.16.10
	last_zgyro=0;//added2.12.10
	last_used_zgyro=0;//added3.10.10
	badrangescondition=10;//added 3.31.10
}

/************************************************************
*	Function Definition: KalmanInitializeFlagSwitch()
*	Inputs: void
*	Outputs: void
*	Purpose: This function switches the flag showing the kalman
*				filter has been initialized.
*************************************************************/
void KalmanStopPropagate( void )
{
	INITIALIZE_FLAG = 0;
	X[0][0] = 0.0;
	X[1][0] = 0.0;
	X[2][0] = 0.0;
	X[3][0] = 0.0;
	X[4][0] = 0.0;
	badjumpflag=0;
	badrangesflag=0;
	failedinvertflag=0;

   #ifdef KALMAN_LIB_VERBOSE
     	printf("\n INITIALIZE FLAG = %d \n",INITIALIZE_FLAG);
   #endif

   //Stop Ship EKF
   INITIALIZE_FLAG_s = 0;
    X_s[0][0] = 0.0;
	X_s[1][0] = 0.0;
	X_s[2][0] = 0.0;
	X_s[3][0] = 0.0;
}

/************************************************************
*	Function Definition: GetKalmanParameters()
*	Inputs: Address of kalman parameter struct in mission.
*	Outputs: void
*	Purpose: This function fills in the parameter struct so it can be
*				logged from mission.lib.
*************************************************************/
void GetKalmanParameters( KalmanParamStruct *Kalman_Parameters )
{
	Kalman_Parameters->State1 = X[0][0];
	Kalman_Parameters->State2 = X[1][0];
	Kalman_Parameters->State3 = X[2][0];
	Kalman_Parameters->State4 = X[3][0] / RADPERDEG;
	Kalman_Parameters->State5 = X[4][0] / RADPERDEG;
	Kalman_Parameters->P11    = P[0][0];
	Kalman_Parameters->P22    = P[1][1];
	Kalman_Parameters->P33    = P[2][2];
	Kalman_Parameters->P44    = P[3][3];
	Kalman_Parameters->P55    = P[4][4];
	Kalman_Parameters->DT     = Time_Step;
	Kalman_Parameters->Zgyro  = last_used_zgyro;
	Kalman_Parameters->P12	  = P[0][1];
	Kalman_Parameters->P13	  = P[0][2];
	Kalman_Parameters->P14	  = P[0][3];
	Kalman_Parameters->P15	  = P[0][4];
	Kalman_Parameters->P23	  = P[1][2];
	Kalman_Parameters->P24	  = P[1][3];
	Kalman_Parameters->P25	  = P[1][4];
	Kalman_Parameters->P34	  = P[2][3];
	Kalman_Parameters->P35	  = P[2][4];
	Kalman_Parameters->P45	  = P[3][4];
}

/************************************************************
*	Function Definition: GetKalmShipParameters()
*	Inputs: Address of kalman ship parameter struct in mission.
*	Outputs: void
*	Purpose: This function fills in the parameter struct so it can be
*				logged from mission.lib.
*************************************************************/
void GetKalmShipParameters( KalmShipParamStruct *KalmShip_Parameters )
{
	KalmShip_Parameters->State1 = X_s[0][0];
	KalmShip_Parameters->State2 = X_s[1][0];
	KalmShip_Parameters->State3 = X_s[2][0];
	KalmShip_Parameters->State4 = X_s[3][0] / RADPERDEG;
	KalmShip_Parameters->P11	= P_s[0][0];
	KalmShip_Parameters->P22	= P_s[1][1];
	KalmShip_Parameters->P33	= P_s[2][2];
	KalmShip_Parameters->P44	= P_s[3][3];
	KalmShip_Parameters->DT		= Time_Step_s;
	KalmShip_Parameters->P12	= P_s[0][1];
	KalmShip_Parameters->P13	= P_s[0][2];
	KalmShip_Parameters->P14	= P_s[0][3];
	KalmShip_Parameters->P23	= P_s[1][2];
	KalmShip_Parameters->P24	= P_s[1][3];
	KalmShip_Parameters->P34	= P_s[2][3];
}
/************************************************************
*	Function Definition: int GetKalmanFlagSatus( void )
*	Inputs: void
*	Outputs: Status o initialize flag (0 or 1)
*	Purpose: This function returns the status of the filter.
*
*************************************************************/
int GetKalmanFlagStatus( void )
{
	return INITIALIZE_FLAG;
}

/************************************************************
*	Function Definition: int GetKalmShipFlagStatus( void )
*	Inputs: void
*	Outputs: Status of initialize flag (0 or 1)
*	Purpose: This function returns the status of the filter.
*
*************************************************************/
int GetKalmShipFlagStatus( void )
{
	return INITIALIZE_FLAG_s;
}

/************************************************************
*	Function Definition: int GetBadJumpFlag( void )
*	Inputs: void
*	Outputs: Status of initialize flag (0 or 1)
*	Purpose: This function returns the status of the flag.
*
*************************************************************/
int GetBadJumpFlag( void )
{
   return badjumpflag;
}

/************************************************************
*	Function Definition: int GetBadRangesFlag void )
*	Inputs: void
*	Outputs: Status of initialize flag (0 or 1)
*	Purpose: This function returns the status of the flag.
*
*************************************************************/
int GetBadRangesFlag( void )
{
	if( badrangesflag >= badrangescondition )
	{
		return 1;
	}
	return 0;
}

/************************************************************
*	Function Definition: int GetFailedInvertFlag void )
*	Inputs: void
*	Outputs: Status of failedinvertflag (0 or 1)
*	Purpose: This function returns the status of the flag.
*
*************************************************************/
int GetFailedInvertFlag( void)
{
   return failedinvertflag;
}

/************************************************************
*	Function Definition: int GiveZGyro( float zgyro )
*	Inputs: zgyro
*	Outputs: none
*	Purpose: gives filter zgyro from imu
*
*************************************************************/
void GiveZGyro( float wz , float wy , float phi , float theta )
{
	phi *= RADPERDEG;
	theta *= RADPERDEG;
	last_zgyro = ((wy*sin(phi)+wz*cos(phi))*(1/cos(theta)))*RADPERDEG*100;
}
/************************************************************
*	Function Definition: InitializeKalmanShip
*	Inputs: N_s, E_s, S_s, P_s
*	Outputs: void
*	Purpose: Initialize the state vector of the ship to the best
*			known start location.
*			Initialize the error covariance matrix to ones
*			on diagonal so the filter will run.
*************************************************************/

void InitializeKalmanShip( float North_Pos, float East_Pos, float Veh_Speed, float Veh_Heading )
{
	//Set Initial State
	X_s[0][0] = East_Pos;
	X_s[1][0] = North_Pos;
	X_s[2][0] = Veh_Speed;
	X_s[3][0] = Veh_Heading; //In Radians
	//Initialize P matrix
	P_s[0][0] = 50.0;  		P_s[0][1] = 0.0;   		P_s[0][2] = 0.0;   		P_s[0][3] = 0.0;
	P_s[1][0] = 0.0;   		P_s[1][1] = 50.0;   	P_s[1][2] = 0.0;   		P_s[1][3] = 0.0;
	P_s[2][0] = 0.0;   		P_s[2][1] = 0.0;   		P_s[2][2] = 1.0;   		P_s[2][3] = 0.0;
	P_s[3][0] = 0.0;   		P_s[3][1] = 0.0;   		P_s[3][2] = 0.0;   		P_s[3][3] = 1.0;
	//Setup Q matrix
	Q_s[0][0] = 50.0;  		Q_s[0][1] = 0.0;		Q_s[0][2] = 0.0;		Q_s[0][3] = 0.0;
	Q_s[1][0] = 0.0;		Q_s[1][1] = 50.0;		Q_s[1][2] = 0.0;		Q_s[1][3] = 0.0;
	Q_s[2][0] = 0.0;		Q_s[2][1] = 0.0;		Q_s[2][2] = 1.0;		Q_s[2][3] = 0.0;
	Q_s[3][0] = 0.0;		Q_s[3][1] = 0.0;		Q_s[3][2] = 0.0;		Q_s[3][3] = 1.0;
	//Setup R matrix
    R_s[0][0] = 1.0;	    R_s[0][1] = 0.0;		R_s[0][2] = 0.0;		R_s[0][3] = 0.0;
	R_s[1][0] = 0.0;		R_s[1][1] = 1.0;		R_s[1][2] = 0.0;		R_s[1][3] = 0.0;
	R_s[2][0] = 0.0;		R_s[2][1] = 0.0;		R_s[2][2] = 1.0;	    R_s[2][3] = 0.0;
	R_s[3][0] = 0.0;		R_s[3][1] = 0.0;		R_s[3][2] = 0.0;		R_s[3][3] = 1.0;
	INITIALIZE_FLAG_s = 1;
   #ifdef KALMAN_LIB_VERBOSE
		printf("\n INITIALIZE FLAG_s = %d \n",INITIALIZE_FLAG_s);
   #endif
}

/************************************************************
*	Function Definition: PropagateKalmanShip
*	Inputs: TS
*	Outputs: void
*	Purpose: Propagate the state of the ship forward using the system
*			model. Propagate the error covariance forward.
*************************************************************/

void PropagateKalmanShip( float TS )
{
	float F[6][6];
	float A[6][6];
	float B[6][6];
	Time_Step_s = TS;
   if(INITIALIZE_FLAG_s == 1)
	{
   	#ifdef KALMAN_LIB_VERBOSE
      	printf("\n\t\t\t\t\t PROPOGATING_SHIP \n");
      	printf("\n\t\t\t\t\t E_s: %.3f N_s: %.3f S_s: %.3f H_s: %.3f \n",X_s[0][0],X_s[1][0],X_s[2][0],X_s[3][0]);
      #endif
		//E,N,s,psi
		F[0][0] = 1.0;	F[0][1] = 0.0;	F[0][2] = TS*sin(X_s[3][0]);	F[0][3] =  TS*X_s[2][0]*cos(X_s[3][0]);
		F[1][0] = 0.0;	F[1][1] = 1.0;	F[1][2] = TS*cos(X_s[3][0]);	F[1][3] =  -TS*X_s[2][0]*sin(X_s[3][0]);
		F[2][0] = 0.0;	F[2][1] = 0.0;	F[2][2] = 1.0;				    F[2][3] = 0.0;
		F[3][0] = 0.0;	F[3][1] = 0.0;	F[3][2] = 0.0;				    F[3][3] = 1.0;

		X_s[0][0] = X_s[0][0] + X_s[2][0]*TS*sin(X_s[3][0]);
		X_s[1][0] = X_s[1][0] + X_s[2][0]*TS*cos(X_s[3][0]);
		X_s[2][0] = X_s[2][0];
		X_s[3][0] = X_s[3][0];

		//P = F*P*F' + L*Q*L'
		//F*P
		Mult( F , P_s , 4 , 4 , 4 , A , 0 ); // A = FP 4x4

		//A*F'
		Mult( A , F , 4 , 4 , 4 , B , 1 ); // B = FPF' 4x4

		//B+Q
		Add( B , Q_s , 4 , 4 , P_s , 1 ); // P = F*P*F + L*Q*L w/ L=eye(4)

      #ifdef KALMAN_LIB_VERBOSE
      	//printf("\n\t\t\t\t\t E_s: %.3f N_s: %.3f S_s: %.3f H_s: %.3f \n",X_s[0][0],X_s[1][0],X_s[2][0],X_s[3][0]);
      #endif
   }
}

/************************************************************
*	Function Definition: ShipTransLoc
*	Inputs: n/a
*	Outputs: n/a
*	Purpose: Move the buoys to the ship's position
*		    The buoy is assumed to be at 0,0 on the ship
*************************************************************/
void ShipTransLoc(void)
{
//float transform[3][3];	//& DB changed from 3x3 to 6x6 to fix compile error
float transform[6][6];
//float tsp1[3][4];			//& DB changed from 3x4 to 6x6 to fix compile error
float tsp1[6][6];

transform[0][0] = cos(X_s[3][0]); transform[0][1] = -sin(X_s[3][0]); transform[0][2] = X_s[1][0];
transform[1][0] = sin(X_s[3][0]); transform[1][1] = cos(X_s[3][0]);  transform[1][2] = X_s[0][0];
transform[2][0] = 0;              transform[2][1] = 0;               transform[2][2] = 1;

//transform*tsploc
//void Mult(  float A[][6] , float B[][6] , int a , int b , int c , float C[][6] , int d)
Mult(transform,tsploc,3,3,4,tsp1,0); //tsp1 = transform*tsploc

    BUOYA_E = tsp1[1][0]; // East is pulled from second row
	BUOYA_N = tsp1[0][0]; // North is pulled from first row
	BUOYB_E = tsp1[1][1];
	BUOYB_N = tsp1[0][1];
	BUOYC_E = tsp1[1][2];
	BUOYC_N = tsp1[0][2];
	BUOYD_E = tsp1[1][3];
	BUOYD_N = tsp1[0][3];
	#ifdef KALMAN_LIB_VERBOSE
      	printf("\n\t\t\t\t\t BUOYS MOVED A: %.3f %.3f N,E\n",BUOYA_N,BUOYA_E);  //Buoy A Position message
    #endif
}

/************************************************************
*	Function Definition: KalmanUpdateShip
*	Inputs: Meas_Type, Measurement (North East Heading and
*			Speed or Ranges Up to 4 ranges),Depth
*	Outputs: void
*	Purpose: Using Meas_Type, decide which measurement is
*			sent in Meas. Using measurement data calculate
*			the measurement residual, kalman gain, and
*			state error. Use these values to update the
*			state vector and the error covariance matrix.
*			can handle up to 6x6 matrix,but currently is
*			setup for north, east, heading and speed
*			OR Ranges with up to 4 ranges
*************************************************************/
void KalmanUpdateShip( e_kalman_Measurement_Types Meas_Type,float Meas1 )
{
	float H[6][6];		//Observation matrix
	float M[6][6];		//Measurement Noise Map
	float re[6][6];		//measurement residual
    float XX[2];
    int N;

    XX[0]=0.0; XX[1]=0.0;

    M[0][0] = 0.0;  M[0][1] = 0.0; M[0][2] = 0.0;  M[0][3] = 0.0; M[0][4] = 0.0;  M[0][5] = 0.0;
	M[1][0] = 0.0;  M[1][1] = 0.0; M[1][2] = 0.0;  M[1][3] = 0.0; M[1][4] = 0.0;  M[1][5] = 0.0;
	M[2][0] = 0.0;  M[2][1] = 0.0; M[2][2] = 0.0;  M[2][3] = 0.0; M[2][4] = 0.0;  M[2][5] = 0.0;
	M[3][0] = 0.0;  M[3][1] = 0.0; M[3][2] = 0.0;  M[3][3] = 0.0; M[3][4] = 0.0;  M[3][5] = 0.0;
	M[4][0] = 0.0;  M[4][1] = 0.0; M[4][2] = 0.0;  M[4][3] = 0.0; M[4][4] = 0.0;  M[4][5] = 0.0;
  	M[5][0] = 0.0;  M[5][1] = 0.0; M[5][2] = 0.0;  M[5][3] = 0.0; M[5][4] = 0.0;  M[5][5] = 0.0;

	if( INITIALIZE_FLAG_s == 1 )
	{
		switch (Meas_Type)
		{
			  case MEASURE_SHIP_E:
				N = 1;
				H[0][0] = 1.0; H[0][1] = 0.0; H[0][2] = 0.0; H[0][3] = 0.0;

				M[0][0] = 1.0;

				re[0][0] = ( Meas1 -  X_s[0][0] );
				break;
				case MEASURE_SHIP_N:
				N = 1;
				H[0][0] = 0.0; H[0][1] = 1.0; H[0][2] = 0.0; H[0][3] = 0.0;

				M[0][1] = 1.0;

				re[0][0] = ( Meas1 -  X_s[1][0] );
				break;
				case MEASURE_SHIP_S:
				N = 1;
				H[0][0] = 0.0; H[0][1] = 0.0; H[0][2] = 1.0; H[0][3] = 0.0;

				M[0][2] = 1.0;

				re[0][0] = ( Meas1 -  X_s[2][0] );
				break;
				case MEASURE_SHIP_H:
				N = 1;
				H[0][0] = 0.0; H[0][1] = 0.0; H[0][2] = 0.0; H[0][3] = 1.0;

				M[0][3] = 1.0;

				re[0][0] = ( Meas1 -  X_s[3][0] );
				while( re[0][0] > +PI ) re[0][0] -= (2 * PI);
				while( re[0][0] < -PI ) re[0][0] += (2 * PI);
				break;
			}//End Switch


		UpdateEquations(H, M, N, re, XX, X_s, P_s, R_s, Meas_Type); //Update Equations with Ship X P R
		ShipTransLoc();
	}//End if initialized
}	//End function
