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
/** @file IMU3Module.h
 *
 * @brief
 *	A class encapsulating communications with an Archangel IM3 IMU module
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#ifndef IMU3MODULE_H_
#define IMU3MODULE_H_

#include <stdint.h>
#include <YellowSubUtils.h>


#define IMU3_PACKET_SIZE  70 /* Number of Bytes in IMU and AHRS packets */




//=============================================================================
//=============================================================================
/** A class to wrap communications with an Archangel IM3 IMU module connected
 *  via serial port
 *
 * @details
 *  To use this class, create an IMU3Module object, and call its
 *  ProcessSerialData() method to process data received from a serial port
 *  connected to the Archangel IMU3 module
 *
 * @ingroup iArchangelIMU
 */
//=============================================================================
//=============================================================================
class IMU3Module
{
public:

	/** Creates an instance of the object
	 * @param SerialPort
	 * 	A reference to the serial port connected to the IM3 module
	 */
	IMU3Module(int Verbosity = 0);

	/** Called when the object goes out of scope */
	~IMU3Module();


    //=========================================================================
    /** Parses data received from a serial port connected to the IMU3 module
     *
     * @param pRxData
     *	Pointer to a buffer of received data Bytes to parse
     *
     * @param NumBytes
     * 	The number of Bytes pointed to by pRxData
     *
     * @returns
     * 	true if the received data was parsed without errors; else false if an
     * 	error was detected during parsing.
     */
    void ProcessSerialData( char* pRxData, uint32_t NumBytes );


    //=========================================================================
    /** Returns true if data has been received from the IMU3 within the last
     * three seconds. */
    bool IsConnected( void ) const { return m_ImuIsConnected; }


    //=============================
    // IMU3 measurement accessors
    //=============================

    /** Returns true if values in an IMU message have been received since the
     *  last time this function was called.
     */
    bool ImuDataIsFresh( void );


    /** Returns a bitfield of latched flags indicating the status of IMU3
     *  self-tests (see e_SelfTestFlagBits for details)
     *
     * @details
     *  Calling this function clears all self-test flags
     */
    int GetSelfTestErrorFlags( void );


    /** Returns true if the IMU is running its initial 45-second self-alignment
     *  routine following power-up
     */
    bool IsAligning( void ) const { return m_ImuIsAligning; };


    /** Returns the delta body roll angle in degrees */
    inline float DeltaRollAngle( void ) const { return m_DeltaRoll_deg; }

    /** Returns the delta body pitch angle in degrees */
    inline float DeltaPitchAngle( void ) const { return m_DeltaPitch_deg; }

    /** Returns the delta body yaw angle in degrees */
    inline float DeltaYawAngle( void ) const { return m_DeltaYaw_deg; }

    /** Returns the delta body longitudinal velocity (meters per second ) */
    inline float DeltaVLongitudinal( void ) const { return m_dVLongitudinal; }

    /** Returns the delta body latitudinal velocity (meters per second ) */
    inline float DeltaVLateral( void ) const { return m_dVLateral; }

    /** Returns the delta body normal velocity (meters per second ) */
    inline float DeltaVNormal( void ) const { return m_dVNormal; }



    /** Returns true if the values in an AHRS message have been received since
     *  the last time this function was called.
     */
    bool AhrsDataIsFresh( void );

    /** Returns the inertial roll angle in degrees */
    inline float InertialRoll( void ) const { return m_InertialRoll_deg; }

    /** Returns the inertial pitch angle in degrees */
    inline float InertialPitch( void ) const { return m_InertialPitch_deg; }

    /** Returns the inertial yaw angle in degrees */
    inline float InertialYaw( void ) const { return m_InertialYaw_deg; }

    /** Returns the inertial roll rate in degrees per second */
    inline float InertialRollRate( void ) const
    { return m_InertialRollRate_dps; }

    /** Returns the inertial pitch rate in degrees per second */
    inline float InertialPitchRate( void ) const
    { return m_InertialPitchRate_dps; }

    /** Returns the inertial yaw rate in degrees per second */
    inline float InertialYawRate( void ) const
    { return m_InertialYawRate_dps; }


    /** @enum e_SelfTestFlagBits
     * Bit positions of IMU3 Self-test flags returned by
     * GetSelfTestErrorFlags()
     */
    enum e_SelfTestFlagBits
    {
        STFLAG_GyroX1 = 0,  /**< Set on self-test failure of Gyro X1 */
        STFLAG_GyroY1,  /**< Set on self-test failure of Gyro Y1 */
        STFLAG_GyroZ1,  /**< Set on self-test failure of Gyro Z1 */
        STFLAG_AccX1,   /**< Set on self-test failure of Accelerometer X1 */
        STFLAG_AccY1,   /**< Set on self-test failure of Accelerometer Y1 */
        STFLAG_AccZ1,   /**< Set on self-test failure of Accelerometer Z1 */
        STFLAG_AccX2,   /**< Set on self-test failure of Accelerometer X2 */
        STFLAG_AccY2,   /**< Set on self-test failure of Accelerometer Y2 */
        STFLAG_AccZ2,   /**< Set on self-test failure of Accelerometer Z2 */
        STFLAG_TempX,   /**< TempX Status */
        STFLAG_TempY,   /**< TempY Status */
        STFLAG_TempZ,   /**< TempZ Status */
    };


    int m_Verbosity; /**< Verbosity of debugging messages (greater = more) */


private:

	//-------------------------------
	// Variables used for parsing
	// received data from the IMU
	//-------------------------------
	uint8_t m_MessageBuffer[IMU3_PACKET_SIZE];
	int m_NumMessageBytesReceived;
	bool m_RxIsSynchronized;    /**< true after synchronization to signature
	                                 Byte sequences in received data has been
	                                 established */


	bool m_ImuIsConnected;		/**< true if IMU3 messages are being received */


	bool m_ImuDataIsFresh;		/**< true when an IMU message has been received
									 since the last time ImuDataIsFresh() was
									 called */

	bool m_AhrsDataIsFresh;		/**< true when an IMU message has been received
									 since the last time ImuDataIsFresh() was
									 called */

	//-------------------------
	// IMU message data
	//-------------------------
	uint8_t m_ImuStatusBytes[3];/**< IMU status Bytes */
	int m_ImuSelfTestFlags;     /**< Self-test flags reported in IMU message */
	bool m_ImuIsAligning;		/**< true if the IMU is performing its initial
	                                 45-second self-alignment routine */

	float m_DeltaRoll_deg;		/**< Delta body roll angle (degrees) */
	float m_DeltaPitch_deg;		/**< Delta body pitch angle (degrees) */
	float m_DeltaYaw_deg;		/**< Delta body yaw angle (degrees) */

	float m_dVLongitudinal;	    /**< Delta body longitudinal velocity
									 (meters per second) */
	float m_dVLateral;		    /**< Delta body lateral velocity
									 (meters per second) */
	float m_dVNormal;		    /**< Delta body normal velocity
									 (meters per second) */


	//-------------------------
	// AHRS message data
	//-------------------------
	float m_InertialRoll_deg;		/**< Inertial roll angle (degrees) */
	float m_InertialPitch_deg;		/**< Inertial pitch angle (degrees) */
	float m_InertialYaw_deg;		/**< Inertial yaw angle (degrees) */

	float m_InertialRollRate_dps;	/**< Inertial roll rate (degrees/sec) */
	float m_InertialPitchRate_dps;	/**< Inertial pitch rate (degrees/sec) */
	float m_InertialYawRate_dps;	/**< Inertial yaw rate (degrees/sec) */


	//-----------------------------------------
	// Filters used to reduce IMU sample rate
	// from 100-Hz to a more manageable 10-Hz
	//-----------------------------------------
	/**< Decimating low-pass filters applied to incoming IMU and AHRS
         measurements to reduce the effective sampling rate */
	YellowSubUtils::PolyphaseFIRDecimator<float>* m_pLPF[12];

	/** @enum e_LPF_IDs
	 * @brief
	 *  ID's used to index into m_pLPF
	 */
	enum e_LPF_IDs
	{
	    /* IMU message measurements */
	    LPF_DeltaBodyRollAngle = 0,
	    LPF_DeltaBodyPitchAngle,
	    LPF_DeltaBodyYawAngle,
	    LPF_DeltaBodyLongitudinalVelocity,
	    LPF_DeltaBodyLateralVelocity,
	    LPF_DeltaBodyNormalVelocity,

	    /* AHRS message measurements */
	    LPF_InertialRollAngle,
	    LPF_InertialPitchAngle,
	    LPF_InertialYawAngle,
	    LPF_InertialRollRate,
	    LPF_InertialPitchRate,
	    LPF_InertialYawRate,

	    NumLPF_IDs  /* Number of filter IDs */
	};

	static const float sm_LPFCoefficients[]; /**< Coefficients of the FIR
	                                              low-pass filter applied
	                                              during IMU sample rate
	                                              reduction */

    static const int sm_NumLPFCoefficients;    /**< The number of elements in
                                                    sm_LPFCoefficients */



    //=========================================================================
    /** Handles a received IMU message from the Archangel IM3 module
     *
     * @param pMessageBytes
     *	A pointer to the 70-Byte IMU message
     */
    void HandleIMUMessage(const uint8_t* pMessageBytes);


    //=========================================================================
    /** Handles a received AHRS message from the Archangel IM3 module
     *
     * @param pMessageBytes
     *	A pointer to the 70-Byte AHRS message
     */
    void HandleAHRSMessage(const uint8_t* pMessageBytes);


	// Prevent automatic generation of copy constructor and
    // assignment operator
    IMU3Module(const IMU3Module&);
    const IMU3Module& operator= (const IMU3Module&);
};

#endif /* IMU3MODULE_H_ */

