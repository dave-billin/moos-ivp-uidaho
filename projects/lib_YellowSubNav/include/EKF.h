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
/** @file EKF.h
 *
 * @brief
 * 	Declaration of an abstract base class for an AUV Extended Kalman Filter
 * 	(EKF)
 */
//=============================================================================
#ifndef _EKF_H_
#define _EKF_H_

#include <stdint.h>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h" 
#include "newmat.h"	// newmat matrix library


#include "LblPositionEstimator.h"


namespace YellowSubNav
{


//=============================================================================
/** Base class for a measurements supplied to the UpdateMeasurement method of
 * an object derived from EKF
 */
class EKFMeasurement
{
public:

    /** Creates a measurement object with a specified type ID */
    EKFMeasurement( int TypeId ) { m_TypeId = TypeId; }

    virtual ~EKFMeasurement() {}    // Must make base class polymorphic

    /** Returns the object's measurement type */
    int MeasurementType( void ) const { return m_TypeId; }

private:
    int m_TypeId;   /**< Identifies the kind of measurement data the object is
                         carrying.  Each derived classes should populate this
                         field with a unique value */
};





//=============================================================================
/** Abstract base class supplying the interface for Extended Kalman Filter
 *	objects used in the U of I AUV
 */
class EKF
{
public:

	//=========================================================================
	/** Creates an instance of an EKF object.
	 *
	 * @param NumStates
	 *	Desired number of states in the EKF.
	 *
	 * @param NumMeasurements
	 * 	The maximum number of measurements used by the EKF
	 *
	 * @param NumNoiseSources
	 * 	The number of noise sources used by the EKF model
	 */
	EKF( uint32_t NumStates, uint32_t NumNoiseSources );


	//=========================================================================
	/** Called when the object goes out of scope */
	virtual	~EKF();


	//=========================================================================
	/** Called to initialize the EKF with seed values.  This function calls the
	 * DoInitializeEKF() method supplied by a derived class.
	 *
	 * @param pInitialStateValues
	 *	Pointer to an array of (floating-point) initial values of EKF states.
	 *	It is left to the caller to ensure that the array contains at least as
	 *	many values as the EKF object has states.
	 *
	 * @param t
	 * 	The current time in seconds
	 *
	 * @throw
	 *	A CMOOSException object if pInitialStateValues is NULL
	 */
	void Initialize( NEWMAT::Real* pInitialStateValues, double t );



	//=========================================================================
	/** Called to apply a measurement to the EKF.  This function calls the
	 * DoUpdateMeasurement() method supplied by a derived class
	 *
	 * @param MeasurementData
	 *	An object specific to the derived class EKF implementation containing
	 *	the measurement data to apply.
	 */
	void UpdateMeasurement( const EKFMeasurement& MeasurementData );



	//=========================================================================
	/** Called at a regular interval to propagate the EKF.  The rate at which
	 * this method is invoked must meet or exceed the maximum rate of any EKF
	 * sensor input.
	 *
	 * @param Delta_t
	 * 	Current time in seconds.  Used to calculate the elapsed time since the
	 * 	last EKF propagation
	 */
	void Propagate( double t );



	//=========================================================================
	/** Returns the current EKF (estimated) states in a column vector */
	const NEWMAT::ColumnVector& GetStates( void ) const { return X; }


	//=========================================================================
	inline const NEWMAT::Matrix& GetP( void ) const { return P; }
	inline const NEWMAT::Matrix& GetQ( void ) const { return Q; }
	inline const NEWMAT::Matrix& GetR( void ) const { return R; }
	inline const NEWMAT::Matrix& GetK( void ) const { return K; }
	inline const NEWMAT::Matrix& GetH( void ) const { return H; }
	inline const NEWMAT::Matrix& GetM( void ) const { return M; }


	//=========================================================================
	/** Returns true if a matrix inversion failed during a previous call to the
	 * EKF's UpdateEquations() method. */
	inline bool InvertFailed( void ) const { return m_InvertFailed; }


	//=========================================================================
	/** Resets the m_InvertFailed flag to false */
	inline void ResetInvertFlag(void) { m_InvertFailed = false; }

	//=========================================================================
	/** Returns true if the EKF has been initialized */
	inline bool IsInitialized(void) const { return m_IsInitialized; }


protected:

	//=========================================================================
	/** Implementation-specific EKF initialization supplied by a derived
	 *  class.
	 *
	 * @param pInitialStateValues
	 *	Pointer to an array of (floating-point) initial values of EKF states.
	 *	It is left to the caller to ensure that the array contains at least as
	 *	many values as the EKF object has states.
	 */
	virtual void DoInitializeEKF( NEWMAT::Real* pInitialStateValues ) = 0;


	//=========================================================================
	/** Implementation-specific EKF measurement update supplied by a derived
	 *  class.  This gets called from within ApplyMeasurement() just prior to
	 *  an EKF measurement update
	 *
	 * @param MeasurementData
	 *	An object containing the measurement data to apply.
	 *
	 * @param [out] OUT_ShouldUpdateEKF
	 *	If the data in MeasurementData is invalid or could not be applied,
	 *	this function should set this value to false.  Otherwise, if set to
	 *	true or left unmodified, an EKF measurement update will be carried out
	 *	as soon as this function returns.
	 */
	virtual void PreUpdateMeasurement( const EKFMeasurement& Measurement,
									   bool& OUT_ShouldUpdateEKF ) = 0;


	//=========================================================================
	/** This function gets called just after an EKF measurement update. It may
	 *	be used by a derived class to handle implementation-specific details.
	 *
	 * @param Measurement
	 * 	Measurement object that triggered the EKF update.
	 */
	virtual void PostUpdateMeasurement( const EKFMeasurement& Measurement ) = 0;


	//=========================================================================
	/** Implementation-specific method supplied by a derived class that gets
	 * 	called just prior to propagation of the EKF.
	 *
	 * @details
	 *	Typically, the derived class will use this function to set up the F, X,
	 *	and other EKF matrices in a manner specific to its EKF implementation.
	 *	After this function returns, the EKF will propagate.
	 *
	 * @param Delta_t
	 * 	Time (in seconds) since the last EKF propagation
	 */
	virtual void PrePropagate( double Delta_t ) = 0;


	//=========================================================================
	/** This function gets called just after the EKF is propagated.  It may
	 *	be used by a derived class to handle implementation-specific details.
	 *
	 * @param Delta_t
	 * 	Time (in seconds) preceding the EKF propagation just performed
	 */
	virtual void PostPropagate( double Delta_t ) = 0;


	//=========================================================================
	/** Calling this function will cause the EKF object to behave as if it is
	 * not initialized, effectively disabling it.  This can be called by
	 * inherited class functions to handle critical errors.  Use with caution.
	 */
	void DeInitialize( void ) { m_IsInitialized = false; }


    //----------------------------------------------------------------------
	/** @name EKF Matrices
	 * @brief
	 *	These matrices are automatically created (and sized) within the EKF
	 *	class constructor.  They may be used directly within derived class
	 *	methods.
	 */
    //----------------------------------------------------------------------
    //@{
	NEWMAT::ColumnVector X;	/**< EKF state (X) vector */
	NEWMAT::Matrix P;		/**< EKF error covariance matrix */
	NEWMAT::Matrix Q;		/**< EKF Q process noise standard deviation */
	NEWMAT::Matrix R;		/**< EKF noise covariance matrix */

	// The following matrices are used for sensor updates
	NEWMAT::Matrix H;		/**< Observation matrix */
	NEWMAT::Matrix M;		/**< Measurement Noise Map */
	NEWMAT::ColumnVector Residual;	/**< Measurement residual */

	NEWMAT::Matrix F;		/**< EKF state transition matrix */

	NEWMAT::Matrix A;		/**< Scratch matrix used for measurement updates */
	NEWMAT::Matrix K;		/**< Kalman Gain */

	const NEWMAT::IdentityMatrix m_I;	/**< Identity matrix used for calculations */
	//@}

private:


	bool m_IsInitialized;	/**< true after the object's Initialize() method
	 	 	 	 	 	 	 	 has been called; otherwise false */

	bool m_InvertFailed;	/**< true if a matrix inversion fails in a call to
								 the EKF's UpdateEquations() method (e.g. when
								 a new measurement is applied; else false.  Can
								 be reset to false by calling ResetInvertFlag()
								 */

	double m_LastPropagateTime;	/**< Time (seconds) when the EKF was last
									 propagated.  Initially set in the call to
									 Initialize() */

	//=========================================================================
	// Prevent automatic generation of copy constructor and assignment operator
	EKF (const EKF&);
    const EKF& operator= (const EKF&);


};




}	// END namespace YellowSubNav

#endif /* EKF_H_ */
