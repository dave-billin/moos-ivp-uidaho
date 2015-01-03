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
/** @file DSPFilter.h
 * @brief
 *	Declaration of the DSPFilter base class for digital filter objects
 *
 * @author Dave Billin
 *
 * @note
 *	Adapted from filter implementations provided in the book "Algorithms for
 *	Digital Signal Processing" by Paul M. Embree and Damon Danieli
 */
//=============================================================================
#ifndef DSPFILTER_H
#define DSPFILTER_H

#include <vector>
#include <assert.h>
#include <cmath>

#include "DSPFilterParameters.h"


namespace YellowSubUtils
{


//=============================================================================
//=============================================================================
/** Base class that defines an interface shared by inherited filter objects */
//=============================================================================
//=============================================================================
template <class DataType>
class DSPFilter
{
protected:
    bool m_IsInitialized;   /**< true if the filter has been initialized with
                                 coefficients and a history buffer exists */

    bool m_ProcessingIsEnabled; /**< true if the filter should process input
                                     samples; else false to pass input through
                                     the filter unaltered */

    DataType m_PreGain;     /**< Gain applied to filter input samples */
    DataType m_PostGain;    /**< Gain applied to filter output samples */

    DataType m_Output;         /**< Current output of the filter */

public:

	//=========================================================================
	/** Default base class constructor */
	DSPFilter( void )
	  : m_IsInitialized(false),
	    m_ProcessingIsEnabled(true),
	    m_PreGain(static_cast<DataType>(0)),
	    m_PostGain(static_cast<DataType>(0)),
	    m_Output(static_cast<DataType>(0))
	{ ; }


	//=========================================================================
	/** Creates the filter object and initializes it with a set of specified
	 *  coefficients
	 *
	 * @param Params
	 *  Reference to a DSPFilterParameters object containing parameters to
	 *  initialize the filter with
	 */
	DSPFilter( DSPFilterParameters<DataType>& Params )
	{
	    SetParameters(Params);
	}


	//=========================================================================
	/** Called when a filter object goes out of scope */
	virtual ~DSPFilter()
	{
	}



	///////////////////////////////////////////////////////////////////////
	// Virtual functions implemented in derived classes
	///////////////////////////////////////////////////////////////////////

	//=========================================================================
	/** Applies a single sample to the filter
	 *
	 * @param InputSample
	 * 	Input sample to apply to the filter
	 *
	 * @returns
	 * 	The corresponding output sample from the filter
	 */

	virtual DataType ProcessSample( DataType InputSample ) = 0;



	//=========================================================================
	/** Applies the filter to a buffer of sample data
	 *
	 * @param SourceBuffer
	 * 	Pointer to a buffer of sample data to process
	 *
	 * @param DestBuffer
	 * 	Pointer to a buffer where processed samples should be placed
	 *
	 * @param NumSamples
	 * 	Number of elements in SourceBuffer and DestBuffer (it is left to the
	 * 	caller to ensure these buffers have an equal number of elements)
	 *
	 * @returns
	 * 	A pointer to the buffer containing filtered sample data
	 *
	 * @note
	 *	This default implementation uses the ProcessSample() function that
	 *	derived classes must implement.
	 */
	virtual DataType* ProcessSamples( const DataType* SourceBuffer,
								      DataType* DestBuffer,
								      const int NumSamples )
	{
		//----------------------------------
		// Validate arguments.
		//----------------------------------
		assert( (SourceBuffer != NULL) && (DestBuffer != NULL) );

		// The filter must be initialized before it can be used
		assert( m_IsInitialized );

		//----------------------------------
		// Process the sample data
		//----------------------------------
		DataType* pIn = const_cast<DataType*>(SourceBuffer);
		DataType* pOut = DestBuffer;
		int i = NumSamples;
		do
		{
			*pOut++ = ProcessSample( *pIn++ );
		}
		while (--i > 0);

		return DestBuffer;
	}



	//=========================================================================
    /** Returns the filter's output value */
    DataType Output( void ) const
    {
        return m_Output;
    }


	//=========================================================================
	/** Clears the filter's sample history and sets its output to zero */
	virtual void Reset() = 0;



	//=========================================================================
	/** Enables or disables filter processing
	 *
	 * @param ShouldBeEnabled
	 *  true to enable filtering; else false if input samples should be passed
	 *  through the filter without modification
	 */
	virtual void Enable( bool ShouldBeEnabled )
	{ m_ProcessingIsEnabled = ShouldBeEnabled; }

	/** Returns true if filter processing is enabled */
	bool IsEnabled( void ) const { return m_ProcessingIsEnabled; }




	//=========================================================================
	/** Sets the filter's coefficients
	 * @param Params
	 * 	A DSPFilterParameters object containing filter coefficients and gains
	 *
	 * @return
	 *  true if the specified parameters were applied to the filter
	 *  successfully; else false to signal an invalid parameter
	 */
	virtual bool SetParameters(DSPFilterParameters<DataType>& Params) = 0;

	//=========================================================================
	/** Returns a DSPFilterParameters object containing the filter's
	 *  coefficients
	 * @param Coeffs
	 * 	DSPFilterParameters object to be populated with filter coefficients
	 */
	virtual DSPFilterParameters<DataType> GetParameters( void ) const = 0;



    //=========================================================================
    /** Returns the sum of the filter's coefficients (typically for validating
     *  the filter's step response) */
    virtual DataType SumOfCoefficients( void ) const = 0;

};


}; // END namespace YellowSubUtils



#endif // END #ifndef DSPFILTER_H
