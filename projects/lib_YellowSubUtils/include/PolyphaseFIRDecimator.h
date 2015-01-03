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
/* @file PolyphaseFIRDecimator.h
 *
 * @brief
 * 	Declaration of a class to implement a decimating polyphse FIR filter
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef POLYPHASEFIRDECIMATOR_H_
#define POLYPHASEFIRDECIMATOR_H_

#include <cassert>
#include <cstring>
#include "DSPFilter.h"
#include "FIRFilter.h"


namespace YellowSubUtils
{

//=============================================================================
//=============================================================================
/** A class that implements a polyphase topology FIR filter with decimation
 *
 * @details
 *  To use this class, create a PolyphaseFIRDecimator object, and pass it a
 *  DSPFilterParameters object containing FIR tap coefficients as if the filter
 *  were being used in direct form without decimation.  The PolyphaseFIRFilter
 *  class automatically partitions filter coefficients, creates sub-filter
 *  phases, and handles decimation of data.  Following creation, you can call
 *  ProcessSample() to process a single sample or ProcessSamples() to process
 *  a block of samples.  After calling one of these functions, the
 *  OutputIsFresh() method can be used to determine whether the filter
 *  has produced a new, decimated output sample.  One output sample will be
 *  produced every M input samples where M is the object's configured
 *  decimation factor.
 */
//=============================================================================
//=============================================================================
template <class DataType>
class PolyphaseFIRDecimator : public DSPFilter<DataType>
{
protected:
    unsigned int m_DecimationFactor;    /**< The filter's decimation factor
                                             (i.e. number of input samples that
                                             must be applied to produce an
                                             output sample */

    int m_NumCoefficients;  /**< Number of filter coefficients (not including
                                 zero-padding) */

    int m_PhaseIndex;       /**< Commutator index of the current phase */

    FIRFilter<DataType>* m_Phase;   /**< Array of sub-filters that implement
                                         the polyphase phases */

    bool m_OutputIsFresh; /**< true when a new decimated sample is
                                     available at the output of the filter;
                                     set to false when OutputIsFresh()
                                     is called */

public:

    //=========================================================================
    /** Creates an uninitialized PolyphaseFIRDecimator object */
    PolyphaseFIRDecimator( void )
     : m_DecimationFactor(1),
       m_NumCoefficients(0),
       m_PhaseIndex(0),
       m_Phase(NULL),
       DSPFilter<DataType>::m_Output(static_cast<DataType>(0)),
       m_OutputIsFresh(false)
       {
       }



	//=========================================================================
	/** Creates an instance of the PolyPhaseDecimator and initializes it with
	 *  a set of coefficients and decimation factor
	 *
     * @param Params
     *  Coefficients and gains that the FIR filter will be initialized with
     *
	 * @param DecimationFactor
	 * 	A positive value specifying the decimation factor of the filter (i.e.
	 * 	the number of input samples that must be applied to the filter input in
	 * 	order to produce a single filter output)
	 */
	PolyphaseFIRDecimator( DSPFilterParameters<DataType>& Params,
	                       unsigned int DecimationFactor = 1)
	 : m_DecimationFactor(DecimationFactor),
       m_NumCoefficients(0),
       m_PhaseIndex(0),
       m_Phase(NULL),
	   m_OutputIsFresh(false)
	{
	    SetParameters(Params);
	}



	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~PolyphaseFIRDecimator()
	{
	    Clear();
	}



	//=========================================================================
	/** Sets the object's decimation factor
	 *
     * @param DecimationFactor
     *  A positive value specifying the decimation factor of the filter (i.e.
     *  the number of input samples that must be applied to the filter input in
     *  order to produce a single filter output).
     *
     * @note
     *  An attempt to set the decimation factor to zero is ignored
     */
	void SetDecimationFactor(unsigned int DecimationFactor)
	{
	    if (DecimationFactor > 0)
	    {
	        // To change the decimation factor, we must re-configure
	        // the polyphase network
	        DSPFilterParameters<float> Params = GetParameters();
	        m_DecimationFactor = DecimationFactor;
	        SetParameters(Params);
	    }
	}

	/** Returns the object's decimation factor */
	unsigned int GetDecimationFactor( void ) const
	{ return m_DecimationFactor; }





	//=========================================================================
	/** Sets the filter's input history and output to zero
	 */
	void Reset( void )
	{
	    for (unsigned int i = 0; i < m_DecimationFactor; i++)
	    {
	        m_Phase[i].Reset();
	    }

	    // Reset phase index to point to the last phase
	    m_PhaseIndex = m_DecimationFactor - 1;

	    DSPFilter<DataType>::m_Output = static_cast<DataType>(0);
	}




	//=========================================================================
	/** Returns true if a new decimated sample is available at the filter's
	 *  output
	 *
	 * @postcondition
	 *	m_NewSampleIsReady is reset set to false
	 */
	bool OutputIsFresh( void )
	{
	    bool b = m_OutputIsFresh;
	    m_OutputIsFresh = false;
	    return b;
	}





    //=========================================================================
    /** Sets the filter's coefficients and gains
     *
     * @param Params
     *  A DSPFilterParameters object containing filter coefficients and gains
     */
    bool SetParameters( DSPFilterParameters<DataType>& Params )
    {
        Clear();    // Release existing sub-filters

        int N = Params.Numerator.size();    // Number of filter taps

        if (N > 0)
        {
            int M = m_DecimationFactor;         // Decimation factor
            int PhaseLength = std::ceil(N/M);   // # taps in each subfilter
            DSPFilter<DataType>::m_PreGain = Params.PreGain;
            DSPFilter<DataType>::m_PostGain = Params.PostGain;
            DSPFilter<DataType>::m_ProcessingIsEnabled =
                                                Params.ProcessingIsEnabled;

            m_NumCoefficients = N;

            // Zero-pad filter coefficients on the RHS
            // to make dimensions polyphase-compatible
            std::vector<DataType> MasterCoefficients(Params.Numerator);
            uint32_t MaxLength = (M * PhaseLength);
            while ( MasterCoefficients.size() < MaxLength )
            {
                MasterCoefficients.push_back(static_cast<DataType>(0));
            }

            // Create uninitialized sub-filter FIR objects
            m_Phase = new FIRFilter<DataType>[M];

            // Partition filter coefficients and configure phase sub-filters
            DSPFilterParameters<DataType> PhaseParams;
            PhaseParams.PreGain = static_cast<DataType>(1);
            PhaseParams.PostGain = static_cast<DataType>(1);
            for (int i = 0; i < M; i++)
            {
                PhaseParams.Numerator.clear();

                for (uint32_t j = i; j < MasterCoefficients.size(); j += M)
                {
                    PhaseParams.Numerator.push_back(MasterCoefficients[j]);
                }

                m_Phase[i].SetParameters(PhaseParams);
            }

            DSPFilter<DataType>::m_IsInitialized = true;
        }

        // Reset the filter
        Reset();
        return DSPFilter<DataType>::m_IsInitialized;
    }


    //=========================================================================
    /** Returns filter coefficients and gains */
    DSPFilterParameters<DataType> GetParameters( void ) const
    {
        DSPFilterParameters<DataType> Params;

        Params.ProcessingIsEnabled =
                        DSPFilter<DataType>::m_ProcessingIsEnabled;

        if (DSPFilter<DataType>::m_IsInitialized)
        {
            int M = m_DecimationFactor;
            int N = m_NumCoefficients;

            Params.Numerator.reserve(N);

            // Extract coefficients from each phase into Params.Numerator
            for (unsigned int i = 0; i < m_DecimationFactor; i++)
            {
                // Copy coefficients from the current phase
                DSPFilterParameters<DataType> PhaseParams =
                                                m_Phase[i].GetParameters();
                typename std::vector<DataType>::iterator iter =
                                            PhaseParams.Numerator.begin();

                for (int j = i; j < N; j += M)
                {
                    Params.Numerator[j] = *iter++;
                }
            }
        }
        else
        {
            Params.PreGain = Params.PostGain = static_cast<DataType>(0);
        }

        return Params;
    }



    //=========================================================================
    /** Returns the sum of the filter's coefficients (typically for validating
     *  the filter's step response) */
    DataType SumOfCoefficients( void ) const
    {
        DataType Accum = static_cast<DataType>(0);

        if (DSPFilter<DataType>::m_IsInitialized)
        {
            // Sum sub-filter coefficients
            for (unsigned int i = 0; i < m_DecimationFactor; i++)
            {
                Accum += m_Phase[i].SumOfCoefficients();
            }
        }

        return Accum;
    }



    //=========================================================================
    /** Processes a single input sample
     *
     * @param InputSample
     *  Sample to be applied to the input of the filter
     *
     * @return
     *  The corresponding sample from the output of the filter
     */
    DataType ProcessSample( const DataType InputSample )
    {
        DataType OutputSample = 0;

        // The filter object must be initialized with coefficients and gains
        // using SetParameters() before it can process sample data!
        assert( DSPFilter<DataType>::m_IsInitialized);

        // If processing is enabled, apply the new sample
        if (DSPFilter<DataType>::m_ProcessingIsEnabled)
        {
            // Apply input gain
            DataType s = InputSample * DSPFilter<DataType>::m_PreGain;

            // Apply the sample to the current phase's sub-filter
            m_Phase[m_PhaseIndex].ProcessSample(s);

            // Decimate!
            if (m_PhaseIndex-- == 0)
            {
                OutputSample = static_cast<DataType>(0);
                for (unsigned int i = 0; i < m_DecimationFactor; i++)
                {
                    OutputSample += m_Phase[i].Output();
                }

                // Apply output gain
                OutputSample *= DSPFilter<DataType>::m_PreGain;

                DSPFilter<DataType>::m_Output = OutputSample;

                m_PhaseIndex = m_DecimationFactor - 1;   // Wrap phase index

                // Flag new sample
                m_OutputIsFresh = true;
            }
        }
        else
        {
            if (m_PhaseIndex-- == 0)
            {
                // If processing is disabled, just pass input samples along
                DSPFilter<DataType>::m_Output = InputSample;

                m_OutputIsFresh = true;
                m_PhaseIndex = m_DecimationFactor - 1;   // Wrap phase index
            }
        }

        return OutputSample;
    }




    //=========================================================================
    /** Applies the filter to a buffer of sample data
     *
     * @param SourceBuffer
     *  Pointer to a buffer of sample data to process
     *
     * @param DestBuffer
     *  Pointer to a buffer to receive K processed samples from the filter
     *  output, where K = floor(NumSamples/DecimationFactor)
     *
     * @param NumSamples
     *  Number of elements in SourceBuffer
     *
     * @returns
     *  A pointer to the buffer containing filtered sample data
     *
     * @note
     *  This default implementation uses the ProcessSample() function that
     *  derived classes must implement.
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
        assert( DSPFilter<DataType>::m_IsInitialized );

        //----------------------------------
        // Process the sample data
        //----------------------------------
        DataType* pIn = const_cast<DataType*>(SourceBuffer);
        DataType* pOut = DestBuffer;
        int i = NumSamples;
        do
        {
            DataType s = ProcessSample( *pIn++ );
            if (m_OutputIsFresh)
            {
                *pOut++ = s;
            }
        }
        while (--i > 0);

        return DestBuffer;
    }



    // Disallow default and copy constructors and assignment operator
    PolyphaseFIRDecimator(const PolyphaseFIRDecimator&);
    const PolyphaseFIRDecimator& operator= (const PolyphaseFIRDecimator&);

protected:

    //=========================================================================
    /** Helper function that releases all sub-filters     */
    void Clear( void )
    {
        if (m_Phase != NULL)
        {
            delete[] m_Phase;
        }

        m_NumCoefficients = 0;
        m_OutputIsFresh = false;
        DSPFilter<DataType>::m_Output = static_cast<DataType>(0);
        DSPFilter<DataType>::m_IsInitialized = false;
    }

};

} /* END namespace YellowSubUtils */

#endif /* POLYPHASEFIRDECIMATOR_H_ */
