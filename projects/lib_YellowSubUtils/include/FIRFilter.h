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
/** @file FIRFilter.h
 *
 * @brief
 *  Declaration of a templated class implementing a finite impulse response
 *  (FIR) filter
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#ifndef FIRFILTER_H_
#define FIRFILTER_H_

#include <cassert>
#include <cstring>
#include "DSPFilter.h"


namespace YellowSubUtils
{


//=============================================================================
//=============================================================================
/** A class that implements a finite impulse response filter
 * @details
 *  To use this class, simply create a FIRFilter object, initializing it with
 *  a DSPFilterParameters object.  Following this, you can call the object's
 *  ProcessSample() method to process a single sample or ProcessSamples() to
 *  process a block of samples.
 */
//=============================================================================
//=============================================================================
template <class DataType>
class FIRFilter : public DSPFilter<DataType>
{
protected:
    int m_NumTaps;    /**< The number of filter coefficients and elements
                                 in the history buffer */

    DataType* m_TapCoefficients;    /**< Array of filter tap coefficients */

    DataType* m_InputHistory;   /**< Delay line of input sample history with
                                     m_InputHistory[i] = x[n-i] */

    //=========================================================================
    /** A helper function to release coefficients and history buffers */
    void Clear( void )
    {
        if (m_TapCoefficients != NULL)
        {
            delete[] m_TapCoefficients;
            m_TapCoefficients = NULL;
        }

        if (m_InputHistory != NULL )
        {
            delete[] m_InputHistory;
            m_TapCoefficients = NULL;
        }

        m_NumTaps = 0;
        DSPFilter<DataType>::m_Output = static_cast<DataType>(0);
        DSPFilter<DataType>::m_IsInitialized = false;
    }




public:

    //=========================================================================
    /** Creates an uninitialized instance of the object */
    FIRFilter()
    : m_NumTaps(0),
      m_TapCoefficients(NULL),
      m_InputHistory(NULL)
    {
        DSPFilter<DataType>::m_Output = static_cast<DataType>(0);
    }


    //=========================================================================
    /** Creates an instance of the object and initializes it with a specified
     *  set of filter parameters
     *
     * @param Param
     *  Filter coefficients and gains used to initialize the object
     */
    FIRFilter( DSPFilterParameters<DataType>& Params )
    : m_NumTaps(0),
      m_TapCoefficients(NULL),
      m_InputHistory(NULL)
    {
        SetParameters(Params);
    }


    //=========================================================================
    /** Copy constructor
     *
     * @param SrcObj
     *  Object to copy from
     */
    FIRFilter(const FIRFilter<DataType>& SrcObj)
    : m_NumTaps(0),
      m_TapCoefficients(NULL),
      m_InputHistory(NULL)
    {
        // Copy source object parameters and create an initialized
        // input history buffer
        *this = SrcObj;
    }


    //=========================================================================
    /** Called when the object goes out of scope */
    virtual ~FIRFilter()
    {
        Clear();    // Release coefficients and history buffer
    }



    //=========================================================================
    /** Sets the filter's sample history and output to zero */
    virtual void Reset()
    {
        DataType Zero = static_cast<DataType>(0);
        for (int i = 0; i < m_NumTaps; i++)
        {
            m_InputHistory[i] = Zero;
        }

        DSPFilter<DataType>::m_Output = Zero;
    }


    //=========================================================================
    /** Returns the sum of the filter's coefficients (typically for validating
     *  the filter's step response) */
    virtual DataType SumOfCoefficients( void ) const
    {
        DataType Accum = static_cast<DataType>(0);

        if (m_NumTaps > 0)
        {
            for (int i = 0; i < m_NumTaps; i++)
            {
                Accum += m_TapCoefficients[i];
            }
        }

        return Accum;
    }


    //=========================================================================
    /** Sets the filter's coefficients
     * @param Params
     *  A DSPFilterParameters object containing filter coefficients and gains
     *
     * @return
     *  true if the specified parameters were applied to the filter
     *  successfully; else false to signal an invalid parameter
     */
    virtual bool SetParameters(DSPFilterParameters<DataType>& Params)
    {
        Clear();    // Release existing coefficients and history

        // NOTE: for a FIR filter, we ignore the Denominator coefficients...
        int NumTaps = Params.Numerator.size();
        if (NumTaps > 0)
        {
            m_NumTaps = NumTaps;
            m_TapCoefficients = new DataType[NumTaps];
            m_InputHistory = new DataType[NumTaps];
            DSPFilter<DataType>::m_ProcessingIsEnabled =
                                                Params.ProcessingIsEnabled;

            // Copy in tap coefficients
            for (int i = 0; i < NumTaps; i++)
            {
                m_TapCoefficients[i] = Params.Numerator[i];
            }

            DSPFilter<DataType>::m_PreGain = Params.PreGain;
            DSPFilter<DataType>::m_PostGain = Params.PostGain;

            // Reset filter history and output
            Reset();

            // Mark filter as ready
            DSPFilter<DataType>::m_IsInitialized = true;
        }

        return DSPFilter<DataType>::m_IsInitialized;
    }



    //=========================================================================
    /** Returns a DSPFilterParameters object containing filter coefficients and
     *  gains
     *
     * @param Coeffs
     *  DSPFilterParameters object to be populated with filter coefficients
     */
    virtual DSPFilterParameters<DataType> GetParameters( void ) const
    {
        DSPFilterParameters<DataType> Params;
        Params.PreGain = DSPFilter<DataType>::m_PreGain;
        Params.PostGain = DSPFilter<DataType>::m_PostGain;
        Params.ProcessingIsEnabled =
                                DSPFilter<DataType>::m_ProcessingIsEnabled;

        if (m_NumTaps > 0)
        {
            Params.Numerator.resize( m_NumTaps );
            DataType* c = m_TapCoefficients;
            typename std::vector<DataType>::iterator coeff =
                                                    Params.Numerator.begin();
            for (int i = 0; i < m_NumTaps; i++)
            {
                *coeff++ = *c++;
            }
        }

        return Params;
    }



    //=========================================================================
    /** Assignment operator overload - copies parameters and creates a new
     *  history buffer whose elements are initialized to zero
     *
     * @param Rhs
     *  Object to copy from
     */
    const FIRFilter& operator= (const FIRFilter& Rhs)
    {
        // Prevent self-assignment
        if ( &Rhs != this )
        {
            DSPFilterParameters<DataType> Params = Rhs.GetParameters();
            this->SetParameters(Params);
        }

        return *this;
    }



    //=========================================================================
    /** Processes a single sample in the filter
     *
     * @param InputSample
     *  Sample to apply to the filter input
     *
     * @returns
     *  Sample value produced at the output of the filter
     */
    virtual DataType ProcessSample( DataType InputSample )
    {
        // The filter needs to be initialized with SetParameters() before it
        // can be used to process data!
        assert( DSPFilter<DataType>::m_IsInitialized );

        if ( DSPFilter<DataType>::m_ProcessingIsEnabled )
        {
            // Apply pre-filter gain
            InputSample *= DSPFilter<DataType>::m_PreGain;
        }

        // Rotate the input sample into the input history
        DataType* px_n = m_InputHistory + m_NumTaps - 2;
        DataType* px_nMinus1 = px_n + 1;
        int i = m_NumTaps;
        do
        {
            *px_nMinus1-- = *px_n--;
        }
        while (--i > 0);
        m_InputHistory[0] = InputSample;


        if ( DSPFilter<DataType>::m_ProcessingIsEnabled )
        {
            // Compute filter output as the sum-of-products of filter tap
            // coefficients and input history.  A do-while loop is used with
            // post-incremented pointer accesses to keep the multiply-and-
            // accumulate loop tight and open to compiler optimization
            DataType& Output = DSPFilter<DataType>::m_Output;
            Output = static_cast<DataType>(0);
            DataType* pTap = m_InputHistory;
            DataType* pTapCoefficient = m_TapCoefficients;
            int i = m_NumTaps;
            do
            {
                Output += *pTap++ * *pTapCoefficient++;
            }
            while (--i != 0);

            // Apply post-filter gain
            Output *= DSPFilter<DataType>::m_PreGain;
        }
        else
        {
            // If processing is disabled, return the oldest sample in the
            // input history.
            // NOTE: this may well result in output discontinuities if
            // enablement is changed while processing data in real-time.
            DSPFilter<DataType>::m_Output = m_InputHistory[m_NumTaps - 1];
        }

        return DSPFilter<DataType>::m_Output;    // Return the filter output
    }


};


}   // END namespace YellowSubUtils

#endif /* FIRFILTER_H_ */
