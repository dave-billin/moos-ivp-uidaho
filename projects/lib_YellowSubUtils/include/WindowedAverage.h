//=============================================================================
/** @file WindowedAverage.h
 *
 * @brief
 *	Declaration of a templated class that calculates a moving average over a
 *	window of samples
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef AVERAGINGFIRFILTER_H_
#define AVERAGINGFIRFILTER_H_

#include "YellowSubUtils.h"
#include "FIRFilter.h"

namespace YellowSubUtils
{

//=============================================================================
/** @class WindowedAverage
 * @brief
 *   An object that computes the average value of its input over a fixed-size
 *   window of consecutive samples
 */
//=========================================================================
template <class DataType>
class WindowedAverage: public YellowSubUtils::FIRFilter<DataType>
{
public:

    //=========================================================================
    /** Default constructor: creates a single-sample window (eminently useless,
     *  for averaging; included for numerical consistency)
     */
    WindowedAverage( void )
    {
        DSPFilterParameters<DataType> params = MakeParameters( 1 );
        FIRFilter<DataType>::SetParameters( params );
    }


    //=========================================================================
    /** Creates an instance of the averaging filter with a specified number
     *  of taps
     *
     * @param [in] NumSamplesToAverage
     *   Number of samples in the window the filter computes an average from
     *
     * @param [in] PreGain
     *   Gain applied to input samples and filter coefficients (typically for
     *   scaling purposes)
     */
    WindowedAverage( uint32_t NumSamplesToAverage,
                     DataType PreGain = 1,
                     DataType PostGain = 1 )
    {
        DSPFilterParameters<DataType> params =
                                    MakeParameters( NumSamplesToAverage,
                                                    PreGain, PostGain );
        FIRFilter<DataType>::SetParameters( params );

    }


    //=========================================================================
    /** Copy constructor */
    WindowedAverage( WindowedAverage const& other )
      : FIRFilter<DataType>( other )
    {}

    /** Called when the object goes out of scope */
    virtual ~WindowedAverage()
    {
        // Coefficient de-allocation is handled by FIRFilter destructor
    }



    //=========================================================================
    /** Assignment operator overload - copies another averaging filter object
     *  and zeros the filter history
     *
     * @param Rhs
     *  Object to copy from
     */
    const WindowedAverage& operator= (const WindowedAverage& Rhs)
    {
        FIRFilter<DataType>::operator=( Rhs );
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
        if ( DSPFilter<DataType>::m_ProcessingIsEnabled )
        {
            // Apply pre-filter gain
            InputSample *= DSPFilter<DataType>::m_PreGain;
        }

        // Rotate the input sample into the input history
        DataType* px_n = FIRFilter<DataType>::m_InputHistory
                            + FIRFilter<DataType>::m_NumTaps - 2;
        DataType* px_nMinus1 = px_n + 1;
        int i = FIRFilter<DataType>::m_NumTaps;
        do
        {
            *px_nMinus1-- = *px_n--;
        }
        while (--i > 0);
        FIRFilter<DataType>::m_InputHistory[0] = InputSample;


        // Compute filter output
        if ( DSPFilter<DataType>::m_ProcessingIsEnabled )
        {
            // Compute sum of sample history
            DataType& Output = DSPFilter<DataType>::m_Output;
            Output = static_cast<DataType>(0);
            DataType* pTap = FIRFilter<DataType>::m_InputHistory;
            int i = FIRFilter<DataType>::m_NumTaps;
            do
            {
                Output += *pTap++;
            }
            while (--i != 0);

            // Compute average of sample history
            Output /= FIRFilter<DataType>::m_NumTaps;

            // Apply post-filter gain
            Output *= DSPFilter<DataType>::m_PostGain;
        }
        else
        {
            // If processing is disabled, return the oldest sample in the
            // input history to preserve phase between enabled and disabled
            // filters
            DSPFilter<DataType>::m_Output =
                FIRFilter<DataType>::m_InputHistory[
                                         FIRFilter<DataType>::m_NumTaps - 1];
        }

        return DSPFilter<DataType>::m_Output;
    }


    //=========================================================================
    /** Builds DSPFilterParameters used to create a WindowedAverage object
     *
     * @param [in] NumSamplesToAverage
     *   Size of the window of samples to be averaged
     *
     * @param [in] PreGain
     *   Gain applied to input samples prior to processing
     *
     * @param [in] PostGain
     *   Gain applied to the output of the filter
     */
    static DSPFilterParameters<DataType> MakeParameters(
                                               uint32_t NumSamplesToAverage,
                                               DataType PreGain = 1,
                                               DataType PostGain = 1 )
    {
        // Ensure the filter has at least one tap
        if ( NumSamplesToAverage == 0 )
        {
            NumSamplesToAverage = 1;
        }

        DSPFilterParameters<DataType> params;
        DataType c = static_cast<DataType>(1)
                            / static_cast<DataType>(NumSamplesToAverage);

        params.Numerator.resize( NumSamplesToAverage );
        for (
                typename std::vector<DataType>::iterator coeff =
                                                params.Numerator.begin();
                coeff != params.Numerator.end(); coeff++ )
        {
            *coeff = c;
        }

        params.PreGain = PreGain;
        params.PostGain = PostGain;
        return params;
    }
};

} /* namespace YellowSubUtils */
#endif /* AVERAGINGFIRFILTER_H_ */
