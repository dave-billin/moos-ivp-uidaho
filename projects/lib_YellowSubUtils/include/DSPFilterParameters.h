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
/** @file DSPFilterParameters.h
 *
 * @brief
 *	Declaration of a class used to specify filter parameters for DSPFilter
 *	objects
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#ifndef DSPFILTERPARAMETERS_H_
#define DSPFILTERPARAMETERS_H_

#include <vector>


namespace YellowSubUtils
{

//=============================================================================
//=============================================================================
/** An object used with DSPFilter classes to specify filter coefficients and
 *  gains */
//=============================================================================
//=============================================================================
template <class DataType>
class DSPFilterParameters
{
public:
    std::vector<DataType> Numerator; /**< Numerator (feed-forward) z-plane
                                           filter coefficients */
    std::vector<DataType> Denominator;   /**< Denominator (feedback) z-plane
                                               filter coefficients */

    DataType PreGain;   /**< Gain to be applied at the filter's input */
    DataType PostGain;  /**< Gain to be applied at the filter's output */

    bool ProcessingIsEnabled;   /**< true if the filter should process input;
                                     else false if input samples should be
                                     passed through the filter unmodified */


    //=========================================================================
    /** Default constructor: defaults to empty numerator and denominator,
     *  Pre and Post gains set to 1.0, and processing enabled
     */
    DSPFilterParameters( void )
    {
        DataType One = static_cast<DataType>(1);
        PreGain = PostGain = One;
        ProcessingIsEnabled = true;
    }



    //=========================================================================
    /** Parameterized constructor: if numerator or denominator coefficients are
     *  left unspecified, a unity coefficient will be added automatically
     *
     * @param EnableProcessing
     *  true if the filter should process incoming data; else false if the
     *  filter should pass data through unmodified
     *
     * @param _PreGain
     *  Gain to apply at the input of the filter
     *
     * @param _PostGain
     *  Gain to apply at the filter output
     *
     * @param pNumeratorCoefficients
     *  Pointer to an array of coefficients from the numerator of the filter's
     *  z-domain transfer function with element[0] = b0 and element[N] = bN
     *
     * @param NumNumeratorCoefficients
     *  The number of numerator coefficients pointed to by
     *  pNumeratorCoefficients
     *
     * @param pDenominatorCoefficients
     *  Pointer to an array of coefficients from the denominator of the
     *  filter's z-domain transfer function with element[0] = b0 and
     *  element[N] = bN
     *
     * @param NumDenominatorCoefficients
     *  The number of numerator coefficients pointed to by
     *  pNumeratorCoefficients
     */
    DSPFilterParameters( const bool EnableProcessing,
                         const DataType _PreGain = 1.0,
                         const DataType _PostGain = 1.0,
                         const DataType* pNumeratorCoefficients = NULL,
                         const int NumNumeratorCoefficients = 0,
                         const DataType* pDenominatorCoefficients = NULL,
                         const int NumDenominatorCoefficients = 0 )
    {
        ProcessingIsEnabled = EnableProcessing;
        PreGain = _PreGain;
        PostGain = _PostGain;

        //-------------------------------------
        // Copy in numerator coefficients
        //-------------------------------------
        if ( (NumNumeratorCoefficients > 0) &&
             (pNumeratorCoefficients != NULL) )
        {
            for (int i = 0; i < NumNumeratorCoefficients; i++)
            {
                Numerator.push_back(pNumeratorCoefficients[i]);
            }
        }
        else
        {
            // If numerator is empty, add a unity coefficient
            Numerator.push_back(static_cast<DataType>(1));
        }


        //-------------------------------------
        // Copy in denominator coefficients
        //-------------------------------------
        if ( (NumDenominatorCoefficients > 0) &&
             (pDenominatorCoefficients != NULL) )
        {
            for (int i = 0; i < NumDenominatorCoefficients; i++)
            {
                Denominator.push_back(pDenominatorCoefficients[i]);
            }
        }
    }
};

}   // END namespace YellowSubUtils

#endif /* DSPFILTERPARAMETERS_H_ */
