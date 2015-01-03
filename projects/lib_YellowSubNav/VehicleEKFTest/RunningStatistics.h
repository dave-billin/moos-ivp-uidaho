//=============================================================================
/** @file RunningStatistics.h
 *
 * @brief
 * 	Declaration of the RunningStatistics class
 */
//=============================================================================
#ifndef _RUNNINGSTATISTICS_H_
#define _RUNNINGSTATISTICS_H_

#include <stdint.h>
#include <math.h>


template <class T>
/** A templated class useful for gathering statistics on a variable's value */
class RunningStatistics
{
public:
	/** Creates an instance of the object */
	RunningStatistics( void )
	{
		Initialize();
	}

	/** Initializes the statistics object to a zero starting point */
	void Initialize( void )
	{
		m_NumSamples = 0;
		m_MostPositive = 0.0;
		m_MostNegative = 0.0;
	}

	/** Applies a new sample value to the statistics object */
	void Update(T& NewSample)
	{
		m_NumSamples++;		// Increment sample count
		m_Sum += NewSample;	// Accumulate

		// Gather peaks
		if ((NewSample > 0) && (NewSample > m_MostPositive) )
		{
			m_MostPositive = NewSample;
		}

		if ((NewSample < 0) && (NewSample < m_MostNegative) )
		{
			m_MostNegative = NewSample;
		}

		//------------------------------------------------------------------
		// Calculate running variance using method originally proposed by
		// B. P. Welford and presented on page 232 of Donald Knuth's "Art
		// of Computer Programming" Vol 2, 3rd edition.
		//------------------------------------------------------------------
		if (m_NumSamples == 1)	// First sample only
		{
			m_LastMean = NewSample;
			m_Mean = NewSample;
			m_S = 0.0;
		}
		else
		{
			m_Mean = m_LastMean + (NewSample - m_LastMean) / m_NumSamples;
			m_S = m_LastS + (NewSample - m_LastMean)*(NewSample - m_Mean);

			// set up for next iteration
			m_LastMean = m_Mean;
			m_LastS = m_S;
		}
	}


	/** Returns the number of samples accumulated by the object */
	double NumSamples(void) const { return m_NumSamples; }

	/** Returns the sample value with the greatest positive value */
	T MostPositive(void) const { return m_MostPositive; }

	/** Returns the sample value with the most negative value */
	T MostNegative(void) const { return m_MostNegative; }

	/** Returns the mean value of all samples */
	double Mean(void) const
	{
		return (m_NumSamples > 0) ? m_Mean : 0.0;
	}

	/** Returns the variance of all samples */
	double Variance(void) const
	{
		return ( (m_NumSamples > 1) ? m_S / (m_NumSamples - 1) : 0.0 );
	}

	/** Returns the standard deviation of all samples */
	double StdDev(void) const
	{
		return sqrt( Variance() );
	}


private:
	uint64_t m_NumSamples;	/**< Total number of samples gathered so far */
	double m_Sum;			/**< Sum of all samples */
	T m_MostPositive;		/**< Most-positive value */
	T m_MostNegative;		/**< Most-negative value */
	double m_Mean;			/**< Mean value */
	double m_LastMean;		/**< Previous mean used for running variance */
	double m_S;				/**< Used to calculate running variance */
	double m_LastS;			/**< Used to calculate running variance */
};



#endif 	// END #ifndef _RUNNINGSTATISTICS_H_
