//=============================================================================
/*    Copyright (C) 2013  Dave Billin

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
/* @file PrecisionTime.h
 *
 * @brief
 * 	Declaration of a class for working with precision (sub-second) times
 *
 * @author	Dave Billin
 */
//=============================================================================
#ifndef PRECISIONTIME_H_
#define PRECISIONTIME_H_

#include <exception>
#include <string>

#include <stdint.h>
#include <time.h>

namespace YellowSubUtils
{


//=============================================================================
/** @exception PrecisionTimeException
 *
 * @brief
 *  Exception that may be thrown by PrecisionTime methods
 */
//=============================================================================
class PrecisionTimeException : public std::exception
{
public:
    /** Constructor */
    PrecisionTimeException( const char* szWhat )
        : m_What(szWhat) {}

    /** Destructor */
    virtual ~PrecisionTimeException() throw () {}

    const char* what()  { return m_What.c_str(); }
private:
    std::string m_What;
};



//=============================================================================
/** @struct TimeSpec
 *  @brief
 *      A Data structure used internally by PrecisionTimeInterval and
 *      PrecisionTime objects to store nanosecond-precision time values
 */
class TimeSpec
{
    friend class PrecisionTime;
    friend class PrecisionTimeInterval;

public:
    //=========================================================================
    /** Default Constructor - initializes time fields to zero */
    TimeSpec( void ) : Seconds(0), NanoSeconds(0) {}

    //=========================================================================
    /** Constructor
     *
     * @param [in] InitialSeconds        Initial Seconds value
     * @param [in] InitialNanoSeconds    Initial Nanoseconds value
     */
    TimeSpec( time_t InitialSeconds, uint32_t InitialNanoSeconds );

    //=========================================================================
    /** Sets the TimeSpec fields from a double containing whole and fractional
     *  seconds since the UNIX epoch
     *
     * @param [in] SecondsSinceEpoch
     *   A double containing whole and fractional seconds since the UNIX epoch
     */
    TimeSpec( double SecondsSinceEpoch );


    //=========================================================================
    /** Copy constructor
     * @param [in] other    A TimeSpec object to copy
     */
    TimeSpec( TimeSpec const& other );


    //=========================================================================
    /** Sets the whole and fractional values of the Timespec
     *
     * @param [in] Sec      Whole seconds value
     * @param [in] NanoSec  Fractional seconds value in nanoseconds
     */
    void Set( uint32_t Sec, uint32_t NanoSec )
    { Seconds = Sec; NanoSeconds = NanoSec; }


    //=========================================================================
    /** Sets the TimeSpec fields from a double containing whole and fractional
     *  seconds since the UNIX epoch
     *
     * @param [in] SecondsSinceEpoch
     *   A double containing whole and fractional seconds since the UNIX epoch
     */
    void Set( double SecondsSinceEpoch );


    //=========================================================================
    /** Sets the whole and fractional seconds to zero */
    void Zero( void ) { Seconds = NanoSeconds = 0; }


    /** Returns a double containing the whole and fractional seconds held in
     *  the object */
    double AsDouble( void ) const;


    //=========================================================================
    /** @addtogroup TimeSpec_Operators Overloaded operators
     * @{
     */

    /** Copies the time from another TimeSpec object */
    TimeSpec& operator=( const TimeSpec& other );

    /** Adds the fields of another TimeSpec to the TimeSpec object */
    TimeSpec& operator+=( const TimeSpec& rhs );

    /** Subtracts the fields of another TimeSpec from the TimeSpec object,
     *  saturating the time at zero so that a TimeSpec cannot be negative */
    TimeSpec& operator-=( const TimeSpec& rhs );



    /** Returns the sum of two TimeSpec objects' fields */
    TimeSpec operator+( const TimeSpec& rhs ) const;

    /** Returns the absolute difference of two TimeSpec objects */
    TimeSpec operator-( const TimeSpec& rhs ) const;



    /** Returns true if the magnitude of rhs is exactly equal to this */
    bool operator ==( const TimeSpec& rhs ) const;

    /** Returns true if the magnitude of rhs is less than this */
    bool operator <( const TimeSpec& rhs ) const;

    /** Returns true if the magnitude of rhs is less than or equal to this */
    bool operator <=( const TimeSpec& rhs ) const;

    /** Returns true if the magnitude of rhs is greater than this */
    bool operator >( const TimeSpec& rhs ) const;

    /** Returns true if the magnitude of rhs is greater than or equal to this*/
    bool operator >=( const TimeSpec& rhs ) const;

    operator double() { return AsDouble(); }

    /** @} */

private:
    //=========================================================================
    /** Helper function that computes the difference of two TimeSpec objects
     * @param Larger  [in]  The larger of the two TimeSpec objects
     * @param Smaller [in]  The smaller of the two TimeSpec objects
     * @param Result  [out] Resulting TimeSpec value to be populated
     */
    static void Difference( TimeSpec const& Larger, TimeSpec const& Smaller,
                            TimeSpec& Result );

    //=========================================================================
    /** Helper function that carries the NanoSeconds field in a TimeSpec into
     *  its Seconds count if nanoseconds are greater than 1000000000 */
    void Reduce( void );

    uint32_t Seconds;     /**< Whole seconds */
    uint32_t NanoSeconds; /**< Fractional seconds as nanoseconds */

    static const uint32_t NanoSecondsPerSecond;
};



//=============================================================================
/** @class PrecisionTimeInterval
 * @brief
 *  A class that represents a closed interval of time
 */
//=============================================================================
class PrecisionTimeInterval
{
    friend class PrecisionTime;

public:

    /** @enum eTimeUnits
     * ID's used to specify units for PrecisionTimeDelta methods
     */
    enum eTimeUnits
    {
        NANOSECONDS = 0,
        MICROSECONDS,
        MILLISECONDS,
        SECONDS,
        MINUTES,
        HOURS,
        DAYS,
        NUM_UNITS  /**< Internal use only - not a valid time unit!! */
    };

    static const uint32_t NanosecondsPerSecond;
    static const uint32_t NanosecondsPerMillisecond;
    static const uint32_t NanosecondsPerMicrosecond;

    //=========================================================================
    /** Default constructor - creates a delta representing a zero-length
     *  time interval  */
    PrecisionTimeInterval( void );

    //=========================================================================
    /** Creates a PrecisionTimeInterval representing a specified number of
     *  seconds and nanoseconds
     *
     * @param Seconds
     *  Seconds to initialize the interval with
     *
     * @param NanoSeconds
     *  Fractional seconds to initialize the interval with in nanoseconds
     */
    PrecisionTimeInterval( time_t Seconds, uint32_t NanoSeconds );

    //=========================================================================
    /** Creates a PrecisionTimeInterval that is initialize to a specified
     *  value in units
     *
     * @param Value     Time interval value
     * @param Units     Units of Value from eTimeUnits
     */
    PrecisionTimeInterval( uint32_t Value, eTimeUnits Units );

    //=========================================================================
    /** Copy constructor */
    PrecisionTimeInterval( PrecisionTimeInterval const& Other );

    /** Destructor */
    ~PrecisionTimeInterval();


    //=========================================================================
    /** Sets the TimeSpec fields from a double containing whole and fractional
     *  seconds since the UNIX epoch
     *
     * @param [in] SecondsSinceEpoch
     *   A double containing whole and fractional seconds since the UNIX epoch
     */
    void Set( double SecondsSinceEpoch ) { m_TimeSpec.Set(SecondsSinceEpoch); }


    //=========================================================================
    /** Sets the time interval to exactly zero */
    void Zero( void ) { m_TimeSpec.Zero(); }


    //=========================================================================
    /** Returns the interval scaled to reflect a specified unit
     *
     * @note
     *  Be careful with this function!  Since it only returns a 32-bit result,
     *  it is not possible to return very large values (e.g. the number of
     *  nanoseconds in a week)
     *
     * @param Units
     *  Unit to return the interval value in
     *
     * @return
     *  The value of the time interval in specified units
     */
    uint32_t As( eTimeUnits Units );


    //=========================================================================
    /** @return
     *  The object's time interval as a double that includes whole and
     *  fractional seconds */
    double AsDouble( void ) const;


    //=========================================================================
    /** @addtogroup PrecisionTime_Operators Overloaded operators
     * @{
     */

    /** Copies the interval from another PrecisionTimeInterval object */
    PrecisionTimeInterval& operator=( const PrecisionTimeInterval& other );

    /** Computes the sum of two intervals */
    PrecisionTimeInterval operator+( const PrecisionTimeInterval& rhs ) const;

    /** Computes the absolute difference of two intervals */
    PrecisionTimeInterval operator-( const PrecisionTimeInterval& rhs ) const;


    /** Adds a time interval to this interval */
    PrecisionTimeInterval& operator+=( const PrecisionTimeInterval& rhs );

    /** Adds a time interval to this interval */
    PrecisionTimeInterval& operator-=( const PrecisionTimeInterval& rhs );


    /** Returns true if the magnitude of rhs is exactly equal to this */
    bool operator ==( const PrecisionTimeInterval& rhs ) const;

    /** Returns true if the magnitude of rhs is less than this */
    bool operator <( const PrecisionTimeInterval& rhs ) const;

    /** Returns true if the magnitude of rhs is less than or equal to this */
    bool operator <=( const PrecisionTimeInterval& rhs ) const;

    /** Returns true if the magnitude of rhs is greater than this */
    bool operator >( const PrecisionTimeInterval& rhs ) const;

    /** Returns true if the magnitude of rhs is greater than or equal to this*/
    bool operator >=( const PrecisionTimeInterval& rhs ) const;

    /** @} */


private:
    TimeSpec m_TimeSpec;


    //=========================================================================
    /** Constructor that takes a TimeSpec as a param */
    PrecisionTimeInterval( TimeSpec const& OtherTimeSpec )
      : m_TimeSpec(OtherTimeSpec) {}
};





//=============================================================================
/** @class PrecisionTime
 *
 * @brief
 *  An object that encapsulates a precision time value maintained internally
 *  as the number of seconds and nanoseconds elapsed since the UNIX epoch.
 *
 * @remarks
 *  This class is intended to be used for timing events that occur within a
 *  process with nanosecond resolution.
 *
 * @throw PrecisionTimeException
 */
//=============================================================================
class PrecisionTime
{
public:
    //=========================================================================
    /** Creates a PrecisionTime object set to zero (i.e. the UNIX epoch)  */
    PrecisionTime( void );


    //=========================================================================
    /** Creates an instance of the object associated with a specified
     * time in seconds and nanoseconds referenced to the UNIX epoch
     *
     * @param SecondsSinceUnix
     *  Time in seconds to initialize the object with
     *
     * @param NanoSeconds
     *  Nanosecond count to initialize the PrecisionTime object with
     */
    PrecisionTime( time_t SecondsSinceUnix, uint32_t NanoSeconds = 0 );


    //=========================================================================
    /** Copy constructor
     * @param Other A PrecisionTime object to be copied
     */
    PrecisionTime( const PrecisionTime& Other );


    //=========================================================================
    /** Called when the object goes out of scope */
    virtual ~PrecisionTime();


    //=========================================================================
    /** Returns a PrecisionTime object representing the current system time
     *  from CLOCK_REALTIME
     *
     * @param UseMonotonicTime
     *  true to return system time based on CLOCK_MONOTONIC, which is
     *  guaranteed to be monotonic across multiple calls (default); else false
     *  to return system time based on CLOCK_REALTIME
     *
     * @throw
     *  A PrecisionTimeException object onInterval  failure to get the time
     */
    static PrecisionTime Now( bool UseMonotonicTime = true )
    throw(PrecisionTimeException);


    //=========================================================================
    /** Returns a PrecisionTime object representing the time at midnight of the
     *  current day
     */
    static PrecisionTime Midnight( void );



    //=========================================================================
    /** Sets the object's time to the current system time
     *
     * @param UseMonotonicTime
     *  true to return system time guaranteed to be monotonic across multiple
     *  calls
     *
     * @throw
     * 	A std::exception object on failure to get the time
     */
    bool GetSystemTime( bool UseMonotonicTime )
    throw(PrecisionTimeException);


    //=========================================================================
    /** Set the object's time to zero (i.e. the UNIX epoch) */
    void Zero( void ) { m_TimeSpec.Zero(); }

    //=========================================================================
    /** Sets the time from a double containing whole and fractional seconds
     *  since the UNIX epoch
     *
     * @param [in] SecondsSinceEpoch
     *   A double containing whole and fractional seconds since the UNIX epoch
     */
    void Set( double SecondsSinceEpoch ) { m_TimeSpec.Set(SecondsSinceEpoch); }



    //=========================================================================
    /** Returns the integral number of seconds in the object's time */
    time_t WholeSeconds( void ) const { return m_TimeSpec.Seconds; }

    //=========================================================================
    /** Returns the fractional component of the object's time in nanoseconds */
    uint32_t FractionalSeconds( void ) const { return m_TimeSpec.NanoSeconds; }

    //=========================================================================
    /** @return
     *  The object's time as a double that includes whole and fractional
     *  seconds */
    double AsDouble( void ) const;


    //=========================================================================
    /** @addtogroup PrecisionTime_Operators Overloaded operators
     * @{
     */

    /** Sets the time from a double containing epoch time (seconds since UNIX)
     *
     * @param [in] SecondsSinceEpoch
     *   The number of whole and fractional seconds elapsed since the UNIX
     *   epoch
     */
    PrecisionTime& operator=( double SecondsSinceEpoch );

    /** Copies the time from another PrecisionTime object */
    PrecisionTime& operator=(const PrecisionTime& other);

    /** Adds a time interval to the object's time */
    PrecisionTime& operator+=(const PrecisionTimeInterval &rhs);

    /** Subtracts a time interval from the object's time
     * @note    Saturates the object's time at zero in case of underflow */
    PrecisionTime& operator-=(const PrecisionTimeInterval &rhs);


    /** Computes the interval separating the object from another PrecisionTime
     *  object */
    PrecisionTimeInterval operator-(const PrecisionTime& rhs) const;


    /** Adds a time interval in rhs to the object's time */
    PrecisionTime operator+(const PrecisionTimeInterval& rhs);

    /** Subtracts the time interval in rhs from the object's time
     * @note    Saturates the object's time at zero in case of underflow */
    PrecisionTime operator-(const PrecisionTimeInterval& rhs);



    /** Returns true if the time in rhs occurs later */
    bool operator <(const PrecisionTime& rhs) const;

    /** Returns true if the time in rhs occurs later or at exactly the same
     *  time */
    bool operator <=(const PrecisionTime& rhs) const;

    /** Returns true if the time in rhs is the same as the object's time */
    bool operator ==(const PrecisionTime& rhs) const;

    /** Returns true if the time in rhs occurs prior to the object's time */
    bool operator >(const PrecisionTime& rhs) const;

    /** Returns true if the time in rhs occurs prior to- or is exactly the same
     *  as the object's time */
    bool operator >=(const PrecisionTime& rhs) const;

    /** @} */
    //=========================================================================


    //=========================================================================
    /** Returns a PrecisionTimeInterval representing the interval separating
     *  the current system time from the object's time
     *
     * @remarks
     *  The current system time is obtained monotonically
     */
    PrecisionTimeInterval ElapsedTime( void ) const;


private:
    TimeSpec m_TimeSpec;

    //=========================================================================
    /** Constructor that takes a TimeSpec as a param */
    PrecisionTime( TimeSpec const& OtherTimeSpec )
      : m_TimeSpec(OtherTimeSpec) {}

};


}   // END namespace YellowSubUtils

#endif /* PRECISIONTIME_H_ */
