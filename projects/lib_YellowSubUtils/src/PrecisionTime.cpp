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
/** @file PrecisionTime.cpp
 *
 * @brief
 *  Implementation of the PrecisionTime class
 *
 * @author Dave Billin
 */
//=============================================================================
#include <string.h>
#include <cmath>
#include <unistd.h>
#include <sys/times.h>
#include <sys/time.h>
#include <errno.h>

#include "PrecisionTime.h"

using std::string;
using std::exception;


namespace YellowSubUtils
{

//=============================================================================
//=============================================================================
//=============================================================================
//=============================================================================

/**< Nanoseconds per second */
const uint32_t PrecisionTimeInterval::NanosecondsPerSecond = 1000000000UL;

/**< Nanoseconds per millisecond */
const uint32_t PrecisionTimeInterval::NanosecondsPerMillisecond = 1000000UL;

/**< Nanoseconds per microsecond */
const uint32_t PrecisionTimeInterval::NanosecondsPerMicrosecond = 1000UL;

const uint32_t TimeSpec::NanoSecondsPerSecond = 1000000000UL;


//=============================================================================
TimeSpec::TimeSpec( time_t InitialSeconds, uint32_t InitialNanoSeconds )
  : Seconds(InitialSeconds), NanoSeconds(InitialNanoSeconds)
{
    Reduce();
}


//=============================================================================
TimeSpec::TimeSpec( double SecondsSinceEpoch )
{
    Set( SecondsSinceEpoch );
}

//=============================================================================
TimeSpec::TimeSpec( TimeSpec const& other )
  : Seconds(other.Seconds), NanoSeconds(other.NanoSeconds)
{
    Reduce();
}


//=============================================================================
void TimeSpec::Set( double SecondsSinceEpoch )
{
    double WholeSeconds;
    double FractionalSeconds = modf( SecondsSinceEpoch, &WholeSeconds );

    Seconds = static_cast<uint32_t>( WholeSeconds );
    NanoSeconds =
            static_cast<uint32_t>( FractionalSeconds * NanoSecondsPerSecond );
}


//=============================================================================
TimeSpec& TimeSpec::operator=(const TimeSpec& other )
{
    if ( &other != this )     // Prevent self-assignment
    {
        Seconds = other.Seconds;
        NanoSeconds = other.Seconds;
        Reduce();
    }

    return *this;
}

//=============================================================================
TimeSpec& TimeSpec::operator+=(const TimeSpec &rhs)
{
    Seconds += rhs.Seconds;
    NanoSeconds += rhs.NanoSeconds;
    Reduce();
    return *this;
}


//=============================================================================
TimeSpec& TimeSpec::operator-=(const TimeSpec &rhs)
{
    // Saturate at zero to prevent 'negative' time
    if ( *this < rhs )
    {
        // Saturate at zero
        Seconds = 0;
        NanoSeconds = 0;
    }
    else
    {
        Difference( *this, rhs, *this );
    }
    return *this;
}


//=============================================================================
TimeSpec TimeSpec::operator+( const TimeSpec& rhs ) const
{
    return TimeSpec( Seconds + rhs.Seconds,
                     NanoSeconds + rhs.NanoSeconds );
}



//=============================================================================
TimeSpec TimeSpec::operator-( const TimeSpec& rhs ) const
{
    TimeSpec Diff;

    // Return the absolute difference of the two TimeSpec objects
    if ( *this > rhs )
    {
        Difference( *this, rhs, Diff );
    }
    else
    {
        Difference( rhs, *this, Diff );
    }

    return Diff;
}

//=============================================================================
bool TimeSpec::operator ==( const TimeSpec& rhs ) const
{
    // NOTE: Objects being compared are assumed to be Reduce()-ed already
    return (NanoSeconds == rhs.NanoSeconds) &&
           (Seconds == rhs.Seconds);
}

//=============================================================================
bool TimeSpec::operator <( const TimeSpec& rhs ) const
{
    bool RetVal = false;
    if ( Seconds < rhs.Seconds )
    {
        RetVal = true;
    }
    else if ( rhs.Seconds == Seconds )
    {
        RetVal = (NanoSeconds < rhs.NanoSeconds);
    }
    return RetVal;
}

//=============================================================================
bool TimeSpec::operator <=( const TimeSpec& rhs ) const
{
    return !(*this > rhs);
}

//=============================================================================
bool TimeSpec::operator >( const TimeSpec& rhs ) const
{
    bool RetVal = true;
    if ( Seconds < rhs.Seconds )
    {
        RetVal = false;
    }
    else if ( rhs.Seconds == Seconds )
    {
        RetVal = (NanoSeconds > rhs.NanoSeconds);
    }
    return RetVal;
}

//=============================================================================
bool TimeSpec::operator >=( const TimeSpec& rhs ) const
{
    return !(*this < rhs);
}

//=============================================================================
double TimeSpec::AsDouble( void ) const
{
    double dSeconds = static_cast<double>( Seconds );
    dSeconds += static_cast<double>(NanoSeconds) / NanoSecondsPerSecond;
    return dSeconds;
}

//=============================================================================
void TimeSpec::Difference( TimeSpec const& Larger,
                           TimeSpec const& Smaller,
                           TimeSpec& Result )
{
    uint32_t Borrow = 0;
    if ( Larger.NanoSeconds < Smaller.NanoSeconds )
    {
        Borrow = 1;
        Result.NanoSeconds = TimeSpec::NanoSecondsPerSecond
                      - (Smaller.NanoSeconds - Larger.NanoSeconds);
    }
    else
    {
        Result.NanoSeconds = Larger.NanoSeconds - Smaller.NanoSeconds;
    }

    Result.Seconds = Larger.Seconds - Smaller.Seconds - Borrow;
}


//=============================================================================
void TimeSpec::Reduce( void )
{
    const uint32_t nsecPerSec = 1000000000UL;

    if ( NanoSeconds > nsecPerSec )
    {
        uint32_t Carry = NanoSeconds / nsecPerSec;
        NanoSeconds -= Carry * nsecPerSec;
        Seconds += Carry;
    }
}







//=============================================================================
//=============================================================================
//=============================================================================
//=============================================================================
PrecisionTimeInterval::PrecisionTimeInterval()
   : m_TimeSpec(0, 0)
{
}

//=============================================================================
PrecisionTimeInterval::~PrecisionTimeInterval()
{
}


//=============================================================================
PrecisionTimeInterval::PrecisionTimeInterval( time_t Seconds,
                                              uint32_t NanoSeconds )
   : m_TimeSpec(Seconds, NanoSeconds)
{
}


//=============================================================================
PrecisionTimeInterval::PrecisionTimeInterval( uint32_t Value,
                                              eTimeUnits Units )
{
    // These conversion factors are used to convert second and
    // sub-second values into the native values stored in a
    // PrecisionTimeInterval object
    static const uint32_t UnitConversionTable[NUM_UNITS] =
    {
        1000000000UL,   // NANOSECONDS per second
        1000000UL,      // MICROSECONDS per second
        1000UL,         // MILLISECONDS per second
        1UL,            // SECONDS per second
        60UL,           // Seconds per MINUTE
        3600UL,         // Seconds per HOUR
        86400UL         // Seconds per DAY
    };

    if ( Units < SECONDS )  // Value is in sub-second units
    {
        int ConversionIndex = static_cast<int>(Units);
        uint32_t UnitsPerSecond = UnitConversionTable[ConversionIndex];
        uint32_t UnitsPerNanoSecond = (Units > NANOSECONDS) ?
            (UnitsPerSecond / UnitConversionTable[ConversionIndex - 1]) : 1;
        m_TimeSpec.Seconds = Value / UnitsPerSecond;   // Extract seconds

        // Extract NanoSeconds
        uint32_t Offset = ( m_TimeSpec.Seconds > 0 ) ?
                (Value - (m_TimeSpec.Seconds * UnitsPerSecond)) : 0;
        m_TimeSpec.NanoSeconds = (Value - Offset) * UnitsPerNanoSecond;
    }
    else
    {
        uint32_t SecondsPerUnit = UnitConversionTable[Units];
        m_TimeSpec.Seconds = Value / SecondsPerUnit;
        m_TimeSpec.NanoSeconds = 0;
    }

    m_TimeSpec.Reduce();
}


//=============================================================================
PrecisionTimeInterval::PrecisionTimeInterval(
                                           PrecisionTimeInterval const& Other )
{
    m_TimeSpec = Other.m_TimeSpec;
}


//=============================================================================
uint32_t PrecisionTimeInterval::As( eTimeUnits Units )
{
    // These conversion factors are used to convert second and
    // sub-second values into the native values stored in a
    // PrecisionTimeInterval object
    static const uint32_t UnitConversionTable[NUM_UNITS] =
    {
        1UL,            // nanoseconds to NANOSECONDS
        1000,           // nanoseconds to MICROSECONDS
        1000000UL,      // nanoseconds to MILLISECONDS
        1000000000UL,   // nanoseconds to SECONDS
        60UL,           // Seconds to MINUTES
        3600UL,         // Seconds to HOURS
        86400UL         // Seconds to DAYS
    };

    uint32_t AsValue = 0;

    if ( Units <= SECONDS )
    {
        int NanoSecondIndex= static_cast<int>(Units);
        int SecondsIndex = SECONDS - Units;

        // Convert seconds to desired units
        AsValue = m_TimeSpec.Seconds * UnitConversionTable[SecondsIndex];
        // Convert nanoseconds to desired units
        AsValue += m_TimeSpec.NanoSeconds
                    / UnitConversionTable[NanoSecondIndex];
    }
    else
    {
        AsValue = m_TimeSpec.Seconds / UnitConversionTable[Units];
    }

    return AsValue;
}

//=============================================================================
double PrecisionTimeInterval::AsDouble( void ) const
{
    return m_TimeSpec.AsDouble();
}

//=============================================================================
PrecisionTimeInterval& PrecisionTimeInterval::operator=(
                                            const PrecisionTimeInterval& rhs )
{
    if ( &rhs != this )     // Prevent self-assignment
    {
        m_TimeSpec = rhs.m_TimeSpec;
    }
    return *this;
}



//=============================================================================
PrecisionTimeInterval PrecisionTimeInterval::operator+(
                                            const PrecisionTimeInterval& rhs )
const
{
    return PrecisionTimeInterval( m_TimeSpec + rhs.m_TimeSpec );
}



//=============================================================================
PrecisionTimeInterval PrecisionTimeInterval::operator-(
                                            const PrecisionTimeInterval& rhs )
const
{
    // Return the absolute difference of the two time intervals
    return PrecisionTimeInterval( m_TimeSpec - rhs.m_TimeSpec );
}


//=============================================================================
PrecisionTimeInterval& PrecisionTimeInterval::operator+=(
                                           const PrecisionTimeInterval& rhs )
{
    m_TimeSpec += rhs.m_TimeSpec;
    return *this;
}



//=============================================================================
PrecisionTimeInterval& PrecisionTimeInterval::operator-=(
                                           const PrecisionTimeInterval& rhs )
{
    m_TimeSpec -= rhs.m_TimeSpec;
    return *this;
}



//=============================================================================
bool PrecisionTimeInterval::operator ==( const PrecisionTimeInterval& rhs )
const
{
    // NOTE: Objects being compared are assumed to be Reduce()-ed already
    return m_TimeSpec == rhs.m_TimeSpec;
}


//=============================================================================
bool PrecisionTimeInterval::operator <( const PrecisionTimeInterval& rhs )
const
{
    return m_TimeSpec < rhs.m_TimeSpec;
}

//=============================================================================
bool PrecisionTimeInterval::operator <=( const PrecisionTimeInterval& rhs )
const
{
    return !(m_TimeSpec > rhs.m_TimeSpec);
}

//=============================================================================
bool PrecisionTimeInterval::operator >( const PrecisionTimeInterval& rhs )
const
{
    return (m_TimeSpec > rhs.m_TimeSpec);
}

//=============================================================================
bool PrecisionTimeInterval::operator >=( const PrecisionTimeInterval& rhs )
const
{
    return !(m_TimeSpec < rhs.m_TimeSpec);
}











//=============================================================================
//=============================================================================
//=============================================================================
//=============================================================================
//=============================================================================
PrecisionTime::PrecisionTime( void )
    : m_TimeSpec(0, 0)
{
}

//=============================================================================
PrecisionTime::PrecisionTime( time_t SecondsSinceUnix, uint32_t NanoSeconds )
  : m_TimeSpec(SecondsSinceUnix, NanoSeconds)
{
}

//=============================================================================
PrecisionTime::PrecisionTime( const PrecisionTime& Other )
  : m_TimeSpec( Other.m_TimeSpec )
{
}

//=============================================================================
PrecisionTime::~PrecisionTime()
{
}

//=========================================================================
PrecisionTime PrecisionTime::Now( bool UseMonotonicTime )
throw(PrecisionTimeException)
{
    PrecisionTime NewTime;
    clock_t RefClock = (UseMonotonicTime) ? CLOCK_MONOTONIC : CLOCK_REALTIME;
    if ( NewTime.GetSystemTime( RefClock ) == false )
    {
        throw PrecisionTimeException("Failed to read system time");
    }

    return NewTime;
}

//=============================================================================
PrecisionTime PrecisionTime::Midnight( void )
{
    time_t t;
    struct tm MidnightTime;
    struct tm* pLocalTime;

    time( &t ); // Get the raw time
    pLocalTime = localtime( &t );   // Translate to local time struct

    // Get a local copy of the time
    memcpy( &MidnightTime, pLocalTime, sizeof(struct tm) );

    // Set up the time at midnight
    MidnightTime.tm_hour = 0;
    MidnightTime.tm_min = 0;
    MidnightTime.tm_sec = 0;

    // Return the time object
    uint32_t Seconds = mktime(&MidnightTime);

    // Return the number of seconds at midnight
    return PrecisionTime( Seconds, 0 );
}

//=============================================================================
bool PrecisionTime::GetSystemTime( bool UseMonotonicTime )
throw(PrecisionTimeException)
{
    clock_t RefClock = (UseMonotonicTime) ? CLOCK_MONOTONIC : CLOCK_REALTIME;
    struct timespec ts;

    bool GotTime = false;
    if ( clock_gettime( RefClock, &ts ) == 0 )
	{
	    m_TimeSpec.Seconds = ts.tv_sec;
	    m_TimeSpec.NanoSeconds = ts.tv_nsec;
	    GotTime = true;
	}

	return GotTime;
}


//=============================================================================
double PrecisionTime::AsDouble( void ) const
{
    return m_TimeSpec.AsDouble();
}


//=============================================================================
PrecisionTime& PrecisionTime::operator=( double SecondsSinceEpoch )
{
    m_TimeSpec.Set( SecondsSinceEpoch );
    return *this;
}


//=============================================================================
PrecisionTime& PrecisionTime::operator=(const PrecisionTime& other )
{
    if ( &other != this )     // Prevent self-assignment
    {
        m_TimeSpec = other.m_TimeSpec;
    }
    return *this;
}

//=============================================================================
PrecisionTime& PrecisionTime::operator+=(const PrecisionTimeInterval &rhs)
{
    m_TimeSpec += rhs.m_TimeSpec;
    return *this;
}

//=============================================================================
PrecisionTime& PrecisionTime::operator-=(const PrecisionTimeInterval &rhs)
{
    m_TimeSpec -= rhs.m_TimeSpec;
    return *this;
}

//=============================================================================
PrecisionTimeInterval PrecisionTime::operator-( const PrecisionTime& rhs )
const
{
    // Always return the absolute interval separating the two times
    return PrecisionTimeInterval( m_TimeSpec - rhs.m_TimeSpec );
}


//=============================================================================
PrecisionTime PrecisionTime::operator+( const PrecisionTimeInterval& rhs )
{
    return PrecisionTime( m_TimeSpec + rhs.m_TimeSpec );
}

//=============================================================================
PrecisionTime PrecisionTime::operator-(const PrecisionTimeInterval& rhs)
{
    return PrecisionTime( m_TimeSpec - rhs.m_TimeSpec );
}

//=============================================================================
bool PrecisionTime::operator <(const PrecisionTime& rhs) const
{
    return m_TimeSpec < rhs.m_TimeSpec;
}

//=============================================================================
bool PrecisionTime::operator <=(const PrecisionTime& rhs) const
{
    return !(m_TimeSpec > rhs.m_TimeSpec);
}

//=============================================================================
bool PrecisionTime::operator ==(const PrecisionTime& rhs) const
{
    return (m_TimeSpec == rhs.m_TimeSpec);
}

//=============================================================================
bool PrecisionTime::operator >(const PrecisionTime& rhs) const
{
    return (m_TimeSpec > rhs.m_TimeSpec);
}

//=============================================================================
bool PrecisionTime::operator >=(const PrecisionTime& rhs) const
{
    return !(m_TimeSpec < rhs.m_TimeSpec);
}

//=============================================================================
PrecisionTimeInterval PrecisionTime::ElapsedTime( void ) const
{
    return Now() - *this;
}


}	// END namespace YellowSubUtils


