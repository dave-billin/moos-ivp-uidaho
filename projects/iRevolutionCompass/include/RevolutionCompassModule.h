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
/** @file RevolutionCompassModule.h
 *
 * @brief
 *  Declaration of the RevolutionCompassModule class
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef REVOLUTIONCOMPASSMODULE_H_
#define REVOLUTIONCOMPASSMODULE_H_

#include <string>
#include <sstream>
#include <vector>
#include <limits>
#include <stdint.h>

#include "MOOS/libMOOS/Utils/MOOSSerialPort.h"
#include "YellowSubUtils.h"


class RevolutionCompassModule
{
public:

    /** @enum e_MessageOutputRateIds
     * @brief
     *  ID's used to specify the number of times per minute the compass sends a
     *  particular NMEA report sentence
     */
    enum e_MessageOutputRateIds
    {
        Rate_MessageDisabled = 0,/**< Message will not be sent */
        Rate_1perMin,       /**< Message sent once per minute */
        Rate_2perMin,       /**< Message sent twice per minute */
        Rate_3perMin,       /**< Message sent three times per minute */
        Rate_6perMin,       /**< Message sent six times per minute */
        Rate_12perMin,      /**< Message sent twelve times per minute */
        Rate_20perMin,      /**< Message sent twenty times per minute */
        Rate_30perMin,      /**< Message sent thirty times per minute */
        Rate_1Hz,           /**< Message sent once per second */
        Rate_2Hz,           /**< Message sent at 2 Hz*/
        Rate_3Hz,           /**< Message sent at 3 Hz */
        Rate_5Hz,           /**< Message sent at 5 Hz (sync'd to meas cycle) */
        Rate_6p88Hz,        /**< Message sent at 6.88 Hz */
        Rate_10Hz,          /**< Message sent at 10 Hz (sync'd to meas cycle) */
        Rate_13p75Hz,       /**< Message sent at 13.75 Hz */
        Rate_20Hz,          /**< Message sent at 20 Hz (sync'd to meas cycle) */
        Rate_3p43Hz,        /**< Message sent at 3.43 Hz */
        Num_MessageRateIDs       /**< Internal use only */
    };

    /** @enum e_NmeaMessageIds
     * @brief
     *  NMEA message ID's used with SetMessageOutputRate() and for parsing
     *  received NMEA sentences from the compass module
     */
    enum e_NmeaMessageIds
    {
        NMEA_HCHDG,          //!< NMEA_HCHDG
        NMEA_HCHDT,          //!< NMEA_HCHDT
        NMEA_HCXDR,          //!< NMEA_HCXDR
        NMEA_PTNTHTM,        //!< NMEA_PTNTHTM
        NMEA_PTNTRCD,        //!< NMEA_PTNTRCD
        NMEA_PTNTCCD,        //!< NMEA_PTNTCCD
        NMEA_PTNTNCD,        //!< NMEA_PTNTNCD
        Num_NMEA_MessageIDs  //!< NUM_NMEA_SENTENCE_IDS
    };


    //=========================================================================
    /** Creates an instance of the object */
    RevolutionCompassModule( void );

    /** Called when the object goes out of scope */
    virtual ~RevolutionCompassModule();


    //=========================================================================
    /** Must be called to initialize compass module communications and
     *  parameters
     *
     * @param sSerialPortName
     *  A string containing the name of the serial port to open and use for
     *  communicating with the compass module; alternately, this may be used
     *  to specify the full path of a file containing serial data for debugging
     *
     * @param BaudRate
     *  Baud rate to use when communicating with the module, or zero to specify
     *  that sSerialPortName is specifying the path of a file used for serial
     *  port debugging
     *
     * @param Verbosity
     *  Verbosity of debugging messages (non-zero enables
     *
     * @return
     *  true on successful initialization; else false
     */
    bool Initialize( const std::string& sSerialPortName,
                     int BaudRate,
                     int Verbosity = 0 );


    //=========================================================================
    /** Returns true after Initialize() has been called successfully */
    bool IsInitialized( void ) const { return m_CompassIsInitialized; }




    //=========================================================================
    /** Must be called periodically from Iterate() to process serial data
     * received from the compass module
     */
    void ProcessSerialComms( void );




    //=========================================================================
    /** Sends a command to the compass to set a message output rate
     *
     * @param MessageTypeID
     *  ID from e_NmeaMessageIds specifying the message type whose output rate
     *  is being set
     *
     * @param RateID
     *  A rate ID from e_MessageOutputRateIds
     *
     * @return
     *  true on success; else false
     */
    bool SetMessageOutputRate( const int MessageTypeID, const int RateID );



    //=========================================================================
    /** Sends a command to the compass module to set the time constant used for
     * the tilt, magnetic, and alarm filters
     *
     * @param FilterID
     *  An ID from e_FilterIDs specifying which filter time constant to adjust
     *
     * @param TimeConstant_sec
     *  Time constant (seconds) to set the filter to (NOTE: this value gets
     *  multiplied by 13.5 and truncated to an integer by the compass)
     *
     * @return
     *  true on success; else false
     */
    bool SetFilterTimeConstant( const int FilterID,
                                const float TimeConstant_sec );



    //=========================================================================
    /** Queries the time constant used for the tilt, magnetic, or alarm filter
     *
     * @param FilterID
     *  An ID from e_FilterIDs specifying which filter time constant to return
     *
     * @param [out] TimeConstant_sec
     *  Reference to a variable that will be populated with the requested time
     *  constant in seconds.
     *
     * @return
     *  true if the specified filter's time constant was returned in
     *  TimeConstant_sec; else false
     */
    bool GetFilterTimeConstant( const int FilterID,
                                float& TimeConstant_sec );



    //=========================================================================
    /** Enables or disables tilt noise reduction
     *
     * @param NoiseReductionIsEnabled
     *  true if noise reduction should be enabled; else false to disable it
     *
     * @return
     *  true if the specified setting was applied to the compass; else false on
     *  error
     */
    bool EnableTiltNoiseReduction( const bool NoiseReductionIsEnabled );

    //=========================================================================
    /** Returns true if tilt noise reduction is enabled; false otherwise */
    bool TiltNoiseReductionIsEnabled( void ) const
    { return m_TiltNoiseReductionIsEnabled; }




    //=========================================================================
    /** Sets the acquire time cycles and deviation limit used for magnetic
     *  deviation alarms
     *
     * @param DeviationLimit
     *  Value from 0 to 65535 that designates the maximum deviation from the
     *  magnetic reference that may occur before an alarm is triggered
     *
     * @param NumAcquireCycles
     *  Value from 0 to 255 specifying the number of cycles at 13.75 Hz to
     *  acquire a MagH reference for the magnetic deviation alarm function
     *
     * @return
     *  true on success; else false
     */
    bool SetMagneticAlarmParameters( const uint16_t DeviationLimit,
                                     const uint8_t NumAcquireCycles );

    //=========================================================================
    /** Returns the magnetic alarm acquire time cycles and deviation limit
     *
     * @param [out] DeviationLimit
     *  Reference to a variable that will receive the magnetic deviation limit
     *
     * @param [out] NumAcquireCycles
     *  Reference to a variable that will receive the magnetic alarm acquire
     *  time cycle count
     */
    void GetMagneticAlarmParameters( uint16_t& DeviationLimit,
                                     uint8_t& NumAcquireCycles ) const;


    //=========================================================================
    /** Sets the magnetometer gain
     *
     * @param Gain
     *  Gain value to apply (0 to 255)
     *
     * @return
     *  true on success; else false
     */
    bool SetMagnetometerGain( const uint8_t Gain );

    //=========================================================================
    /** Returns the magnetometer gain setting */
    uint8_t GetMagnetometerGain( void ) const { return m_MagnetometerGain; }



    //=========================================================================
    /** Sets compass deviation offset
     *
     * @param Deviation_deg
     *  Angle (degrees) between the magnetic meridian and the axis of the
     *  compass module
     *
     * @return
     *  true on success; else false
     */
    bool SetDeviation( const float Deviation_deg );

    /** Returns the deviation offset of the compass module */
    float GetDeviation( void ) const { return m_Deviation_deg; }




    //=========================================================================
    /** Sets magnetic declination
     *
     * @param Declination_deg
     *  Magnetic declination to apply in degrees
     *
     * @return
     *  true on success; else false
     */
    bool SetDeclination( const float Declination_deg );

    float GetDeclination( void ) const { return m_Declination_deg; }



    //=========================================================================
    /** Sets an offset applied to pitch measurements
     *
     * @param Offset_deg
     *  Offset to apply in degrees
     *
     * @return
     *  true on success; else false
     */
    bool SetPitchOffset( const float Offset_deg );

    //=========================================================================
    /** Returns the offset (in degrees) applied to pitch measurements */
    float GetPitchOffset( void ) const { return m_PitchOffset_deg; }




    //=========================================================================
    /** Sets an offset applied to roll measurements
     *
     * @param Offset_deg
     *  Offset to apply in degrees
     *
     * @return
     *  true on success; else false
     */
    bool SetRollOffset( const float Offset_deg );

    //=========================================================================
    /** Returns the offset (in degrees) applied to roll measurements */
    float GetRollOffset( void ) const { return m_RollOffset_deg; }





    //=========================================================================
    /** Sets the threshold for pitch and roll measurements above which values
     *  will be clamped and an alarm signaled.
     *
     * @param Threshold
     *  Threshold value in degrees; if the magnitude of a measured Pitch or
     *  roll value is greater than this threshold, the measurement will be
     *  clamped at the threshold value and an alarm will be signaled in the
     *  corresponding PitchStatus or RollStatus string.
     *
     * @return
     *  true on success; else false
     */
    bool SetTiltAlarmThreshold( const float Threshold );

    //=========================================================================
    float GetTiltAlarmThreshold( void ) const { return m_TiltAlarmThreshold; }




    //=========================================================================
    /** Sets the threshold for pitch and roll measurements above which a warning
     *  will be signaled.
     *
     * @param Threshold
     *  Threshold value in degrees; if the magnitude of a measured Pitch or
     *  roll value is greater than this threshold, a warning will be signaled in
     *  the corresponding PitchStatus or RollStatus string.
     *
     * @return
     *  true on success; else false
     */
    bool SetTiltWarningThreshold( const float Threshold );

    //=========================================================================
    float GetTiltWarningThreshold( void ) const
    { return m_TiltWarningThreshold; }



    /** @enum e_CompassBaudRateIDs
     * @brief
     *  ID's used to specify a serial baud rate when calling SetBaudRate()
     */
    enum e_CompassBaudRateIDs
    {
        BaudRate_2400 = 1,
        BaudRate_4800 = 2,
        BaudRate_9600 = 3,
        BaudRate_19200 = 4,
        Num_BaudRate_IDs
    };

    //=========================================================================
    /** Sets the serial baud rate the compass module will use the next time it
     *  powers up or is reset
     *
     * @param BaudRateID
     *  Integer ID from e_CompassBaudRateIDs specifying the baud rate to apply
     *
     * @return
     *  true on success; else false
     */
    bool SetBaudRate( const int BaudRateID );


    //=========================================================================
    /** Returns the firmware version reported by the compass module */
    const std::string& GetFirmwareVersion( void ) const { return m_VersionInfo; }


    //=========================================================================
    /** Returns the device ID reported by the compass module */
    const std::string& GetDeviceID( void ) const { return m_DeviceID; }



    //=========================================================================
    /** Returns a string of text summarizing device parameters */
    const std::string GetDeviceParamString( void ) const;


    float GetHeading( void ) const { return m_Heading_deg; }
    float GetDip( void ) const { return m_DipAngle; }
    float GetPitch( void ) const { return m_Pitch_deg; }
    float GetRoll( void ) const { return m_Roll_deg; }
    float GetMagX( void ) const { return m_MagneticField[0]; }
    float GetMagY( void ) const { return m_MagneticField[1]; }
    float GetMagZ( void ) const { return m_MagneticField[2]; }
    std::string GetMagnetometerStatus( void ) const;
    std::string GetPitchStatus( void ) const;
    std::string GetRollStatus( void ) const;

    bool PitchAlarmIsAsserted( void ) const { return m_PitchStatus == 'P'; }
    bool RollAlarmIsAsserted( void ) const { return m_RollStatus == 'P'; }
    bool MagnetometerAlarmIsAsserted( void ) const
    { return (m_MagnetometerStatus < 'M') || (m_MagnetometerStatus > 'O'); }

    /** Returns true if no serial data has been received from the compass for
     *  more than 5 seconds (it is assumed that ProcessSerialComms is called
     *  more often than every 5 seconds) */
    bool ConnectionTimedOut( void ) const { return m_ConnectionTimedOut; }

    /** @enum e_FilterIDs
     * @brief
     *  ID's for use with SetFilterTimeConstant() and GetFilterTimeConstant()
     *  to specify the filter being addressed
     */
    enum e_FilterIDs
    {
        TiltFilter = 0, /**< IIR filter used for tilt */         //!< TiltFilter
        MagneticFilter, /**< IIR filter used for magnetic data *///!< MagneticFilter
        AlarmFilter,    /**< IIR filter used for alarms */       //!< AlarmFilter
        Num_Filter_IDs                                           //!< Num_Filter_IDs
    };



private:
    int m_Verbosity;            /**< Verbosity level for debug messages */

    CMOOSSerialPort* m_SerialPort;  /**< Serial port connection */

    bool m_CompassIsInitialized;    /**< true after Initialize() has been
                                         called successfully */

    std::string m_VersionInfo;   /**< String containing version information
                                      returned by the compass module */

    std::string m_DeviceID;      /**< Device ID string reported by the compass
                                      module */

    bool m_ConnectionTimedOut;  /**< true after 5 seconds elapse without
                                     receiving serial data from the compass
                                     module */

    YellowSubUtils::PrecisionTime m_LastRxTime; /**< Time when the last serial
                                                     data from the compass
                                                     module was processed */


    //---------------------------
    // Device settings
    //---------------------------

    float m_Deviation_deg;       /**< Angle between the magnetic meridian and
                                      the axis of the compass module in
                                      degrees */

    float m_Declination_deg;     /**< Magnetic declination as the angle between
                                      magnetic and geographic meridians (this
                                      indicates the difference between local
                                      magnetic North from 'true' North) */

    std::vector<uint8_t> m_TimeConstant; /**< Time constant used for tilt
                                                   IIR filter (integer value
                                                   from compass) */

    bool m_TiltNoiseReductionIsEnabled;

    uint8_t m_MagneticAlarmAcquireCycles; /**< Magnetic alarm acquire
                                               time cycles */

    uint16_t m_MagneticDeviationLimit;  /**< Limit above/below which a magnetic
                                             deviation will trigger an alarm
                                             (see pg. 42 of user's manual) */

    uint8_t m_MagnetometerGain;     /**< Digital potentiometer setting for
                                         magnetometer gain */

    float m_PitchOffset_deg;    /**< Offset (degrees) applied to pitch
                                     measurements */

    float m_RollOffset_deg;     /**< Offset (degrees) applied to roll
                                     measurements */

    float m_TiltAlarmThreshold; /**< Threshold above which a tilt alarm will be
                                     signaled and a measurement blanked */

    float m_TiltWarningThreshold; /**< Threshold above which a tilt warning
                                       will be signaled */


    //---------------------------
    // Received measurements
    //---------------------------

    float m_Heading_deg;     /**< 'true' compass heading in degrees with
                                  0 = North and a value that increases up to
                                  359.9 degrees with clockwise rotation from
                                  North. */

    float m_DipAngle;       /**< Angle that the earth's magnetic field makes
                                 with the horizontal plane in a specific
                                 geographic location */

    float m_Pitch_deg;      /**< Pitch angle (rotation about the compass
                                 module's lateral axis) in degrees */

    float m_Roll_deg;       /**< Roll angle (rotation about the compass
                                 module's lengthwise axis) in degrees */

    float m_RMHCEMF;        /**< Relative magnitude horizontal component of the
                                 Earth's magnetic field */

    std::vector<float> m_MagneticField; /**< Measured magnetic field along the
                                             [X, Y, Z] axes of the compass
                                             module with units of (Gauss?) */

    char m_MagnetometerStatus;    /**< Reported magnetometer status */
    char m_PitchStatus;           /**< Reported pitch status */
    char m_RollStatus;            /**< Reported roll status */

    static const char szCRLF[];     // NMEA sentences end with this string


    //=========================================================================
    /** Handles a received HDXDR (transducer measurement) NMEA sentence
     *
     * @param vNMEA
     *  Reference to a vector of fields from the message's NMEA sentence
     */
    void HandleMessage_HDXDR( std::vector<std::string>& vNMEA );


    //=========================================================================
    /** Handles a received PTNTHTM (heading, tilt, and magnetic field) NMEA
     *  sentence
     *
     * @param vNMEA
     *  Reference to a vector of fields from the message's NMEA sentence
     */
    void HandleMessage_PTNTHTM( std::vector<std::string>& vNMEA );



    //=========================================================================
    /** Helper function to convert a numeric string to a numeric type */
    template <class T>
    bool FromString( T& t, const std::string& s,
                     std::ios_base& (*f)(std::ios_base&) )
    {
      std::istringstream iss(s);
      return !(iss >> f >> t).fail();
    }



    //=========================================================================
    /** Returns a string corresponding to a status character in the compass
     *  module's HTM message
     *
     * @param c
     *  A status character from an HTM NMEA message
     */
    static std::string GetStatusStringFromCharacter( const char c );



    //=========================================================================
    /** Sends a command string to the compass module and waits up to a
     *  specified timeout for an acknowledgment
     *
     * @param sCommandString
     *  Contents of the command string to be sent to the modem (not including
     *  the leading '@', the checksum field, or the trailing <cr><lf>)
     *
     * @param TimeoutSec
     *  Maximum time (in seconds) to wait for acknowledgment from the modem
     *
     * @return
     *  The string contents of the modem's reply on success (not including the
     *  leading '@' character or trailing *hh checksum); else an empty string
     *  if the timeout expired before an acknowledgment was received
     */
    std::string SendCommandString( const std::string sCommandString,
                                   const double TimeoutSec );


    //=========================================================================
    /* Helper functions to read parameter values from the compass
     *
     * @param sAddress
     *  String containing the address of the parameter to read
     *
     * @param ParamValue
     *  Reference to a variable to be populated with the reply from the compass
     *
     * @return
     *  true if a value was returned in ParamValue; else false if the query
     *  timed out
     */
    bool ReadCompassParam( const std::string sAddress, bool& ParamValue );
    bool ReadCompassParam( const std::string sAddress, uint8_t& ParamValue );
    bool ReadCompassParam( const std::string sAddress, int8_t& ParamValue );
    bool ReadCompassParam( const std::string sAddress, uint16_t& ParamValue );
    bool ReadCompassParam( const std::string sAddress, int16_t& ParamValue );
    bool ReadCompassParam( const std::string sAddress, float& ParamValue );

    bool WriteCompassParam( const std::string sAddress, const bool& ParamValue );
    bool WriteCompassParam( const std::string sAddress, const uint8_t ParamValue );
    bool WriteCompassParam( const std::string sAddress, const int8_t ParamValue );
    bool WriteCompassParam( const std::string sAddress, const uint16_t ParamValue );
    bool WriteCompassParam( const std::string sAddress, const int16_t ParamValue );
    bool WriteCompassParam( const std::string sAddress, const float& ParamValue );

    bool WriteParam( const std::string& sCommand );



    //=========================================================================
    /** Returns a string containing an error description for a parameter write
     *  failure
     *
     * @param sError
     *  Error code returned by the compass for a parameter write failure
     *
     * @return
     *  A string containing corresponding descriptive text
     */
    const std::string GetWriteErrorDescription( std::string sError );




    //=========================================================================
    /** A helper function to tokenize a std::string into a vector of strings
     * @details
     *  This has been modified to return empty strings for consecutive
     *  delimiters.  This is needed for certain applications such as NMEA
     *  sentences that denote empty fields with consecutive commas
     *
     * @param sSource
     *  The string to be tokenized
     *
     * @param sTokens
     *  The vector to be populated with tokens
     *
     * @param sDelimiters
     *  One or more delimiter characters to tokenize around
     *
     * @return
     *  true if at least one delimiter was found in sTokens
     */
    static bool VectorizeString( const std::string& sSource,
                                 std::vector<std::string>& vTokens,
                                 const std::string& sDelimiters = ",");


    //-------------------------------------------------
    // Disable automatically-generated functions
    //-------------------------------------------------
    RevolutionCompassModule (const RevolutionCompassModule&);
    const RevolutionCompassModule& operator= (const RevolutionCompassModule&);
};

#endif /* REVOLUTIONCOMPASSMODULE_H_ */

