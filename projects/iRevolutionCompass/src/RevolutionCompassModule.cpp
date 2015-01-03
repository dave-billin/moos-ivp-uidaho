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
/** @file RevolutionCompassModule.cpp
 *
 * @brief
 *  Implementation of the RevolutionCompassModule class
 *
 * @author Dave Billin
 */
//=============================================================================

#include <iomanip>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "RevolutionCompassModule.h"

using namespace std;
using namespace YellowSubUtils;


//-------------------------------
// MACROS TO IMPLEMENT VERBOSITY
//-------------------------------
#define VERBOSE1(_expr_) if (m_Verbosity > 0) { _expr_; }
#define VERBOSE2(_expr_) if (m_Verbosity > 1) { _expr_; }
#define VERBOSE3(_expr_) if (m_Verbosity > 2) { _expr_; }


// Zero-based field indices for HTM messages
enum e_HTM_MessageFieldIndices
{
    HTM_NMEA_ID = 0,        /**< NMEA ID: "PTNTHTM" */
    HTM_TrueHeading,        /**< 'true' heading */
    HTM_MagnetometerStatus, /**< Magnetometer status character */
    HTM_PitchAngle,         /**< Pitch angle in degrees */
    HTM_PitchStatus,        /**< Pitch transducer status character */
    HTM_RollAngle,          /**< Roll angle in degrees */
    HTM_RollStatus,         /**< Roll transducer status character */
    HTM_DipAngle,           /**< Magnetic dip angle */
    HTM_RMHCEMF,            /**< Relative magnitude horizontal component of the
                                 Earth's magnetic field */
    Num_HTM_Message_Fields          /**< Used for bounds-checking */
};


// Zero-based field indices for NCD messages
enum e_NCD_MessageFieldIndices
{
    NCD_tanP = 1,
    NCD_tanR,
    NCD_magN,
    NCD_magE,
    NCD_magH,
    NCD_magV,
    NCD_Heading,
    Num_NCD_Message_Fields
};


// Zero-based field indices for CCD messages
enum e_CCD_MessageFieldIndices
{
    CCD_tanP = 1,
    CCD_tanR,
    CCD_magX,
    CCD_magY,
    CCD_magZ,
    CCD_magT,
    CCD_Heading,
    Num_CCD_Message_Fields
};




const char RevolutionCompassModule::szCRLF[] = { 13, 10, '\0' };
const char szCompassNotInitializedError[] = "Failed to set parameter: compass "
                                            "module is not initialized\n";


//=============================================================================
RevolutionCompassModule::RevolutionCompassModule( void )
 : m_Verbosity(0),
   m_SerialPort(NULL),
   m_CompassIsInitialized(false),
   m_ConnectionTimedOut(false),
   m_Deviation_deg(0.0),
   m_Declination_deg(0.0),
   m_TimeConstant(3,0),
   m_TiltNoiseReductionIsEnabled(true),
   m_MagneticAlarmAcquireCycles(0),
   m_MagneticDeviationLimit(0),
   m_MagnetometerGain(0),
   m_PitchOffset_deg(0.0),
   m_RollOffset_deg(0.0),
   m_TiltAlarmThreshold(0.0),
   m_TiltWarningThreshold(0.0),
   m_Heading_deg(0.0),
   m_DipAngle(0.0),
   m_Pitch_deg(0.0),
   m_Roll_deg(0.0),
   m_RMHCEMF(0.0),
   m_MagneticField(3,0.0f),
   m_MagnetometerStatus('?'),
   m_PitchStatus('?'),
   m_RollStatus('?')
{
}



//=============================================================================
RevolutionCompassModule::~RevolutionCompassModule()
{
    if (m_SerialPort != NULL)
    {
        delete m_SerialPort;
    }
}



//=============================================================================
bool RevolutionCompassModule::Initialize( const string& sSerialPortName,
                                          int BaudRate, int Verbosity )
{
    m_Verbosity = Verbosity;

    if (BaudRate != 0)
    {
        #ifdef _WIN32
        m_SerialPort = new CMOOSNTSerialPort();
        #else
        m_SerialPort = new CMOOSLinuxSerialPort();
        #endif
     }
     else
     {
         m_SerialPort = new DebugFileSerialPort();
         BaudRate = 115200;
     }

    ostringstream oss;
    oss << BaudRate;

    string sBaud = oss.str();

    // Configure serial port parameters
    STRING_LIST PortParameters;
    PortParameters.push_back("BAUDRATE=" + oss.str());
    PortParameters.push_back("STREAMING=TRUE");
    PortParameters.push_back("HANDSHAKING=FALSE");
    PortParameters.push_back("VERBOSE=FALSE");

    if ( !m_SerialPort->Configure(PortParameters) )
    {
        VERBOSE1( MOOSTrace("Failed to configure serial port parameters!\n") );
        return false;
    }


    //-----------------------------------------------
    // Query the compass module's firmware revision
    // and device ID
    //-----------------------------------------------
    bool Successful = true;
    m_VersionInfo = SendCommandString("X?", 1.0);
    m_DeviceID = SendCommandString("W2F4?", 1.0);
    if (m_VersionInfo.empty() || m_DeviceID.empty() )
    {
        return false;
    }


    //-----------------------------------------------
    // Disable all compass report sentences for the
    // duration of initialization
    //-----------------------------------------------
    SetMessageOutputRate(NMEA_PTNTHTM, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_HCXDR, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_HCHDG, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_HCHDT, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_PTNTRCD, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_PTNTCCD, Rate_MessageDisabled);
    SetMessageOutputRate(NMEA_PTNTNCD, Rate_MessageDisabled);


    string sError = "ERROR: Failed to read ";

    //---------------------------------------
    // Read device settings
    //---------------------------------------

    // Read filter time constants
    Successful = true;
    Successful &= ReadCompassParam( "3", m_TimeConstant[0]);
    Successful &= ReadCompassParam( "4", m_TimeConstant[1]);
    Successful &= ReadCompassParam( "5", m_TimeConstant[2]);
    if (!Successful)
    {
        MOOSTrace(sError + "one or more filter time constants\n");
        return false;
    }

    // Read noise reduction filter enablement
    if ( !ReadCompassParam("2.5", m_TiltNoiseReductionIsEnabled) )
    {
        MOOSTrace(sError + "tilt noise reduction enablement\n");
        return false;
    }

    // Read magnetic alarm acquire time
    if ( !ReadCompassParam("15", m_MagneticAlarmAcquireCycles) )
    {
        MOOSTrace(sError + "magnetic alarm acquire time\n");
        return false;
    }

    // Read magnetic alarm deviation limit
    if ( !ReadCompassParam("2A4", m_MagneticDeviationLimit) )
    {
        MOOSTrace(sError + "magnetic deviation limit\n");
        return false;
    }

    // Read magnetometer gain
    if ( !ReadCompassParam("14", m_MagnetometerGain) )
    {
        MOOSTrace(sError + "magnetometer gain\n");
        return false;
    }

    // Read compass deviation setting
    if ( !ReadCompassParam("290", m_Deviation_deg) )
    {
        MOOSTrace(sError + "compass deviation\n");
        return false;
    }

    // Read magnetic declination setting
    if ( !ReadCompassParam("292", m_Declination_deg) )
    {
        MOOSTrace(sError + "magnetic declination\n");
        return false;
    }

    // Read pitch offset
    if ( !ReadCompassParam("298", m_PitchOffset_deg) )
    {
        MOOSTrace(sError + "pitch offset\n");
        return false;
    }


    // Read roll offset
    if ( !ReadCompassParam("29A", m_RollOffset_deg) )
    {
        MOOSTrace(sError + "roll offset\n");
        return false;
    }

    // Read pitch alarm threshold
    if ( !ReadCompassParam("294", m_TiltAlarmThreshold) )
    {
        MOOSTrace(sError + "pitch alarm threshold\n");
        return false;
    }

    // Read roll alarm threshold
    if ( !ReadCompassParam("296", m_TiltWarningThreshold) )
    {
        MOOSTrace(sError + "roll alarm threshold\n");
        return false;
    }


    //-----------------------------------------------
    // Enable all transducers in XDR messages
    //-----------------------------------------------
    Successful = true;
    Successful &= WriteCompassParam("1.0", true); // Pitch
    Successful &= WriteCompassParam("1.1", true); // Roll
    Successful &= WriteCompassParam("1.2", true); // MagX
    Successful &= WriteCompassParam("1.3", true); // MagY
    Successful &= WriteCompassParam("1.4", true); // MagZ

    if (!Successful)
    {
        MOOSTrace(sError + "XDR message content\n");
    }


    //-----------------------------------------------
    // Enable HTM and XDR compass report sentences
    // at the default rate of 3 Hz
    //-----------------------------------------------
    SetMessageOutputRate(NMEA_PTNTHTM, Rate_3Hz);
    SetMessageOutputRate(NMEA_HCXDR, Rate_3Hz);

    m_LastRxTime.GetSystemTime(true);

    m_CompassIsInitialized = true;
    return true;
}




//=============================================================================
void RevolutionCompassModule::ProcessSerialComms( void )
{

    static const char* szSentenceIdFields[] = { "$HCHDG", "$HCHDT", "$HCXDR",
                                                "$PTNTHTM", "$PTNTRCD",
                                                "$PTNTCCD", "$PTNTNCD"
                                               };

    /*static const int NmeaSentenceIDs[] = { NMEA_PTNTHTM, NMEA_HCXDR,
                                           NMEA_HCHDT, NMEA_HCHDG,
                                           NMEA_PTNTNCD, NMEA_PTNTCCD,
                                           NMEA_PTNTRCD };
    */

    if (m_SerialPort == NULL)
    {
        return;
    }

    while (1)
    {
        //-----------------------------------------------------
        // Grab received serial data preceding a '\r\n' combo
        //-----------------------------------------------------
        string sRxData;
        double RxTime;
        if ( !m_SerialPort->GetEarliest( sRxData, RxTime ) )
        {
            break;
        }

        // Validate NMEA sentence start - discard telegrams with missing data
        if (sRxData[0] != '$')
        {
            continue;
        }

        m_LastRxTime.GetSystemTime(true);   // Update Rx time

        VERBOSE3( MOOSTrace("[Rx] " + sRxData + "\n") );


        //-----------------------------------------------------
        // Tokenize NMEA fields into a vector
        //-----------------------------------------------------
        vector<string> vNmeaFields;
        VectorizeString(sRxData, vNmeaFields );

        int Id;
        for (Id = 0; Id < Num_NMEA_MessageIDs; Id++)
        {
            if ( MOOSStrCmp(vNmeaFields[0], string(szSentenceIdFields[Id])) )
            {
                break;
            }
        }

        switch (Id)
        {
            case NMEA_HCXDR:
                HandleMessage_HDXDR(vNmeaFields);
                break;

            case NMEA_PTNTHTM:
                HandleMessage_PTNTHTM(vNmeaFields);
                break;

            case NMEA_HCHDT:
            case NMEA_HCHDG:
            case NMEA_PTNTNCD:
            case NMEA_PTNTCCD:
            case NMEA_PTNTRCD:
                break;

            default:
                MOOSTrace("Unrecognized NMEA sentence ID: %s\n",
                          vNmeaFields[0].c_str() );
                break; // Ignore unrecognized sentences
        }
    }

    // Latch connection timeout after 5 seconds
    if (m_LastRxTime.ElapsedTime() > PrecisionTimeInterval(5UL, 0) )
    {
        m_ConnectionTimedOut = true;
    }
}






//=============================================================================
void RevolutionCompassModule::HandleMessage_HDXDR( vector<string>& vNMEA )
{
    float f;

    string s = vNMEA[2];
    if ( FromString<float>(f, vNMEA[2], std::dec) )
    {
        m_Pitch_deg = f;
    }

    if ( FromString<float>(f, vNMEA[6], std::dec) )
    {
        m_Roll_deg = f;
    }

    if ( FromString<float>(f, vNMEA[10], std::dec) )
    {
        m_MagneticField[0] = f; // MAG X
    }

    if ( FromString<float>(f, vNMEA[14], std::dec) )
    {
        m_MagneticField[1] = f; // MAG Y
    }

    if ( FromString<float>(f, vNMEA[18], std::dec) )
    {
        m_MagneticField[2] = f; // MAG Z
    }
}




//=============================================================================
void RevolutionCompassModule::HandleMessage_PTNTHTM( vector<string>& vNMEA )
{
    enum CompassValuesIds
    {
        TrueHeading = 0,
        PitchAngle,
        RollAngle,
        DipAngle,
        RMHCEMF,
        NUM_COMPASSVALUES
    };
    const int Indices[5] = { HTM_TrueHeading, HTM_PitchAngle, HTM_RollAngle,
                             HTM_DipAngle, HTM_RMHCEMF };
    float CompassValues[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    //---------------------------------
    // Update transducer status codes
    //---------------------------------
    m_MagnetometerStatus = vNMEA[HTM_MagnetometerStatus][0];
    m_PitchStatus = vNMEA[HTM_PitchStatus][0];
    m_RollStatus = vNMEA[HTM_RollStatus][0];



    // Convert compass value strings to numeric values
    for (int i = 0; i < 5; i++)
    {
        float f;
        string s = vNMEA[ Indices[i] ];
        if ( !s.empty() )
        {
            if ( FromString<float>(f, s, std::dec) )
            {
                CompassValues[i] = f;
            }
        }
    }

    // Update heading
    if ( !vNMEA[HTM_TrueHeading].empty() )
    {
        m_Heading_deg = CompassValues[TrueHeading];
    }

    // Handle pitch tilt alarms
    //if (m_PitchStatus == 'P')
    if ( vNMEA[HTM_PitchAngle].empty() )
    {
        float Sign = (m_Pitch_deg > 0.0f) ? 1 : -1;
        m_Pitch_deg = m_TiltAlarmThreshold * Sign;
    }
    else
    {
        m_Pitch_deg = CompassValues[PitchAngle];
    }

    // Handle roll tilt alarms
    //if (m_RollStatus == 'P')
    if ( vNMEA[HTM_RollAngle].empty() )
    {
        float Sign = (m_Roll_deg > 0.0f) ? 1 : -1;
        m_Roll_deg = m_TiltAlarmThreshold * Sign;
    }
    else
    {
        m_Roll_deg = CompassValues[RollAngle];
    }

    // Update dip
    m_DipAngle = CompassValues[DipAngle];
    m_RMHCEMF = CompassValues[RMHCEMF];
}







//=============================================================================
bool RevolutionCompassModule::SetMessageOutputRate( const int MessageTypeID,
                                                    const int RateID )
{
    bool b;


    if ( (MessageTypeID < Num_NMEA_MessageIDs) &&
         (RateID < Num_MessageRateIDs) )
    {
        if (m_Verbosity > 1)
        {
            const char* szMessageNameTable[] = { "HCHDG", "HCHDT", "HCXDR",
                                                 "PTNTHTM", "PTNTRCD", "PTNTCCD",
                                                 "PTNTNCD" };

            MOOSTrace("Setting %s NMEA sentence output rate index = %d\n",
                      szMessageNameTable[MessageTypeID], RateID );
        }

        ostringstream oss;
        oss << std::hex << std::uppercase << (MessageTypeID + 7);
        b = WriteCompassParam( oss.str(), static_cast<uint8_t>(RateID) );
    }
    else
    {
        b = false;
    }

    return b;
}





//=============================================================================
bool RevolutionCompassModule::SetFilterTimeConstant( const int FilterID,
                                                const float TimeConstant_sec )
{
    if (!m_CompassIsInitialized)
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (FilterID < Num_Filter_IDs)
    {
        if (m_Verbosity > 1)
        {
            const char* szFilterNameTable[] = {"Tilt", "Magnetic", "Alarm"};
            MOOSTrace("Setting %s filter time constant to %4.3f sec\n",
                      szFilterNameTable[FilterID], TimeConstant_sec);
        }

        uint8_t Tc = static_cast<uint8_t>(TimeConstant_sec * 13.5);

        // Do nothing if new time constant equals the old time constant
        if (Tc == m_TimeConstant[FilterID])
        {
            return true;
        }

        ostringstream oss;
        oss << std::hex << (FilterID + 3);


        bool b = WriteCompassParam(oss.str(), Tc);

        if (b)
        {
            m_TimeConstant[FilterID] = Tc;
        }

        return b;
    }
    else
    {
        return false;
    }
}



//=============================================================================
bool RevolutionCompassModule::GetFilterTimeConstant( const int FilterID,
                                                     float& TimeConstant_sec )
{
    if (FilterID < Num_Filter_IDs)
    {
        TimeConstant_sec = m_TimeConstant[FilterID] / 13.5;
        return true;
    }
    else
    {
        return false;
    }
}




//=============================================================================
bool RevolutionCompassModule::EnableTiltNoiseReduction(
                                           const bool NoiseReductionIsEnabled )
{
    if (!m_CompassIsInitialized)
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting tilt filter enablement: %s\n",
                  (NoiseReductionIsEnabled) ? "ENABLED" : "DISABLED");
    }

    // Do nothing if old value equals new value
    if (NoiseReductionIsEnabled == m_TiltNoiseReductionIsEnabled)
    {
        return true;
    }

    bool b = WriteCompassParam("2.5", NoiseReductionIsEnabled);

    if (b)
    {
        m_TiltNoiseReductionIsEnabled = NoiseReductionIsEnabled;
    }

    return b;
}





//=============================================================================
bool RevolutionCompassModule::SetMagneticAlarmParameters(
                                                const uint16_t DeviationLimit,
                                                const uint8_t NumAcquireCycles )
{
    if (!m_CompassIsInitialized)    // Do nothing if compass isn't initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    //---------------------------
    // Set alarm acquire time
    //---------------------------
    // Only send command if parameter value is changing
    if (m_MagneticAlarmAcquireCycles != NumAcquireCycles)
    {
        if ( WriteCompassParam("15", NumAcquireCycles) )
        {
            m_MagneticAlarmAcquireCycles = NumAcquireCycles;
        }
        else
        {
            return false;
        }
    }


    //---------------------------
    // Set deviation limit
    //---------------------------
    // Only send command if parameter value is changing
    if (DeviationLimit != m_MagneticDeviationLimit)
    {
        if ( WriteCompassParam("2A4", DeviationLimit) )
        {
            m_MagneticDeviationLimit = DeviationLimit;
        }
        else
        {
            return false;
        }
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting magnetic alarm acquire cycles to %d\n",
                  NumAcquireCycles);
        MOOSTrace("Setting magnetic deviation limit to %d\n", DeviationLimit);
    }

    return true;
}





//=============================================================================
void RevolutionCompassModule::GetMagneticAlarmParameters(
                                               uint16_t& DeviationLimit,
                                               uint8_t& NumAcquireCycles )
                                               const
{
    DeviationLimit = m_MagneticDeviationLimit;
    NumAcquireCycles = m_MagneticAlarmAcquireCycles;
}





//=============================================================================
bool RevolutionCompassModule::SetMagnetometerGain( const uint8_t Gain )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting magnetometer gain index to %d\n", Gain);
    }

    // Only send command if parameter value is changing
    if (Gain == m_MagnetometerGain)
    {
        return true;
    }

    bool b = WriteCompassParam("14", Gain);
    if (b)
    {
        m_MagnetometerGain = Gain;
    }

    return b;
}



//=============================================================================
bool RevolutionCompassModule::SetDeviation( const float Deviation_deg )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting Deviation to %4.1f degrees\n", Deviation_deg);
    }


    // Only send command if parameter value is changing
    if (Deviation_deg == m_Deviation_deg)
    {
        return true;
    }

    bool b = WriteCompassParam("290", Deviation_deg);
    if (b)
    {
        m_Deviation_deg = Deviation_deg;
    }

    return b;
}




//=============================================================================
bool RevolutionCompassModule::SetDeclination( const float Declination_deg )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting magnetic Declination to %4.1f degrees\n",
                  Declination_deg);
    }

    // Only send command if parameter value is changing
    if (Declination_deg == m_Declination_deg)
    {
        return true;
    }

    bool b = WriteCompassParam("292", Declination_deg);
    if (b)
    {
        m_Declination_deg = Declination_deg;
    }

    return b;
}




//=============================================================================
bool RevolutionCompassModule::SetPitchOffset( const float Offset_deg )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting pitch offset to %4.1f degrees\n", Offset_deg);
    }

    // Only send command if parameter value is changing
    if (Offset_deg == m_PitchOffset_deg)
    {
        return true;
    }

    bool b = WriteCompassParam("298", Offset_deg);
    if (b)
    {
        m_PitchOffset_deg = Offset_deg;
    }

    return b;
}





//=============================================================================
bool RevolutionCompassModule::SetRollOffset( const float Offset_deg )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting roll offset to %4.1f degrees\n", Offset_deg);
    }

    // Only send command if parameter value is changing
    if (Offset_deg == m_RollOffset_deg)
    {
        return true;
    }

    bool b = WriteCompassParam("29A", Offset_deg);
    if (b)
    {
        m_RollOffset_deg = Offset_deg;
    }

    return b;
}






//=============================================================================
bool RevolutionCompassModule::SetTiltAlarmThreshold( const float Threshold )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting tilt alarm threshold to %4.1f degrees\n",
                  Threshold);
    }

    // Only send command if parameter value is changing
    if (Threshold == m_TiltAlarmThreshold)
    {
        return true;
    }

    bool b = WriteCompassParam("294", Threshold);
    if (b)
    {
        m_TiltAlarmThreshold = Threshold;
    }

    return b;
}




//=============================================================================
bool RevolutionCompassModule::SetTiltWarningThreshold( const float Threshold )
{
    if (!m_CompassIsInitialized)    // Compass module must be initialized
    {
        MOOSTrace(szCompassNotInitializedError);
        return false;
    }

    if (m_Verbosity > 1)
    {
        MOOSTrace("Setting tilt warning threshold to %4.1f degrees\n",
                  Threshold);
    }

    // Only send command if parameter value is changing
    if (Threshold == m_TiltWarningThreshold)
    {
        return true;
    }

    bool b = WriteCompassParam("296", Threshold);
    if (b)
    {
        m_TiltWarningThreshold = Threshold;
    }

    return b;
}




//=============================================================================
bool RevolutionCompassModule::SetBaudRate( const int BaudRateID )
{
    if (BaudRateID < Num_BaudRate_IDs)
    {
        if (m_Verbosity > 1)
        {
            const char* szBaudRateTable[] = {"2400", "4800", "9600", "19200"};
            MOOSTrace("Setting serial baud rate to %s baud\n",
                      szBaudRateTable[BaudRateID]);
        }

        return WriteCompassParam("6", static_cast<uint8_t>(BaudRateID));
    }
    else
    {
        return false;
    }
}




//=============================================================================
string RevolutionCompassModule::GetMagnetometerStatus( void ) const
{
    return GetStatusStringFromCharacter(m_MagnetometerStatus);
}



//=============================================================================
string RevolutionCompassModule::GetPitchStatus( void ) const
{
    return GetStatusStringFromCharacter(m_PitchStatus);
}



//=============================================================================
string RevolutionCompassModule::GetRollStatus( void ) const
{
    return GetStatusStringFromCharacter(m_RollStatus);
}



//=============================================================================
const string RevolutionCompassModule::GetDeviceParamString( void ) const
{
    if (!m_CompassIsInitialized)
    {
        return "Compass module not initialized.\n";
    }

    ostringstream oss;   // String to return
    string sBar80(78, '=');    // 80-character bar

    oss << sBar80 << "\n"
        << "REVOLUTION COMPASS PARAMETERS\n"
        << sBar80 << endl;

    oss << m_VersionInfo << "\n"
        << "Device ID: " << m_DeviceID
        << endl;

    oss << "Deviation:   " << m_Deviation_deg << "°\n"
        << "Declination: " << m_Declination_deg << "°\n"
        << endl;

    oss << "TILT FILTERS: " << ((m_TiltNoiseReductionIsEnabled) ?
                                            "Enabled" : "Disabled") << endl;

    oss << "\nTIME CONSTANTS\n"
        << "   Tilt:     " << (m_TimeConstant[TiltFilter] / 13.5) << " sec\n"
        << "   Magnetic: " << static_cast<int>(m_TimeConstant[MagneticFilter])
                           << " sec\n"
        << "   Alarm:    " << static_cast<int>(m_TimeConstant[AlarmFilter])
                           << " sec\n"
        << endl;

    oss << "Tilt Sensors:\n"
        << "   Tilt Alarm Threshold: " << m_TiltAlarmThreshold << "°\n"
        << "   Tilt Warning Threshold:  " << m_TiltWarningThreshold << "°\n"
        << "   Pitch Offset: " << m_PitchOffset_deg << "°\n"
        << "   Roll Offset: " << m_RollOffset_deg << "°\n"
        << endl;

    oss << "MAGNETOMETER\n"
        << "   Gain: " << static_cast<int>(m_MagnetometerGain) << "\n"
        << "   Deviation Limit: "
                << static_cast<int>(m_MagneticDeviationLimit) << "\n"
        << "   Alarm Acquire Cycles: "
                << static_cast<int>(m_MagneticAlarmAcquireCycles) << "\n"
        << sBar80 << endl;

    return oss.str();
}






//=============================================================================
string RevolutionCompassModule::GetStatusStringFromCharacter( const char c )
{
    static const char StatusChars[] = { "CLMNOPV" };
    static const char* StatusStrings[] = { "Magnetometer calibration alarm",
                                           "Low Alarm",
                                           "Low Warning",
                                           "Normal",
                                           "High Warning",
                                           "High Alarm",
                                           "Magnetometer Voltage Level Alarm"
                                          };

    for (int i = 0; i < 7; i++)
    {
        if (c == StatusChars[i])
        {
            return StatusStrings[i];
        }
    }

    return "???";   // DB: we should never see this!
}





//=============================================================================
string RevolutionCompassModule::SendCommandString( const string sCommandString,
                                                   const double TimeoutSec )
{
    if (sCommandString.empty() || (m_SerialPort == NULL) )
    {
        return "";
    }

    // Calculate message checksum
    const char* c = sCommandString.c_str();
    char Accum = 0;
    int i = sCommandString.length();
    do
    {
        Accum ^= *c++;
    }
    while (--i > 0);

    // Prepend the '@' symbol, and append the checksum value and <cr><lf>
    std::ostringstream ss;
    ss << setw(2) << setfill('0') << setiosflags(ios::right) << std::hex
       << std::uppercase << static_cast<int>(Accum);
    string TxString = "@" + sCommandString + "*" + ss.str() + szCRLF;

    //------------------------------------------
    // Send the command string to the compass
    //------------------------------------------
    double t;
    m_SerialPort->Flush();
    m_SerialPort->Write( TxString.c_str(), TxString.length(), &t );

    // Print transmitted NMEA sentence
    VERBOSE3( MOOSTrace("[Tx] " + TxString) );

    //------------------------------------------
    // Wait for a reply from the compass
    //------------------------------------------
    string sReply;
    double d;
    PrecisionTime RefTime(true);
    do
    {
        if ( m_SerialPort->GetEarliest(sReply, d) )
        {
            // Ignore all received data except the command response
            if ( sReply[0] == '@')
            {
                //----------------------------------------
                // Validate received message's checksum
                //----------------------------------------

                // Extract message checksum
                size_t NathanHale_Index = sReply.find("*");
                if (NathanHale_Index == string::npos)
                {
                    // Bug out if no checksum field is present
                    continue;
                }

                // Print received NMEA sentence
                VERBOSE3( MOOSTrace("[Rx] " + sReply + "\n") );

                // Extract message checksum value
                string sChecksumField = sReply.substr(NathanHale_Index + 1, 2);
                istringstream iss(sChecksumField);
                int ReceivedChecksum;
                iss >> hex >> ReceivedChecksum;
                string sMessageContent = sReply.substr(1, NathanHale_Index - 1);
                ReceivedChecksum &= 0xff;

                // Calculate message checksum
                char Accum = 0;
                for (string::iterator iter = sMessageContent.begin();
                     iter != sMessageContent.end(); iter++)
                {
                    Accum ^= *iter;
                }

                // Make sure checksums match
                if (static_cast<char>(ReceivedChecksum) != Accum)
                {
                    MOOSTrace("Message checksum failed in command reply: %s\n",
                              sReply.c_str() );
                    return "";
                }

                //----------------------------------------
                // Checksum OK.  Return the command reply
                // stripped of the leading '@' and
                // trailing checksum field
                //----------------------------------------
                sReply = sReply.substr(1, NathanHale_Index - 1);
                //MOOSPause(100);
                return sReply;
            }
        }

    }
    while ( RefTime.ElapsedTime().AsDouble() < TimeoutSec );

    // STATUS:  we've timed out waiting for a response from the compass
    return "";
}




//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                bool& ParamValue )
{
    string sReply = SendCommandString( "F" + sAddress + "?", 1.0);
    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    int i;
    if ( !FromString<int>(i, sReply, std::dec) )
    {
        return false;   // Should never have this problem...
    }

    ParamValue = (i != 0);

    return true;
}



//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                uint8_t& ParamValue )
{
    string sReply = SendCommandString( "B" + sAddress + "?", 1.0);

    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    int ParamAsInt;
    bool b = FromString<int>(ParamAsInt, sReply, std::dec);
    if (b == true)
    {
        ParamValue = static_cast<uint8_t>(ParamAsInt);
    }

    return b;
}



//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                int8_t& ParamValue )
{
    string sReply = SendCommandString( "C" + sAddress + "?", 1.0);

    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    int ParamAsInt;
    bool b = FromString<int>(ParamAsInt, sReply, std::dec);
    if (b == true)
    {
        ParamValue = static_cast<int8_t>(ParamAsInt);
    }

    return b;
}


//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                uint16_t& ParamValue )
{
    string sReply = SendCommandString( "W" + sAddress + "?", 1.0);

    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    int ParamAsInt;
    bool b = FromString<int>(ParamAsInt, sReply, std::dec);
    if (b == true)
    {
        ParamValue = static_cast<uint16_t>(ParamAsInt);
    }

    return b;
}



//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                int16_t& ParamValue )
{
    string sReply = SendCommandString( "I" + sAddress + "?", 1.0);

    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    int ParamAsInt;
    bool b = FromString<int>(ParamAsInt, sReply, std::dec);
    if (b == true)
    {
        ParamValue = static_cast<int16_t>(ParamAsInt);
    }

    return b;
}



//=============================================================================
bool RevolutionCompassModule::ReadCompassParam( const string sAddress,
                                                float& ParamValue )
{
    string sReply = SendCommandString( "I" + sAddress + "?", 1.0);

    if ( sReply.empty() )
    {
        return false;   // Query timed out
    }

    return FromString<float>(ParamValue, sReply, std::dec);
}




//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const bool& ParamValue )
{
    string sCommand = "F" + sAddress + "=";
    sCommand += (ParamValue == true) ? "1":"0";
    return WriteParam(sCommand);
}


//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const uint8_t ParamValue )
{
    ostringstream oss;
    oss << static_cast<int>(ParamValue);

    string sTx = "B" + sAddress + "=" + oss.str();
    return WriteParam( sTx );
}



//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const int8_t ParamValue )
{
    ostringstream oss;
    oss << static_cast<int>(ParamValue);

    string sTx = "C" + sAddress + "=" + oss.str();
    return WriteParam( sTx );
}


//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const uint16_t ParamValue )
{
    ostringstream oss;
    oss << static_cast<int>(ParamValue);

    string sTx = "W" + sAddress + "=" + oss.str();
    return WriteParam( sTx );
}



//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const int16_t ParamValue )
{
    ostringstream oss;
    oss << static_cast<int>(ParamValue);

    string sTx = "I" + sAddress + "=" + oss.str();
    return WriteParam( sTx );
}


//=============================================================================
bool RevolutionCompassModule::WriteCompassParam( const string sAddress,
                                                 const float& ParamValue )
{
    ostringstream oss;
    oss << fixed << setprecision(1) << ParamValue;

    string sTx = "I" + sAddress + "=" + oss.str();
    return WriteParam( sTx );
}




//=============================================================================
bool RevolutionCompassModule::WriteParam( const string& sCommand )
{
    string sReply = SendCommandString(sCommand, 0.5);

    if ( sReply.empty() )
    {
        return false;
    }

    if ( !MOOSStrCmp(sReply, "!0000") )
    {
        MOOSTrace( GetWriteErrorDescription(sReply.substr(1, sReply.length()))
                   + "\n" );
    }

    return true;
}





//=============================================================================
const string RevolutionCompassModule::GetWriteErrorDescription( string sError )
{
    string sErrorDescription;

    uint16_t ErrorCode;
    if ( !FromString(ErrorCode, sError, std::hex) )
    {
        return "???";
    }

    int WriteError = (ErrorCode >> 8) & 0xff;
    int FlagError = (ErrorCode & 0xff);

    if (WriteError != 0)
    {
        const int ErrorCodeTable[] = { 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
                                       0xf7, 0xe8, 0x80, 0x81, 0x82 };
        const char* szWriteErrorTable[] = {
                        "Error in contents of access type field",
                        "Syntax error in input string",
                        "Address not allowed",
                        "Flag number out of range 0-7",
                        "Error in data length field",
                        "Write protect error",
                        "Error in contents of data field",
                        "Error writing EEPROM",
                        "Badly formed input sentence",
                        "Missed LF at end of sentence",
                        "Missed @ or $ at start of sentence" };

        sErrorDescription = "[WRITE ERROR] ";
        for (int i = 0; i < 12; i++)
        {
            if (WriteError == ErrorCodeTable[i])
            {
                sErrorDescription += szWriteErrorTable[i];
                break;
            }
        }

        sErrorDescription += "  ";
    }


    if (FlagError != 0)
    {
        sErrorDescription += "[ERROR FLAGS]";
        if (FlagError & 0x01)
        {
            sErrorDescription += "<Rx overrun>";
        }
        if (FlagError & 0x02)
        {
            sErrorDescription += "<Rx framing error>";
        }
        if (FlagError & 0x04)
        {
            sErrorDescription += "<110-Byte Rx buffer overrun>";
        }
        if (FlagError & 0x08)
        {
            sErrorDescription += "<bad checksum>";
        }
        if (FlagError & 0x10)
        {
            sErrorDescription += "<unrecognized sentence>";
        }
        if (FlagError & 0x20)
        {
            sErrorDescription += "<EEPROM read error>";
        }
        if (FlagError & 0x40)
        {
            sErrorDescription += "<power-on-reset occurred>";
        }
        if (FlagError & 0x40)
        {
            sErrorDescription += "<timeout reset occurred>";
        }
    }

    return sErrorDescription;
}






bool RevolutionCompassModule::VectorizeString( const string& sSource,
                                               vector<string>& vTokens,
                                               const string& sDelimiters )
{
    // Skip sDelimiters at beginning.
    string::size_type lastPos = sSource.find_first_not_of(sDelimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos     = sSource.find_first_of(sDelimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        string sTok = sSource.substr(lastPos, pos - lastPos);
        vTokens.push_back(sTok);

        // Skip sDelimiters.  Note the "not_of"
        lastPos = sSource.find_first_not_of(sDelimiters, pos);

        // For each delimiter between pos and the next token
        // add an empty string
        if ( (pos != string::npos) && (lastPos != string::npos) )
        {
            for ( unsigned int i = (pos + 1); (i < lastPos) && (i < sSource.length()); i++ )
            {
                if ( sDelimiters.find(sSource[i]) != string::npos )
                {
                    vTokens.push_back( string() );
                }
            }
        }
        // Find next "non-delimiter"
        pos = sSource.find_first_of(sDelimiters, lastPos);
    }

    return !vTokens.empty();
}
