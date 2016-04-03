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
/** @file BHV_ReverseDive.cpp
 *
 * @brief
 * 	Implementation of the BHV_ReverseDive class
 *
 * @author Dave Billin
 */
//=============================================================================


//#include <cstdlib>
#include "BHV_ReverseDive.h"
#include "config.h"
#include <sstream>

#include <MOOS/libMOOS/Utils/MOOSUtilityFunctions.h>
#include <MBUtils.h>
#include <AngleUtils.h>
#include <BuildUtils.h>
#include <ZAIC_HEQ.h>
#include <OF_Coupler.h>


//===================================
// MACROS
//===================================
// Macros to implement verbosity
#define VERBOSE1(_expr_) if (m_Verbosity >= 1) { _expr_; }
#define VERBOSE2(_expr_) if (m_Verbosity >= 2) { _expr_; }
#define VERBOSE3(_expr_) if (m_Verbosity >= 3) { _expr_; }




//===================================
// STATIC MEMBERS
//===================================
const std::string BHV_ReverseDive::sm_sBhvName = "BHV_ReverseDive";

const double BHV_ReverseDive::DefaultPitchDownPitch_deg = 20.0;
const double BHV_ReverseDive::DefaultPitchDownTimeout_sec = 10.0;
const double BHV_ReverseDive::DefaultPitchDownSpeed_mps = 5.0;

const double BHV_ReverseDive::DefaultDiveToDepthTarget_m = 0.5;
const double BHV_ReverseDive::DefaultDiveToDepthTimeout_sec = 10.0;
const double BHV_ReverseDive::DefaultDiveToDepthSpeed_mps = 5.0;

const double BHV_ReverseDive::DefaultLevelOutPitch_deg = 0.0;
const double BHV_ReverseDive::DefaultLevelOutTimeout_sec = 10.0;
const double BHV_ReverseDive::DefaultLevelOutSpeed_mps = 1.0;






//=============================================================================
BHV_ReverseDive::BHV_ReverseDive(IvPDomain Domain)
 : IvPBehavior::IvPBehavior(Domain),
   m_IsSimulation(false),
   m_DepthZaicPeak(0.0),
   m_pDepthZaic(NULL),
   m_SpeedZaicPeak(0.0),
   m_pSpeedZaic(NULL),
   m_pObjectiveFunction(NULL),
   m_DiveState(BHV_ReverseDive::INIT),
   m_StateEntryTime(0.0),
   m_Verbosity(0)
{
    // Print version
    MOOSTrace( "BHV_ReverseDive v%s object created\n", APP_VERSION_STRING );

	// Populate default settings
	m_StateConfig[INIT].Flags.push_back( VarDataPair("DIVE_STATE",
													 "INIT"));

	m_StateConfig[PITCH_DOWN].Pitch_rad =
				MOOS_ANGLE_WRAP( MOOSDeg2Rad(DefaultPitchDownPitch_deg) );
	m_StateConfig[PITCH_DOWN].Depth_m = DefaultDiveToDepthTarget_m;
	m_StateConfig[PITCH_DOWN].Speed_mps = DefaultPitchDownSpeed_mps;
	m_StateConfig[PITCH_DOWN].Timeout_sec = DefaultPitchDownTimeout_sec;
	m_StateConfig[PITCH_DOWN].Flags.push_back( VarDataPair("DIVE_STATE",
														   "PitchDown"));

	m_StateConfig[DIVETODEPTH].Pitch_rad = m_StateConfig[PITCH_DOWN].Pitch_rad;
	m_StateConfig[DIVETODEPTH].Depth_m = DefaultDiveToDepthTarget_m;
	m_StateConfig[DIVETODEPTH].Speed_mps = DefaultDiveToDepthSpeed_mps;
	m_StateConfig[DIVETODEPTH].Timeout_sec = DefaultDiveToDepthTimeout_sec;
	m_StateConfig[DIVETODEPTH].Flags.push_back( VarDataPair("DIVE_STATE",
														    "DiveToDepth"));

	m_StateConfig[LEVEL_OUT].Pitch_rad =
				MOOS_ANGLE_WRAP( MOOSDeg2Rad(DefaultLevelOutPitch_deg) );
	m_StateConfig[LEVEL_OUT].Depth_m = DefaultDiveToDepthTarget_m;
	m_StateConfig[LEVEL_OUT].Speed_mps = DefaultLevelOutSpeed_mps;
	m_StateConfig[LEVEL_OUT].Timeout_sec = DefaultLevelOutTimeout_sec;
	m_StateConfig[LEVEL_OUT].Flags.push_back( VarDataPair("DIVE_STATE",
														  "LevelOut"));

	m_StateConfig[SUCCESS].Flags.push_back( VarDataPair("DIVE_STATE",
														"Success"));

	m_StateConfig[ERROR].Flags.push_back( VarDataPair("DIVE_STATE",
										  "Error"));

	IvPBehavior::setParam("name", sm_sBhvName);
	m_domain = subDomain(m_domain, "depth,speed");

	addInfoVars("NAV_DEPTH, NAV_PITCH");
}


//=============================================================================
BHV_ReverseDive::~BHV_ReverseDive()
{
	if (m_pDepthZaic != NULL)
	{
		delete m_pDepthZaic;
	}

	if (m_pSpeedZaic != NULL)
	{
		delete m_pSpeedZaic;
	}

	if (m_pObjectiveFunction != NULL)
	{
		delete m_pObjectiveFunction;
	}
}




//=============================================================================
bool BHV_ReverseDive::setParam( string sParamName, string sParamValue )
{
	bool Rc = false;

	// DB: standard IvP parameters are parsed before this function is called.

	// Convert the parameter to lower-case for more general matching
	sParamName = tolower(sParamName);

	//--------------------------
	// HANDLE ALL STATE FLAGS
	//--------------------------
	if ( strEnds(sParamName, "_flag", false) )
	{
		VarDataPair Vdp("", "");
		int TargetState = NUM_DIVE_STATES;

		if ( ExtractVarValue(sParamValue, Vdp) )
		{
			if (sParamName == "init_flag")
			{
				TargetState = INIT;
			}
			else if (sParamName == "pitchdown_flag")
			{
				TargetState = PITCH_DOWN;
			}
			else if (sParamName == "divetodepth_flag")
			{
				TargetState = DIVETODEPTH;
			}
			else if (sParamName == "levelout_flag")
			{
				TargetState = DIVETODEPTH;
			}
			else if (sParamName == "error_flag")
			{
				TargetState = DIVETODEPTH;
			}
			else if (sParamName == "success_flag")
			{
				TargetState = SUCCESS;
			}


			if (TargetState < NUM_DIVE_STATES)
			{
				m_StateConfig[TargetState].Flags.push_back(Vdp);
				Rc = true;
			}
		}
	}


	else if ( sParamName == "is_simulation")
	{
		m_IsSimulation = MOOSStrCmp(sParamValue, "true");
		Rc = true;
	}

	else	// The remaining parameters are all numeric
	{
		if ( !isNumber(sParamValue) )
		{
			return false;
		}

		double dVal;
		istringstream Iss(sParamValue, istringstream::in);
		Iss >> dVal;


		//==========================
		// PITCH STATE PARAMETERS
		//==========================
		if ( sParamName == "pitchdown_pitch_deg" )
		{
			m_StateConfig[PITCH_DOWN].Pitch_rad =
						MOOS_ANGLE_WRAP( MOOSDeg2Rad(dVal) );
			m_StateConfig[DIVETODEPTH].Pitch_rad =
						m_StateConfig[PITCH_DOWN].Pitch_rad;
			Rc = true;
		}
		else if ( sParamName == "pitchdown_timeout" )
		{
			if (dVal >= 0.0)
			{
				m_StateConfig[PITCH_DOWN].Timeout_sec = dVal;
				Rc = true;
			}
		}
		else if ( sParamName == "pitchdown_speed" )
		{
			m_StateConfig[PITCH_DOWN].Speed_mps = dVal;
			Rc = true;
		}


		//================================
		// DIVETODEPTH STATE PARAMETERS
		//================================
		else if ( sParamName == "divetodepth_target" )
		{
			m_StateConfig[DIVETODEPTH].Depth_m = dVal;
			m_StateConfig[PITCH_DOWN].Depth_m = dVal;
			Rc = true;
		}
		else if ( sParamName == "divetodepth_timeout" )
		{
			if (dVal >= 0.0)
			{
				m_StateConfig[DIVETODEPTH].Timeout_sec = dVal;
				Rc = true;
			}
		}
		else if ( sParamName == "divetodepth_speed" )
		{
			m_StateConfig[DIVETODEPTH].Speed_mps = dVal;
			Rc = true;
		}


		//================================
		// LEVELOUT STATE PARAMETERS
		//================================
		else if ( sParamName == "levelout_pitch_deg" )
		{
			m_StateConfig[LEVEL_OUT].Pitch_rad =
								MOOS_ANGLE_WRAP( MOOSDeg2Rad(dVal) );
			Rc = true;
		}
		else if ( sParamName == "levelout_timeout" )
		{
			if (dVal > 0.0)
			{
				m_StateConfig[LEVEL_OUT].Timeout_sec = dVal;
				Rc = true;
			}
		}
		else if ( sParamName == "levelout_speed" )
		{
			m_StateConfig[LEVEL_OUT].Speed_mps = dVal;
			Rc = true;
		}



		else if ( sParamName == "verbosity" )
		{
			m_Verbosity = static_cast<int>(dVal);
			Rc = true;
		}
	}

	if (m_Verbosity > 2)
	{
		string InfoMessage = "Set parameter " + sParamName + " to " +
							 sParamValue;
		postWMessage(InfoMessage);
	}

	return Rc;
}



//=============================================================================
void BHV_ReverseDive::onIdleState()
{
	postMessage("DIVE_STATE", "INIT");
	m_StateEntryTime = MOOSTime(false);
}




//=============================================================================
IvPFunction* BHV_ReverseDive::onRunState()
{
	bool DepthOk, PitchOk;
	string s;
	double Depth = getBufferDoubleVal("NAV_DEPTH", DepthOk);
	double Pitch = getBufferDoubleVal("NAV_PITCH", PitchOk);
	int NextState = m_DiveState;

	double t = MOOSTime(false);
	double dt;

	dt = t - m_StateEntryTime;
	dt = (dt < 0) ? 0.0 : dt;

	// Make sure we've got a valid depth and pitch reading
	if ( !DepthOk )
	{
		postWMessage("No NAV_DEPTH value in the info buffer!");
		return NULL;
	}
	if ( !PitchOk )
	{
		postWMessage("No NAV_PITCH value in the info buffer!");
		return NULL;
	}

	//---------------------------------
	// Run the dive state machine
	//---------------------------------
	switch (m_DiveState)
	{
	case INIT:
		/*postMessage("ISCOTTY_CMD", "YawControlIsEnabled=FALSE", "repeatable");
		postMessage("ISCOTTY_CMD", "SpeedControlIsEnabled=FALSE", "repeatable");
		postMessage("ISCOTTY_CMD", "DepthControlIsEnabled=FALSE", "repeatable");
		postMessage(m_sDiveStateVarName, "INIT");
		postMessage(m_sDiveErrorVarName, "NONE");
		*/
		// State transition
		NextState = PITCH_DOWN;
		break;


	case PITCH_DOWN:
		// Detect state timeout
		if (dt > m_StateConfig[PITCH_DOWN].Timeout_sec)
		{
			HandleDiveError("PitchDown state timed out");
			NextState = ERROR;
		}

		/*
		postMessage("DESIRED_PITCH", m_PitchDownDesiredPitch, "repeatable");
		postMessage("DESIRED_THRUST", m_PitchDownDesiredThrust, "repeatable");
		postMessage(m_sDiveStateVarName, "PitchDown");
		postMessage(m_sDiveErrorVarName, "NONE");
		*/

		// State transition:
		else if ( Pitch >= m_StateConfig[PITCH_DOWN].Pitch_rad )
		{
			NextState = DIVETODEPTH;
		}
		break;


	case DIVETODEPTH:
		// Detect state timeout
		if (dt > m_StateConfig[DIVETODEPTH].Timeout_sec)
		{
			HandleDiveError("DiveToDepth state timed out");
			NextState = ERROR;
		}

		/*
		postMessage("DESIRED_PITCH", m_PitchDownDesiredPitch, "repeatable");
		postMessage("DESIRED_THRUST", m_PitchDownDesiredThrust, "repeatable");
		postMessage(m_sDiveStateVarName, "DiveToDepth");
		postMessage(m_sDiveErrorVarName, "NONE");
		*/
		// State transition
		else if ( Depth >= m_StateConfig[DIVETODEPTH].Depth_m )
		{
			NextState = LEVEL_OUT;
		}
		break;


	case LEVEL_OUT:
		// Detect state timeout
		if (dt > m_StateConfig[LEVEL_OUT].Timeout_sec)
		{
			HandleDiveError("LevelOut state timed out");
			NextState = ERROR;
		}

		/*
		postMessage("DESIRED_PITCH", m_LevelOutDesiredPitch, "repeatable");
		postMessage("DESIRED_THRUST", m_LevelOutDesiredThrust, "repeatable");
		postMessage(m_sDiveStateVarName, "LevelOut");
		postMessage(m_sDiveErrorVarName, "NONE");
		*/

		// State transition
		else if ( Pitch <= m_StateConfig[LEVEL_OUT].Pitch_rad )
		{
			NextState = SUCCESS;
		}
		break;


	case SUCCESS:
		/*
		postMessage("DESIRED_PITCH", 0.0, "repeatable");
		postMessage("DESIRED_RUDDER", 0.0, "repeatable");
		postMessage("DESIRED_ELEVATOR", 0.0, "repeatable");
		postMessage("DESIRED_AILERON", 0.0, "repeatable");
		postMessage("DESIRED_THRUST", 0.0, "repeatable");
		postMessage("ISCOTTY_CMD", "YawControlIsEnabled=TRUE", "repeatable");
		postMessage("ISCOTTY_CMD", "SpeedControlIsEnabled=TRUE", "repeatable");
		postMessage("ISCOTTY_CMD", "DepthControlIsEnabled=TRUE", "repeatable");
		postMessage(m_sDiveStateVarName, "Success");
		postMessage(m_sDiveErrorVarName, "NONE");
		*/

		NextState = INIT;
		setComplete();	// Mark the behavior as finished
		break;


	case ERROR:
	default:
		/*
		postMessage("ISCOTTY_CMD", "YawControlIsEnabled=TRUE", "repeatable");
		postMessage("ISCOTTY_CMD", "SpeedControlIsEnabled=TRUE", "repeatable");
		postMessage("ISCOTTY_CMD", "DepthControlIsEnabled=TRUE", "repeatable");
		postMessage("DESIRED_PITCH", 0.0, "repeatable");
		postMessage("DESIRED_THRUST", 0.0, "repeatable");
		postMessage(m_sDiveStateVarName, "Error");
		*/
		NextState = INIT;
		setComplete();	// Mark the behavior as finished
		break;
	}

	// Publish any flags for the current state
	if (m_DiveState < NUM_DIVE_STATES)
	{
		PublishFlagList(m_StateConfig[m_DiveState].Flags);
	}

	// Generate an objective function for the current state
	GenerateObjectiveFunction( m_StateConfig[m_DiveState].Depth_m,
							   m_StateConfig[m_DiveState].Speed_mps );

	// Handle state transitions
	if (NextState != m_DiveState)
	{
		m_StateEntryTime = MOOSTime(false);
		m_DiveState = NextState;
	}

	// Return the current objective function
	return m_pObjectiveFunction;
}







//=============================================================================
void BHV_ReverseDive::onIdleToRunState( void )
{

}


//=============================================================================
void BHV_ReverseDive::onRunToIdleState( void )
{

}


//=============================================================================
void BHV_ReverseDive::onCompleteState( void )
{
	m_DiveState = INIT;
}



//=============================================================================
bool BHV_ReverseDive::ExtractVarValue( string sSource, VarDataPair& VarValue )
{
	string sVarName = MOOSChomp(sSource, "=", true);
	string sVarValue = sSource;
	MOOSTrimWhiteSpace(sVarName);	// Trim white space
	MOOSTrimWhiteSpace(sVarValue);

	if ( sVarName.empty() || sVarValue.empty() )
	{
		return false;
	}
	else
	{
		// DB NOTE: we have to copy a VarDataPair because there is no method
		// to set the double value directly.
		VarDataPair Vdp(sVarName, sVarValue, string("auto"));
		Vdp.set_key(sVarName);
		Vdp.set_sdata(sVarValue);
		VarValue = Vdp;

		return true;
	}
}



//=============================================================================
void BHV_ReverseDive::PublishFlagList( list<VarDataPair>& VdpList )
{
	for ( list<VarDataPair>::iterator iter = VdpList.begin();
		  iter != VdpList.end(); iter++ )
	{
		if (iter->is_string())
		{
			postMessage( iter->get_var(), iter->get_sdata(), "repeatable");
		}
		else
		{
			postMessage( iter->get_var(), iter->get_ddata(), "repeatable");
		}
	}

}


//=============================================================================
void BHV_ReverseDive::HandleDiveError( string sError )
{
	postMessage( "DIVE_ERROR", sError, "repeatable");

	// Post all registered dive errors
	PublishFlagList( m_StateConfig[ERROR].Flags );

	postEMessage(sError);
}




//=============================================================================
/* DB NOTE:
 * 	In this function, we only generate a new speed and/or depth objective
 * 	function if the peak value(s) change.  Otherwise, we simple re-use the
 * 	previous objective function and save calculation time.
 */
IvPFunction* BHV_ReverseDive::GenerateObjectiveFunction( double DesiredDepth,
														 double DesiredSpeed )
{
	bool ObjectiveChanged = false;

	// Generate a depth ZAIC peak function
	if ((m_DepthZaicPeak != DesiredDepth) || (m_pDepthZaic == NULL))
	{
		if (m_pDepthZaic != NULL)
		{
			delete m_pDepthZaic;
		}

		m_pDepthZaic = new ZAIC_HEQ(m_domain, "depth");
		m_pDepthZaic->setSummit(DesiredDepth);
		m_pDepthZaic->setBaseWidth( DesiredDepth * 0.2);
		if(m_pDepthZaic->stateOK() == false)
		{
			string warnings = sm_sBhvName + " Depth ZAIC problems: " +
							  m_pDepthZaic->getWarnings();
			postWMessage(warnings);
		}

		ObjectiveChanged = true;
	}

	// Generate a speed ZAIC peak function
	if ((m_SpeedZaicPeak != DesiredSpeed) || (m_pSpeedZaic == NULL))
	{
		if (m_pSpeedZaic != NULL)
		{
			delete m_pSpeedZaic;
		}

		m_pSpeedZaic = new ZAIC_HEQ(m_domain, "speed");
		m_pSpeedZaic->setSummit(DesiredSpeed);
		m_pSpeedZaic->setBaseWidth( DesiredSpeed * 0.1);
		if(m_pSpeedZaic->stateOK() == false)
		{
			string warnings = sm_sBhvName + " Speed ZAIC problems: " +
								m_pSpeedZaic->getWarnings();
			postWMessage(warnings);
		}

		ObjectiveChanged = true;
	}

	if (ObjectiveChanged)
	{
		IvPFunction* DepthObjFunction = m_pDepthZaic->extractIvPFunction();
		IvPFunction* SpeedObjFunction = m_pSpeedZaic->extractIvPFunction();

		OF_Coupler Coupler;
		m_pObjectiveFunction = Coupler.couple( DepthObjFunction,
											   SpeedObjFunction );
	}

	return(m_pObjectiveFunction);
}

