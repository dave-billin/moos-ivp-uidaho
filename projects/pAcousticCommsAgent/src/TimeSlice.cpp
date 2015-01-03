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
/** @file TimeSlice.cpp
 *
 * @brief	Implementation of the TimeSlice class
 *
 * @author Dave Billin
 */
//=============================================================================

#include "TimeSlice.h"

using namespace std;

//=============================================================================
TimeSlice::TimeSlice( InfoBuffer& TargetInfoBuffer,
					  CMOOSCommClient& CommsClient )
: m_Order(0),
  m_Duration(0.0),
  m_ConditionAlwaysTrue(false),
  m_InfoBuffer(TargetInfoBuffer),
  m_Comms(CommsClient)
{
}


//=============================================================================
TimeSlice::TimeSlice( const TimeSlice& SrcObj )
: m_sName(SrcObj.m_sName),
  m_Order(SrcObj.m_Order),
  m_Duration(SrcObj.m_Duration),
  m_ConditionAlwaysTrue(SrcObj.m_ConditionAlwaysTrue),
  m_Condition(SrcObj.m_Condition),
  m_PublishVars(SrcObj.m_PublishVars),
  m_PublishAlwaysVars(SrcObj.m_PublishAlwaysVars),
  m_InfoBuffer(SrcObj.m_InfoBuffer),
  m_Comms(SrcObj.m_Comms)

{
}



//=============================================================================
TimeSlice::~TimeSlice()
{
}




//=============================================================================
void TimeSlice::SetInfoBuffer( InfoBuffer& TargetInfoBuffer )
{
	m_InfoBuffer = TargetInfoBuffer;
}



//=============================================================================
void TimeSlice::SetName( const string& sName )
{
	if (!sName.empty())
	{
		m_sName = sName;
	}
}



//=============================================================================
void TimeSlice::SetDuration( double Duration_sec )
{
	if (Duration_sec > 0.0)
	{
		m_Duration = Duration_sec;
	}
}




//=============================================================================
bool TimeSlice::SetCondition( string& ConditionString )
{
	m_ConditionAlwaysTrue = ( MOOSStrCmp(ConditionString, "TRUE") );
	return m_Condition.setCondition(ConditionString);
}



//=============================================================================
bool TimeSlice::ConditionIsTrue( void )
{
	return (m_ConditionAlwaysTrue) ? true : m_Condition.eval();
}



//=============================================================================
void TimeSlice::AddToPublishItems( const VarDataPair& VariableData,
								   bool PublishAlways )
{
	if (PublishAlways)
	{
		m_PublishAlwaysVars.push_back(VariableData);
	}
	else
	{
		m_PublishVars.push_back(VariableData);
	}
}




//=============================================================================
vector<string> TimeSlice::GetMoosVariableNames( void )
{
	return m_Condition.getVarNames();
}




//=============================================================================
double TimeSlice::Activate( void )
{
	bool sValIsOK, dValIsOK;
	string sVal;
	double dVal, ReferenceTime;

	ReferenceTime = HPMOOSTime();

	if ( m_Comms.IsConnected() )
	{
		//-------------------------------------------
		// Update the values of MOOS variables used
		// in the time slice's CONDITION
		//-------------------------------------------
		vector<string> ConditionVarNames = GetMoosVariableNames();
		for (vector<string>::iterator iter = ConditionVarNames.begin();
			 iter != ConditionVarNames.end(); iter++)
		{
			sVal = m_InfoBuffer.sQuery(*iter, sValIsOK);
			dVal = m_InfoBuffer.dQuery(*iter, dValIsOK);

			if (sValIsOK)
			{
				m_Condition.setVarVal(*iter, sVal);
			}
			else if (dValIsOK)
			{
				m_Condition.setVarVal(*iter, dVal);
			}
		}

		//-------------------------------------------
		// Publish all ALWAYS variables
		//-------------------------------------------
		for (vector<VarDataPair>::iterator iter = m_PublishAlwaysVars.begin();
			 iter != m_PublishAlwaysVars.end(); iter++)
		{
			if (iter->is_string())
			{
				m_Comms.Notify( iter->get_var(), iter->get_sdata() );
			}
			else
			{
				m_Comms.Notify( iter->get_var(), iter->get_ddata() );
			}
		}


		//-------------------------------------------
		// Publish CONDITION dependent variables
		//-------------------------------------------
		if ( this->ConditionIsTrue() )
		{
			for (vector<VarDataPair>::iterator iter = m_PublishVars.begin();
				 iter != m_PublishVars.end(); iter++)
			{
				if (iter->is_string())
				{
					m_Comms.Notify( iter->get_var(), iter->get_sdata() );
				}
				else
				{
					m_Comms.Notify( iter->get_var(), iter->get_ddata() );
				}
			}
		}
	}

	//----------------------------------------------------
	// Return the time remaining in the slice's duration
	//----------------------------------------------------
	dVal = m_Duration - (HPMOOSTime() - ReferenceTime);
	if (dVal < 0.0)
	{
		MOOSTrace("WARNING: time slice %s lasted %10.6f seconds longer than "
				  "its specified duration (%10.6f seconds)\n",
				  m_sName.c_str(), (-1.0*dVal), m_Duration );
		return 0.0;
	}
	else
	{
		return dVal;
	}
}




//=============================================================================
bool TimeSlice::operator<(const TimeSlice& rhs) const
{
	return m_Order < rhs.m_Order;
}


//=============================================================================
const TimeSlice& TimeSlice::operator=(const TimeSlice& Rhs)
{
    if ( &Rhs != this)
    {
        m_sName = Rhs.m_sName;
        m_Order = Rhs.m_Order;
        m_Duration = Rhs.m_Duration;
        m_Condition = Rhs.m_Condition;
        m_InfoBuffer = Rhs.m_InfoBuffer;
        m_Comms = Rhs.m_Comms;

        m_PublishAlwaysVars = Rhs.m_PublishAlwaysVars;
        m_PublishVars = Rhs.m_PublishVars;
    }
    return *this;
}

