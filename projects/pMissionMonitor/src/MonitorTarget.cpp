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
/** @file MonitorTarget.cpp

@brief
	Implementation of the MonitorTarget class defined in MonitorTarget.h

@author Dave Billin

*/
//=============================================================================

#include <sstream>
#include "MonitorTarget.h"

using namespace::std;

//=============================================================================
MonitorTarget::MonitorTarget()
{
}




//=============================================================================
MonitorTarget::MonitorTarget( const MonitorTarget& SrcObj )
{
	*this = SrcObj;
}




//=============================================================================
MonitorTarget::~MonitorTarget()
{
}




//=============================================================================
void MonitorTarget::SetName( const string& sName )
{
	if (!sName.empty())
	{
		m_sName = sName;
	}
}



//=============================================================================
bool MonitorTarget::AddCondition( string& ConditionString )
{
	LogicCondition NewCondition;
	NewCondition.setAllowDoubleEquals(true);

	if ( NewCondition.setCondition(ConditionString) )
	{
		m_Conditions.push_back(NewCondition);

		// Add condition variable names
		vector<string> VarNames = NewCondition.getVarNames();
		for ( vector<string>::iterator iter = VarNames.begin();
			  iter != VarNames.end(); iter++ )
		{
			m_VarNames.insert(*iter);
		}

		return true;
	}
	else
	{
		return false;
	}
}




//=============================================================================
void MonitorTarget::AddToPublishItems( const VarDataPair& VariableData )
{
	m_PublishVars.push_back(VariableData);
}




//=============================================================================
set<string> MonitorTarget::GetMoosVariableNames( void ) const
{
	return m_VarNames;
}




//=============================================================================
void MonitorTarget::UpdateConditionVariables(InfoBuffer& InfoBuff)
{
	vector<string> ConditionVars;
	string sVal;
	double dVal;

	//-------------------------------------------
	// Update the values of MOOS variables used
	// in each of the MonitorTarget's CONDITIONS
	//-------------------------------------------

	for ( list<LogicCondition>::iterator Condition = m_Conditions.begin();
		  Condition != m_Conditions.end(); Condition++ )
	{
		ConditionVars = Condition->getVarNames();

		for ( vector<string>::iterator VarName = ConditionVars.begin();
			  VarName != ConditionVars.end(); VarName++ )
		{
			bool sValIsOK, dValIsOK;

			sVal = InfoBuff.sQuery(*VarName, sValIsOK);
			if (sValIsOK)
			{
				Condition->setVarVal(*VarName, sVal);
			}
			else
			{
				dVal = InfoBuff.dQuery(*VarName, dValIsOK);
				if (dValIsOK)
				{
					Condition->setVarVal(*VarName, dVal);
				}
			}
		}
	}
}




//=============================================================================
bool MonitorTarget::EvaluateConditions( void )
{
	for ( list<LogicCondition>::iterator iter = m_Conditions.begin();
		  iter != m_Conditions.end(); iter++ )
	{
		if ( iter->eval() == false )
		{
			return false;	// Return false if any CONDITION evaluates FALSE
		}
	}

	return true;
}




//=============================================================================
void MonitorTarget::PublishItems( CMOOSCommClient& CommsTarget )
{
	double t = HPMOOSTime();

	for ( vector<VarDataPair>::iterator p = m_PublishVars.begin();
		  p != m_PublishVars.end(); p++ )
	{
		if (p->is_string())
		{
			CommsTarget.Notify(p->get_var(), p->get_sdata(), t);
		}
		else
		{
			CommsTarget.Notify(p->get_var(), p->get_ddata(), t);
		}
	}
}




//=============================================================================
const MonitorTarget& MonitorTarget::operator=(const MonitorTarget& SrcObj )
{
    if ( &SrcObj != this )
    {
        m_sName = SrcObj.m_sName;
        m_sPrintString = SrcObj.m_sPrintString;
        m_Conditions = SrcObj.m_Conditions;
        m_VarNames = SrcObj.m_VarNames;
        m_PublishVars = SrcObj.m_PublishVars;
    }
    return *this;
}
