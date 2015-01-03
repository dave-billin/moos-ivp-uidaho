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
/** @file MonitorTarget.h

@brief
	Declaration of the MonitorTarget class

@author Dave Billin

*/
//=============================================================================

#if _MSC_VER > 1000
#pragma once
#endif

#ifndef _MonitorTarget_H_
#define _MonitorTarget_H_

#include <string>
#include <set>
#include <list>
#include <vector>

#include "MOOS/libMOOS/MOOSLib.h"
#include "LogicCondition.h"		// IVP lib_logic logic expression class
#include "InfoBuffer.h"			// IVP lib_logic InfoBuffer class
#include "VarDataPair.h"        // IVP lib_mbutil Variable/value pair class



//=============================================================================
/** An object used to monitor a set of logical conditions placed on selected
 * MOOS variables.  If these logical conditions do not all evaluate to TRUE,
 * the object will publish specified values to one or more MOOS variables.
 */
class MonitorTarget
{
public:

	//=========================================================================
	/** Creates an instance of the MonitorTarget class  */
	MonitorTarget( void );

	/** Copy constructor */
	MonitorTarget( const MonitorTarget& SrcObj );

	//=========================================================================
	//! Called when the object goes out of scope
	~MonitorTarget();



	//=========================================================================
	/** Sets the name string of the MonitorTarget.  Ignores empty strings */
	void SetName( const std::string& sName );

	/** Returns the name of the MonitorTarget object */
	inline const std::string& GetName( void ) const { return m_sName; }



	//=========================================================================
	/** Sets the string to be printed if the MonitorTarget's CONDITION
	 * evaluates to FALSE */
	void SetPrintString( std::string& s ) { m_sPrintString = s; }

	inline const std::string& GetPrintString( void ) const
	{ return m_sPrintString; }



	//=========================================================================
	/** Adds a CONDITION to be monitored by the MonitorTarget */
	bool AddCondition( std::string& ConditionString );





	//=========================================================================
	/** Adds a variable/value pair to be published when all of the
	 * MonitorTarget's CONDITIONS are satisfied
	 *
	 * @param VariableData
	 * 	Variable-value pair to add to the slice's PUBLISH activities
	 */
	void AddToPublishItems( const VarDataPair& VariableData );



	//=========================================================================
	/** Returns a vector containing the names of MOOS variables used in the
	 * MonitorTarget's CONDITIONS
	 */
	std::set<std::string> GetMoosVariableNames( void ) const;




	//=========================================================================
	/** Updates the values of MOOS variables used in the MonitorTarget's
	 * CONDITIONS using a specified InfoBuffer object
	 *
	 * @param InfoBuff
	 * 	A reference to the InfoBuffer object that should be queried to obtain
	 *  the current value of CONDITION variables.
	 */
	void UpdateConditionVariables(InfoBuffer& InfoBuff);



	//=========================================================================
	/** Evaluates all of the MonitorTarget's CONDITIONS and posts PUBLISH items
	 * if any CONDITIONS evaluate to FALSE.
	 *
	 * @returns
	 * 	true if all CONDITIONS being monitored by the MonitorTarget evaluate to
	 * 	TRUE; and false otherwise
	 */
	bool EvaluateConditions( void );


	//=========================================================================
	/** Publishes all items given by the MonitorTarget's PUBLISH rules
	 *
	 * @param CommsTarget
	 * 	Reference to a CMOOSCommClient object to publish to
	 */
	void PublishItems( CMOOSCommClient& CommsTarget );



	//=========================================================================
	/** Assignment operator overload */
	const MonitorTarget& operator=(const MonitorTarget& SrcObj );

private:
	std::string m_sName;	/**< Name associated with the MonitorTarget object */

	std::string m_sPrintString;	/**< String to print if CONDITIONS evaluate to
								 FALSE */

	std::list<LogicCondition> m_Conditions;	/**< CONDITIONS monitored by the
	 	 	 	 	 	 	 	 	 	 	 MonitorTarget object */

	std::set<std::string> m_VarNames;		/**< The names of all MOOS variables used in
	 	 	 	 	 	 	 	 	 the MonitorTarget's CONDITIONS */

	std::vector<VarDataPair> m_PublishVars;	/**< MOOS variables/values to publish
											 when the MonitorTarget's
											 CONDITIONS are all satistfied */

	//=========================================================================
	/** A helper function to update the values of MOOS variables used in the
	 * MonitorTarget's CONDITIONS from the assigned InfoBuffer
	 *
	 * @return
	 * 	true if all variables were updated from the InfoBuffer
	 */
	bool UpdateConditionVariables( void );

};



#endif	// END #ifndef _MonitorTarget_H_
