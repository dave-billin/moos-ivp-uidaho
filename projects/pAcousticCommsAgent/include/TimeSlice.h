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
/** @file TimeSlice.h
 *
 * @brief	Declaration of a TDM time slice class
 *
 * @author Dave Billin
 */
//=============================================================================

#ifndef _TIMESLICE_H_
#define _TIMESLICE_H_

#include <stdint.h>
#include <string>
#include <map>
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "LogicCondition.h"		// IVP lib_logic logic expression class
#include "InfoBuffer.h"			// IVP lib_logic InfoBuffer class
#include "VarDataPair.h"		// IVP lib_mbutil Variable/value pair class



/** An object used to represent a single time slice in a TDM communications
 *  system
 */
class TimeSlice
{
public:

	//=========================================================================
	/** Creates an instance of the TimeSlice class
	 *
	 * @param TargetInfoBuffer
	 * 	Reference to an infoBuffer object from which the TimeSlice object will
	 * 	obtain the value of MOOS variables used in its CONDITION(s)
	 *
	 * @param CommsClient
	 * 	Reference to the MOOSCommClient object the TimeSlice object will use to
	 * 	publish to MOOS variables
	 */
	TimeSlice( InfoBuffer& TargetInfoBuffer, CMOOSCommClient& CommsClient );

	/** Copy constructor */
	TimeSlice( const TimeSlice& SourceObj );

	/** Called when the object goes out of scope */
	~TimeSlice();

	//=========================================================================
	/** Associates the time slice with an info buffer object that provides
	 *  the values of MOOS variables used in the slice's CONDITION.
	 *
	 * @param TargetInfoBuffer
	 * 	Reference to the InfoBuffer object to associate with.
	 */
	void SetInfoBuffer( InfoBuffer& TargetInfoBuffer );


	//=========================================================================
	/** Sets the name string of the time slice.  Empty strings are ignored. */
	void SetName( const std::string& sName );

	/** Returns the name of the TimeSlice object */
	inline const std::string& GetName( void ) const { return m_sName; }



	//=========================================================================
	/** Sets the time slice's order value */
	void SetOrder( uint32_t Order ) { m_Order = Order; }

	/** Returns the time slice's order value */
	inline uint32_t GetOrder( void ) const { return m_Order; }



	//=========================================================================
	/** Sets the duration of the slice in seconds */
	void SetDuration( double Duration_sec );

	/** Returns the duration of the slice in seconds */
	inline double GetDuration( void ) const { return m_Duration; }


	//=========================================================================
	/** Sets the time slice's CONDITION parameter */
	bool SetCondition( std::string& ConditionString );

	/** Returns true if the time slice's CONDITION evaluates to TRUE */
	inline bool ConditionIsTrue( void );


	//=========================================================================
	/** Adds a variable/value pair to be published
	 *
	 * @param VariableData
	 * 	Variable-value pair to add to the slice's PUBLISH activities
	 *
	 * @param PublishAlways
	 * 	Set this to true if the variable/value should be published regardless
	 * 	of whether the time slice's CONDITION is satisfied.  Default setting
	 * 	is false.
	 */
	void AddToPublishItems( const VarDataPair& VariableData,
							bool PublishAlways = false );


	//=========================================================================
	/** Returns a vector of MOOS variable names used in the time slice's
	 * CONDITION statement.
	 */
	std::vector<std::string> GetMoosVariableNames( void );


	//=========================================================================
	/** Activates the time slice, causing it to evaluate its CONDITION and
	 * carry out any PUBLISH activity.
	 *
	 * @return
	 * 	The number of seconds remaining until the time slot's duration has
	 * 	elapsed.
	 */
	double Activate( void );


	//=========================================================================
	/** Less-than operator overload for easy STL::list sorting of TimeSlice
	 *  objects */
	bool operator<(const TimeSlice& rhs) const;


	//=========================================================================
	/** Assignment operator overload */
	const TimeSlice& operator=(const TimeSlice& rhs);

private:
	std::string m_sName;	/**< Name associated with the time slice */

	uint32_t m_Order;	/**< Integer value used to order the object with
							 respect to other TimeSlice's */

	double m_Duration;	/**< Duration of the time slice in seconds */

	bool m_ConditionAlwaysTrue;	/**< Used to disable evaluation of m_Condition
									 in the case where CONDITION is set to the
									 static value "TRUE" */

	LogicCondition m_Condition;	/**< Logic condition used to enable/disable
									 PUBLISH activity in the time slice */

	std::vector<VarDataPair> m_PublishVars;	/**< MOOS variables/values to
											     publish when the slice's
											     CONDITION is satistfied */

	std::vector<VarDataPair> m_PublishAlwaysVars;	/**< MOOS variables/values
													     that always get
													     published when the
													     slice is active */

	InfoBuffer& m_InfoBuffer;	/**< Reference to an InfoBuffer object that
									 provides the value of MOOS variables
									 in the time slice's CONDITION */

	CMOOSCommClient& m_Comms;	/**< Reference to a MOOS Comms client object
									 used when publishing variable values */

	//=========================================================================
	/** A helper function to update the values of MOOS variables used in the
	 * time slice's CONDITION from the assigned Info Buffer
	 *
	 * @return
	 * 	true if all variables were updated from the Info Buffer
	 */
	bool UpdateConditionVariables( void );
};


#endif	// #ifndef _TIMESLICE_H_
