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
/** @file QueuedCommandEvent.h

@brief
	Declaration of the QueuedCommandEvent class used to represent a 
	prioritized command to be carried out on the modem
	
@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the 
	University of Idaho, USA.
*/
//=============================================================================

#ifndef _CQUEUEDCOMMANDEVENT_H_
#define _CQUEUEDCOMMANDEVENT_H_

#include <string>


//=============================================================================
/** Internal class used by CWhoiCommandManager to represent a prioritized 
	command to be carried out on the modem.  
@internal
*/
class QueuedCommandEvent
{
public:
	int CommandTypeID;		/** Type ID of the command */
	std::string sCommandParams;	/** Comma-separated parameter values */
    unsigned int Priority;  /** The priority of this command */

	//=========================================================================
	/** Creates an empty QueuedCommandEvent object */
	QueuedCommandEvent();

	//=========================================================================
	/** Creates a QueuedCommandEvent object with specified contents
	@param CommandTypeID
		The integer type ID of the command

	@param sCommandParamsRef
		A string containing the comma-separated parameters of the command

	@param CommandPriority
		The priority this command should be given in the queue of pending
		commands
	*/
	QueuedCommandEvent( const unsigned int TheCommandTypeID, 
						 const std::string& sCommandParamsRef,
						 const unsigned int CommandPriority );


	//=========================================================================
	/** A function to compare the priorities of two QueuedCommandEvent objects
	@return
		- A positive number if the priority of the Lhs object is strictly 
		  greater than that of the Rhs object
	    - Zero if the priorities of the two objects are the same
		- A negative number if the priority of the Lhs object is strictly less
		  than that of the Rhs object
	*/
	static int ComparePriority( const QueuedCommandEvent& Lhs, 
								const QueuedCommandEvent& Rhs );

	/** Operator overload to permit sorting of these objects in an STL 
		priority_queue object */
	bool operator<(const QueuedCommandEvent& rhs) const;

};


#endif // #ifndef _CQUEUEDCOMMANDEVENT_H_
