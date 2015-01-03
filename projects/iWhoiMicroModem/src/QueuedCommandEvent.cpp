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
/** @file QueuedCommandEvent.cpp

@brief
	Implementation of the QueuedCommandEvent class defined in 
	QueuedCommandEvent.h
	
@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the 
	University of Idaho, USA.
*/
//=============================================================================

#include "QueuedCommandEvent.h"

using namespace::std;


//=========================================================================
/* Creates a QueuedCommandEvent object with specified contents
@param sCommandTypeIDRef
	The 5-character string ID of the command (i.e. the message type ID) 
	to be executed on the modem

@param sCommandParamsRef
	A string containing the comma-separated parameters of the command

@param CommandPriority
	The priority this command should be given in the queue of pending
	commands
*/
QueuedCommandEvent::QueuedCommandEvent( const unsigned int TheCommandTypeID, 
										  const string& sCommandParamsRef,
										  const unsigned int CommandPriority )
: CommandTypeID(TheCommandTypeID),
  sCommandParams(sCommandParamsRef),
  Priority(CommandPriority)
{
}

//=========================================================================
/* Creates an empty QueuedCommandEvent object */
QueuedCommandEvent::QueuedCommandEvent() 
{
}



//=========================================================================
/* A function to compare the priorities of two QueuedCommandEvent objects
@return
	- A positive number if the priority of the Lhs object is strictly 
	  greater than that of the Rhs object
    - Zero if the priorities of the two objects are the same
	- A negative number if the priority of the Lhs object is strictly less
	  than that of the Rhs object
*/
int QueuedCommandEvent::ComparePriority( const QueuedCommandEvent& Lhs, 
										  const QueuedCommandEvent& Rhs )
{
	return Lhs.Priority - Rhs.Priority;
}



/* Operator overload to permit sorting of these objects in an STL 
	priority_queue object */
bool QueuedCommandEvent::operator<(const QueuedCommandEvent& rhs) const
{
	return ( ComparePriority(*this, rhs) < 0 );
}

