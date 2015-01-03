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
/** @file LibBunnySockTest.h
 *
 * @brief
 * 	Declaration of a test driver class for the libBunnySock library
 *
 * @author Dave Billin
 */
//=============================================================================

#ifndef _LIBBUNNYSOCKTEST_H_
#define _LIBBUNNYSOCKTEST_H_

#include <string>
#include <stdint.h>

#include "LibBunnySock.h"



#define LIBBUNNYSOCKTEST_VERSION	1.0f

//=============================================================================
/** A class to implement a test driver for the libBunnySock library */
class LibBunnySockTest : BunnySock::BunnySockListener
{
public:
	/** Creates an instance of the object */
	LibBunnySockTest();
	
	/** Called when the object goes out of scope */
	virtual ~LibBunnySockTest();
	
	/** Runs the test driver as the execution body of a normal application */
	int Run( int argc, const char* argv[] );

	/** Prints the program's usage info to stdout */
	void PrintUsageInfo( void );


	//========================================
	// BunnySockListener interface functions
	//========================================
	
	//=========================================================================
	/** Function called when a packet is received on a BunnySock node.
	 * @note
	 *	This function is called on the receive thread of the BunnySock node.
	 *
	 * @param RxPacket
	 *	A reference to the received BunnySock packet.
	 *
	 * @param pNode
	 *	Reference to the BunnySockNode object that received the packet
	 *
	 * @param TimeStamp_sec
	 *	LocalHost time when the packet was received.
	 */
	void OnPacketReceived( BunnySock::BunnySockPacket& RxPacket,
	                       BunnySock::BunnySockNode& Node,
						   double TimeStamp_sec );


	//=========================================================================
	/** Called when a connection event (connect, disconnect, timeout, error)
	 * occurs on a BunnySock node.
	 *
	 * @details
	 *	Note that this function is called on the receive thread of the
	 *	BunnySockNode.
	 *
	 * @param [in] EventId
	 * 	A member of e_ConnectionEventIds specifying the type of event that
	 *	occurred.
	 *
	 * @param [in] pNode
	 *	Reference to the BunnySockNode object associated with the event
	 *
	 * @param [out] TimeStamp_sec
	 *	LocalHost time when the packet was received.
	*/
	void OnConnectionEvent( int EventId, BunnySock::BunnySockNode& Node,
							double TimeStamp_sec );
	
	
	/** Used to wait for incoming packets */
	enum e_PacketFlagMasks
	{
		SoftwareVersionPacketMask = 0x01,
		NUM_RXPACKETFLAGS
	};
	
	/** Device ID's used for the test nodes */
	enum e_NodeDeviceIds
	{
		SeekNodeDeviceId = 100,
		ListenNodeDeviceId,
		NUM_NODE_DEVICE_IDS
	};
	
private:
	BunnySock::BunnySockNode* m_pNode;	/**< BunnySock node under test */
	
	std::string m_sNodeType;	/**< Type of node being tested ("TCP", "UDP") */
	std::string m_sConnectionMode;	/**< Connection mode ("CLIENT", "SERVER") */
	std::string m_sRemoteHost;	/**< Remote host name from command line */
	uint16_t m_NetworkPort;	/**< Network port to use */
	uint16_t m_DeviceId;	/**< Device ID to associate with the node */
	uint32_t m_ConnectionTimeoutMs;	/**< Connection timeout (milliseconds) */
	uint32_t m_RetryPeriodSec;	/**< Connection retry period in seconds -
									 ignored for client nodes */
	int m_Verbosity;	/**< Verbosity level for node messages */
	int m_RxPacketFlags;


	//=========================================================================
	/** Helper function to parse command line arguments passed in from main()
	 *
	 * @param argc
	 * 	Number of command line parameters from main()
	 *
	 * @param argv
	 *  Array of argument string pointers passed in from main()
	 *
	 * @return
	 * 	true if all command line arguments are valid; else false
	 */
	bool ParseCommandLine(int argc, const char* argv[]);


	//=========================================================================
	/** Helper function to parse a command line option
	 * @param argc
	 * 	Number of command line parameters from main()
	 *
	 * @param argv
	 *  Array of argument string pointers passed in from main()
	 *
	 * @param [inout] ArgIndex
	 *	Index in argv of the option in command line arguments
	 *
	 * @return
	 *	True if the argument was parsed successfully; else false
	 */
	bool ParseOption(int argc, const char* argv[], int& ArgIndex);


	//=========================================================================
	/** Helper function to extract an integer parameter value
	 * @param sz [int]
	 * 	Null-terminated character string to extract value from
	 *
	 * @param [out] IntValue
	 * 	Integer to be populated with the value spcified in sz
	 */
	bool GetIntParamValue(const char* sz, int& IntValue ) const;
};


#endif	// END #ifndef _LIBBUNNYSOCKTEST_H_
