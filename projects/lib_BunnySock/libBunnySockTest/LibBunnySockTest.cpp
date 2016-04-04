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
/** @file LibBunnySockTest.cpp
 *
 * @brief
 * 	Implementation of the LibBunnySockTest class used for testing the
 *	LibBunnySock library
 *
 * @author Dave Billin
 */
//=============================================================================

#include <assert.h>
#include <stdlib.h>

#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"
#include "PrecisionTime.h"
#include "LibBunnySockTest.h"

using namespace std;
using namespace YellowSubUtils;
using namespace BunnySock;

#define SOFTWARE_VERSION_PACKET_TYPE	100


struct SoftwareVersionPacket_t
{
    SoftwareVersionPacket_t( float Version, uint16_t DestId = 0,
                             uint32_t MsTimeStamp = 0 )
    {
        Header.PacketType = SOFTWARE_VERSION_PACKET_TYPE;
        Header.MsTimeStamp = MsTimeStamp;
        SoftwareVersion = Version;
    }

	bsock_PacketHeader_t Header;
	float SoftwareVersion;
};



class SoftwareVersionPacket : public BunnySockPacket
{
public:
	SoftwareVersionPacket( float Version, uint16_t DestId = 0,
						   uint32_t MsTimeStamp = 0 )
	: SoftwareVersion(Version)
	{
		SoftwareVersionPacket_t* pPacketData = (SoftwareVersionPacket_t*)&m_PacketData;
		pPacketData->Header.PacketType = SOFTWARE_VERSION_PACKET_TYPE;
		pPacketData->Header.MsTimeStamp = MsTimeStamp;
		pPacketData->SoftwareVersion = Version;
	}

	float SoftwareVersion;
};



//=============================================================================
LibBunnySockTest::LibBunnySockTest()
: m_pNode(NULL),
  m_sNodeType("TCP"),
  m_sConnectionMode("server"),
  m_NetworkPort(20000),
  m_ConnectionTimeoutMs(2000),
  m_RetryPeriodSec(2),
  m_Verbosity(0),
  m_RxPacketFlags(0)

{
	// Generate a random device ID
	srand( static_cast<int>(MOOSTime()) );
	m_DeviceId = rand();
}



//=============================================================================
LibBunnySockTest::~LibBunnySockTest()
{
	if (m_pNode != NULL)	delete m_pNode;
}



//=============================================================================
void LibBunnySockTest::OnPacketReceived( BunnySockPacket& RxPacket,
										 BunnySockNode& Node,
										 double TimeStamp_sec )
{
	bsock_PacketHeader_t* pPacketHeader = RxPacket.GetHeader();
	string sPacketAsHex, s;
	int i;

	MOOSTrace("<====== Incoming Packet Data ================>\n"
			   "Source ID: %-5d\tDest ID: %-d\n"
			   "Type: %-5d\t\tTimeStamp: %08x\n",
			   pPacketHeader->SourceId, pPacketHeader->DestId,
			   pPacketHeader->PacketType, pPacketHeader->MsTimeStamp);

	if (pPacketHeader->PacketType == SOFTWARE_VERSION_PACKET_TYPE)
	{
		SoftwareVersionPacket_t* pVersionPacket =
					static_cast<SoftwareVersionPacket_t*>(RxPacket.GetRawBytes());

		// For version packets, print the version Peer reports
		MOOSTrace("Software version packet: v%3.2f\n", pVersionPacket->SoftwareVersion);
	}
	else
	{
		// For all other packet types, print the payload
		sPacketAsHex = RxPacket.ToHexString();
		sPacketAsHex.erase(0, sizeof(bsock_PacketHeader_t)*2);	// Remove header

		i = 0;
		while (!sPacketAsHex.empty())
		{
			if (sPacketAsHex.length() >= 8)
			{
				s = s + "  " + sPacketAsHex.substr(0, 8);
				sPacketAsHex.erase(0, 8);
			}
			else
			{
				s = s + "  " + sPacketAsHex;
			}

			i++;
			if ( (i % 6 == 0) && (i != 0) )
			{
				s = s + "\n";
			}
		}
		MOOSTrace("\nPayload:\n" + s + "\n");

	}

	MOOSTrace("<============================================>\n\n");
}


//=============================================================================
void LibBunnySockTest::OnConnectionEvent( int EventId, BunnySockNode& Node,
										  double TimeStamp_sec )
{
	BunnySockTcpNode* pTcpNode = dynamic_cast<BunnySockTcpNode*>(&Node);
	string sMessage;

	switch (EventId)
	{
		case BunnySockListener::CONNECTED:
		{
			sMessage = MOOSFormat("Node Connected to %s\n",
								  pTcpNode->GetRemoteHostName().c_str() );
			break;
		}

		case BunnySockListener::CONNECTION_ERROR:
		{
			sMessage = MOOSFormat("Node Connection error\n");
			break;
		}

		case BunnySockListener::CONNECTION_TIMEOUT:
		{
			sMessage = MOOSFormat("Node Connection Timeout!\n" );
			break;
		}

		case BunnySockListener::DISCONNECTED:
		{
			sMessage = MOOSFormat("Node Disconnected\n");
			break;
		}
	}

	MOOSTrace(sMessage);
}






//=============================================================================
int LibBunnySockTest::Run( int argc, const char* argv[] )
{
	// Parse and extract values from command line arguments
	if ( !ParseCommandLine(argc, argv) )
	{
		MOOSTrace("Type 'LibBunnySockTest --help' to display usage\n\n");
		return -1;	// Exit on bad parameter
	}

	int iConnectionMode = (MOOSStrCmp(m_sConnectionMode, "Client")) ?
						   BunnySockTcpNode::CLIENT : BunnySockTcpNode::SERVER;


	if (MOOSStrCmp(m_sRemoteHost, "ANY_IP") || MOOSStrCmp(m_sRemoteHost, "0.0.0.0"))
	{
		if (iConnectionMode == BunnySockTcpNode::CLIENT)
		{
			MOOSTrace("\n%s can only be specified as a remote host for server nodes\n\n",
					  m_sRemoteHost.c_str());
			return -1;
		}
		else
		{
			m_sRemoteHost = "0.0.0.0";
		}
	}

	// Set up application variables for initial display in the case of UDP mode
	if ( MOOSStrCmp(m_sNodeType, "UDP") )
	{
		m_sRemoteHost = "UDP Broadcast";
		m_ConnectionTimeoutMs = 0;
		m_RetryPeriodSec = 0;
	}


	MOOSTrace("\n===============================================\n"
			  "Starting %s BunnySock %s node\n"
			   "Device ID: %d\n"
			   "Target: [%s] on port %d\n"
			   "Connection Timeout: %u ms\n"
			   "Retry Period: %i sec\n"
			   "Verbosity: %d\n"
			   "===============================================\n\n",
			   m_sNodeType.c_str(), m_sConnectionMode.c_str(),
			   m_DeviceId,
			   m_sRemoteHost.c_str(), m_NetworkPort,
			   m_ConnectionTimeoutMs,
			   m_RetryPeriodSec,
			   m_Verbosity );

    SoftwareVersionPacket VersionPacket( 2.0f,  /* Version */
                                         0,     /* Dest ID */
                                         0 );
	VersionPacket.GetHeader()->SourceId = m_DeviceId;
	VersionPacket.GetHeader()->DestId = 100;

	bool using_udp = MOOSStrCmp(m_sNodeType, "UDP");

	// Create a new BunnySock node
	if ( MOOSStrCmp(m_sNodeType, "TCP") )
	{
		BunnySockTcpNode* pTcpNode;
		pTcpNode = new BunnySockTcpNode( iConnectionMode,/* Mode */
										m_sRemoteHost,	/* Target host */
										m_NetworkPort, 	/* Port */
										m_DeviceId,		/* Device ID */
										m_RetryPeriodSec, /* Retry period */
										m_ConnectionTimeoutMs, /* Timeout */
										m_Verbosity );

		assert(pTcpNode != NULL);
		pTcpNode->Start();
		m_pNode = pTcpNode;
	}
	else if ( MOOSStrCmp(m_sNodeType, "UDP") )
	{
		BunnySockUdpNode* pUdpNode;
		pUdpNode = new BunnySockUdpNode( m_NetworkPort, m_DeviceId,
                                       m_Verbosity );
		assert(pUdpNode != NULL);
		pUdpNode->Start();
		m_pNode = pUdpNode;

		/*MOOSTrace("Sorry.  BunnySock UDP nodes are not implemented yet...  "
				  "Stay tuned!\n\n");
		return 0;
		*/
	}

	// Register this object as a listener for the BunnySock nodes
	m_pNode->AddListener(this);

	// Wait for the nodes to connect to each other
	while ( !using_udp && !m_pNode->IsConnected() )
	{
		MOOSPause(100);
	}

	// Send a test packet from the seek node to the listen node
	PrecisionTime Midnight = PrecisionTime::Midnight();
	while ( (m_RxPacketFlags & SoftwareVersionPacketMask) == 0)
	{
		VersionPacket.GetHeader()->MsTimeStamp =
               Midnight.ElapsedTime().As( PrecisionTimeInterval::MILLISECONDS);
		m_pNode->SendPacket( VersionPacket );
		MOOSPause(3000);
	}

	return 0;
}



//=============================================================================
void LibBunnySockTest::PrintUsageInfo( void )
{
	MOOSTrace("\nLibBunnySockTest version %3.2f\n"
			  "Written by Dave Billin\n\n", LIBBUNNYSOCKTEST_VERSION);

	MOOSTrace(
	"DESCRIPTION:\n"
	"Creates a BunnySock node whose traffic and status are printed to stdio\n\n");

	MOOSTrace(
	"USAGE: LibBunnySockTest [OPTIONS] REMOTEHOSTNAME\n\n"
	"OPTIONS:\n");

	MOOSTrace(
	"  -b, --verbosity : Specifies the level of verbosity used when printing\n"
	"                    status events.  Greater values result in more messages\n"
	"                    being printed.\n\n"

	"  -d, --deviceid  : Device ID the node should function as (1..65535).  By\n"
	"                    default, a random device ID is chosen for the node.\n\n"

	"  -h, --help      : Show this help info\n\n"

	"  -m, --mode      : Specifies the mode the BunnySock node should operate\n"
	"                    in: either 'client' or 'server'.  If not specified, the\n"
	"                    mode defaults to 'server'\n\n"

	"  -p, --port      : Specifies the network port (1..65535) used for the\n"
	"                    connection.  For a client node, this sets the port to \n"
	"                    connect to on the remote computer.  For a server node,\n"
	"                    this specifies the port on which incoming connections\n"
	"                    will be accepted.  If not specified, port 20000 is\n"
	"                    used.\n\n"

	"  -r, --retrysec  : Sets the number of seconds that must elapse between\n"
	"                    consecutive attempts by a client node to connect to the\n"
	"                    target remote host.  This value is ignored by server"
	"                    nodes.  If not specified, this time defaults to 2 seconds.\n\n"

	"  -t, --timeout   : Connection timeout in milliseconds.  If this time elapses\n"
	"                    without receiving any packets from a connected peer, the\n"
	"                    connection will be reset.  The default timeout is 2000 ms.\n\n"

	"  -u, --udp       : Causes the BunnySock node to use a connectionless UDP\n"
	"                    Broadcast socket instead of a default TCP socket connection.\n\n"

	"  -v, --version   : Prints version info.\n\n"
	);

	MOOSTrace(
	"REMOTEHOSTNAME:\n"
	"  Remote host name or IP address of the target peer for the node.  For a\n"
	"  client node, this host name specifies the computer the node should\n"
	"  attempt to open a connection with.  For a server node, this host name\n"
	"  specifies a computer to accept connections from (alternatively, 'ANY_IP'\n"
	"  may be specified to allow the server node to accept connections from\n"
	"  any computer.\n\n"
	);
}


//=============================================================================
bool LibBunnySockTest::ParseCommandLine(int argc, const char* argv[])
{
	int ArgIndex;

	if (argc < 2)
	{
		MOOSTrace("Missing one or more command line parameters\n");
		return false;
	}

	if (argc == 2)	// No options specified
	{
		if (argv[1][0] == '-')
		{
			int i = 1;
			ParseOption(argc, argv, i);
			exit(0);
		}
		m_sRemoteHost = argv[1];
	}
	else
	{
		m_sRemoteHost = argv[argc - 1];
		ArgIndex = 1;

		while (ArgIndex < (argc-1))
		{
			// Parse command line options.  Return false on
			// argument or parameter error
			if ( ParseOption(argc, argv, ArgIndex) == false)
			{
				return false;
			}
		}
	}

	return true;
}


//=============================================================================
bool LibBunnySockTest::ParseOption(int argc, const char* argv[], int& ArgIndex)
{
	int i;
	const char* szParam = argv[ArgIndex];
	const char* szParamValue = ((ArgIndex + 1) < (argc - 1)) ?//(((argc - 1) - ArgIndex) > 0) ?
								argv[ArgIndex + 1] : NULL;
	bool ReturnValue = false;
	string sMissingValue = " argument has missing parameter value\n";

	// Validate parameters
	if ( (argv == NULL) || (argc < 1) || (ArgIndex >= argc) )
	{
		return false;
	}

	// Verbosity parameter
	if ( MOOSStrCmp(szParam, "-b") || MOOSStrCmp(szParam, "--verbosity"))
	{
		if ( GetIntParamValue(szParamValue, i) )
		{
			m_Verbosity = (i < 0) ? 0 : i;
			ReturnValue = true;
		}
	}
	else if ( MOOSStrCmp(szParam, "-d") || MOOSStrCmp(szParam, "--deviceid"))
	{
		if ( GetIntParamValue(szParamValue, i) )
		{
			if (i >= 0)
			{
				m_DeviceId = static_cast<uint16_t>(i);
				ReturnValue = true;
			}
		}

	}
	else if ( MOOSStrCmp(szParam, "-h") || MOOSStrCmp(szParam, "--help"))
	{
		PrintUsageInfo();
		exit(0);
	}
	else if ( MOOSStrCmp(szParam, "-m") || MOOSStrCmp(szParam, "--mode"))
	{
		if (szParamValue)
		{
			if ( MOOSStrCmp(szParamValue, "client") || MOOSStrCmp(szParamValue, "server"))
			{
				m_sConnectionMode = szParamValue;
				ReturnValue = true;
			}
		}
	}
	else if ( MOOSStrCmp(szParam, "-p") || MOOSStrCmp(szParam, "--port"))
	{
		if (GetIntParamValue(szParamValue, i) )
		{
			if (i >= 0)
			{
				m_NetworkPort = static_cast<uint16_t>(i);
				ReturnValue = true;
			}
		}
	}
	else if ( MOOSStrCmp(szParam, "-r") || MOOSStrCmp(szParam, "--retrysec"))
	{
		if (GetIntParamValue(szParamValue, i) )
		{
			if (i >= 0)
			{
				m_RetryPeriodSec = static_cast<uint16_t>(i);
				ReturnValue = true;
			}
		}
	}
	else if ( MOOSStrCmp(szParam, "-t") || MOOSStrCmp(szParam, "--timeout"))
	{
		if ( GetIntParamValue(szParamValue, i) )
		{
			if (i >= 1)
			{
				m_ConnectionTimeoutMs = i;
				ReturnValue = true;
			}
		}
	}
	else if ( MOOSStrCmp(szParam, "-u") || MOOSStrCmp(szParam, "--udp"))
	{
		m_sNodeType = "UDP";
		ArgIndex++;
		return true;
	}
	else if ( MOOSStrCmp(szParam, "-v") || MOOSStrCmp(szParam, "--version"))
	{
		MOOSTrace("\nLibBunnySockTest version %3.2f\n"
				   "LibBunnySock version %3.2f\n\n",
				   LIBBUNNYSOCKTEST_VERSION, LIBBUNNYSOCK_VERSION );

		exit(0);
	}
	else
	{
		MOOSTrace("\nUnrecognized option '%s'\n", szParam);
		return false;
	}

	if (ReturnValue == false)
	{
		MOOSTrace("\noption '%s' missing or invalid parameter value\n",
				   szParam);
	}

	ArgIndex += 2;		// Increment argument index
	return ReturnValue;
}



//=============================================================================
bool LibBunnySockTest::GetIntParamValue(const char* sz, int& IntValue ) const
{
	if (sz == NULL)
	{
		return false;
	}
	else
	{
		IntValue = atoi(sz);
		return true;
	}
}

