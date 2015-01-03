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
/** @file BunnySockPacket.cpp

@brief
	Definitions of methods and attributes of the BunnySockPacket class

@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the
	University of Idaho, USA.
*/
//=============================================================================
#include <iostream>
#include <sstream>
#include <iomanip>
#include "MOOS/libMOOS/Utils/MOOSException.h"
#include "MOOS/libMOOS/Utils/MOOSUtilityFunctions.h"

#include "BunnySockPacket.h"

using namespace::std;
using namespace::BunnySock;




//=============================================================================
BunnySockPacket::BunnySockPacket( void )
{
	Clear();	// Clear the contents of the packet
}


//=============================================================================
BunnySockPacket::BunnySockPacket( const void* pSourceData,
								  const unsigned int NumBytes )
{
	uint32_t NumBytesToCopy = (NumBytes > BUNNYSOCK_PACKET_SIZE) ?
										BUNNYSOCK_PACKET_SIZE : NumBytes;
	memcpy( &m_PacketData, pSourceData, NumBytesToCopy );
}


//=============================================================================
BunnySockPacket::BunnySockPacket( const std::string& sHexByteString )
{
	memset(&m_PacketData, 0, BUNNYSOCK_PACKET_SIZE);

	// Attempt to load the packet's contents from the hex string
	LoadFromHexString(sHexByteString);
}


//=============================================================================
BunnySockPacket::~BunnySockPacket( void )
{
	// Nothing to do here
}



//=============================================================================
void BunnySockPacket::SetHeader( uint16_t PacketType, uint16_t SourceId,
								 uint16_t DestId, uint32_t MsTimeStamp )
{
	bsock_PacketHeader_t* pHeader = &m_PacketData.Header;

	pHeader->PacketType = PacketType;
	pHeader->SourceId = SourceId;
	pHeader->DestId = DestId;
	pHeader->MsTimeStamp = MsTimeStamp;
}



//=============================================================================
std::string BunnySockPacket::ToHexString( void )
{
	static const char* const szHexTable = "0123456789abcdef";
	char szHexChars[BUNNYSOCK_PACKET_SIZE * 2 + 1];
	char* pHexChar = szHexChars;
	char* pData = (char*)&m_PacketData;

    for (int i = 0; i < BUNNYSOCK_PACKET_SIZE; ++i)
    {
        *pHexChar++ = szHexTable[((*pData) >> 4) & 0xf];	// Upper nibble
        *pHexChar++ = szHexTable [(*pData) & 0xf];	// Lower nibble
        ++pData;
    }

    szHexChars[BUNNYSOCK_PACKET_SIZE * 2] = '\0';
    return std::string(szHexChars);
}



//=============================================================================
void BunnySockPacket::LoadFromHexString( const std::string& sSource )
{
	uint8_t* pDestByte = (uint8_t*)&m_PacketData;
	uint32_t ByteCount;
	std::string s, ByteString;

	// Do nothing if the source string is empty
	if ( sSource.empty() )
	{
		return;
	}

	// If the source string has an odd number of characters, prepend a '0'
	s = ( (sSource.length() & 1) != 0 ) ? ("0" + sSource) : sSource;

	memset(&m_PacketData, 0, BUNNYSOCK_PACKET_SIZE);	// Clear packet data

	ByteCount = 0;
	for ( string::iterator iter = s.begin();
		  iter != s.end(); iter++ )
	{
		std::stringstream ss( s.substr(2*ByteCount, 2) );
		ss >> std::hex >> *pDestByte;
		iter += 2;
	}
}



//=============================================================================
std::string BunnySockPacket::Print( void )
{
	bsock_PacketHeader_t* pHeader = &m_PacketData.Header;
	string s;

	s = MOOSFormat(	"<< Generic Packet Contents >>\n"
					"Type: %d\n"
					"Source: %d\n"
					"Destination: %d\n"
					"TimeStamp: %08x\n",
					pHeader->PacketType, pHeader->SourceId, pHeader->DestId,
					pHeader->MsTimeStamp );

	s = s + "Payload:\n" + ToHexString() + "\n\n";

	return s;
}


//=============================================================================
BunnySockPacket& BunnySockPacket::operator=(const BunnySockPacket& SrcObj )
{
	m_PacketData = SrcObj.m_PacketData;
	return *this;
}
