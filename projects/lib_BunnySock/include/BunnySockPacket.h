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
/** @file BunnySockPacket.h

@brief
	Declaration of a base class for all BunnySock packets and associated
	constants common to all packets

@author Dave Billin

@par Created for
	The Microcomputer Research and Communications Institute (MRCI) - at the
	University of Idaho, USA.
*/
//=============================================================================

#ifndef _BUNNYSOCK_H_
#define _BUNNYSOCK_H_

#include <string>
#include <cstring>
#include <stdint.h>




namespace BunnySock
{

//==========================
// CONSTANTS
//==========================

/** @def BUNNYSOCK_PACKET_SIZE
@brief
	The number of Bytes in a BunnySock packet
*/
#ifndef BUNNYSOCK_PACKET_SIZE
	#define BUNNYSOCK_PACKET_SIZE 	128
#endif


/** @def PACKETTYPE_BUNNYSOCK_KEEPALIVE
 * @brief
 * 	Packet type used for BunnySock keep-alive packets
 */
#define PACKETTYPE_BUNNYSOCK_KEEPALIVE	999


//==========================
// DATA TYPES
//==========================

#pragma pack(push, 1)	// This should work for gcc and Visual Studio 2010+


//-----------------------------------------------------------
/** @typedef bsock_PacketHeader_t
 * @brief
 * 	All BunnySock packets begin with this header structure
 */
typedef struct
{
	uint16_t PacketType;	/**< Identifies the type of packet */
	uint16_t SourceId;		/**< Device ID of the packet's sender */
	uint16_t DestId;		/**< Device ID of the packet's intended
								 destination */
	uint16_t Reserved;		/**< Reserved for future use */
	uint32_t MsTimeStamp;	/**< Millisecond time stamp */
} bsock_PacketHeader_t;



//-----------------------------------------------------------
/** @typedef bsockGenericPacket_t
 * The general structure of a Bunnysock packet: a header
 * followed by payload data
 */
typedef struct
{
	bsock_PacketHeader_t Header;	/**< The packet's header */
	char Payload[BUNNYSOCK_PACKET_SIZE -
	             sizeof(bsock_PacketHeader_t) ];	/**< Raw payload Bytes */
} bsockGenericPacket_t;

#pragma pack(pop)	// This should work for gcc and Visual Studio 2010+


//=============================================================================
/** Base class used to represent a generic BunnySock packet */
//=============================================================================
class BunnySockPacket
{
public:

	//=========================================================================
	/** Creates an empty packet and assigns it a packet type of -1 and
	 * Source/Destination ID's of zero
	 */
	BunnySockPacket( void );

	//=========================================================================
	/** Creates a BunnySockPacket object from raw data Bytes
	 * @param[in] pSourceData
	 *	A pointer to the raw data Bytes used to populate the packet
	 * @param[in] NumBytes
	 *	The number of data Bytes pointed to by pSourceData.  Must be less than
	 *	or equal to the size of a BunnySock packet.
	*/
	BunnySockPacket( const void* pSourceData, const unsigned int NumBytes );


	//=========================================================================
	/** Creates a BunnySockPacket object from a string of hexadecimal Byte values
	 * @param [in] sHexByteString
	 * 	A std::string containing hexadecimal-encoded Byte values for of packet
	 *  Bytes.  */
	BunnySockPacket( const std::string& sHexByteString );


	virtual ~BunnySockPacket( void );


	//=========================================================================
	/** Sets the fields in the packet header */
	void SetHeader( uint16_t PacketType, uint16_t SourceId, uint16_t DestId,
					uint32_t MsTimeStamp );


	//=========================================================================
	/** Returns a pointer to the packet's header */
	inline bsock_PacketHeader_t* GetHeader( void )
	{ 	return &m_PacketData.Header; }


	//=========================================================================
	/** Returns a pointer to the raw packet Bytes */
	inline void* GetRawBytes( void )
	{ 	return &m_PacketData; }


	//=========================================================================
	/** Serializes the raw packet data as a string of hexadecimal Byte values */
	std::string ToHexString( void );

	//=========================================================================
	/** Loads the packet's contents from a hex-encoded character string
	 * @return true if the string was loaded; else false on error
	 *
	 * @throw A CMOOSException for badly formatted input string
	 */
	void LoadFromHexString( const std::string& sSource );

	//=========================================================================
	/** Prints the contents of the packet to a string in a human-readable
	 * form */
	virtual std::string Print( void );

	//=========================================================================
	/** Initializes the contents of the packet with a specified value
	 * @param FillValue
	 * 	Value assigned to all packet Bytes
	 */
	inline void Clear( uint8_t FillValue = 0 )
	{	memset(&m_PacketData, FillValue, sizeof(bsockGenericPacket_t)); }


	//=========================================================================
	/** Assignment operator */
	BunnySockPacket& operator=(const BunnySockPacket& SrcObj );

protected:
	bsockGenericPacket_t m_PacketData;	/**< The raw packet data */
};


};	// END namespace BunnySock

#endif	// END #ifndef _BUNNYSOCKPACKETS_H_
