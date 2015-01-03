//=============================================================================
/** @file UAVNetPacket.h
 *
 * @brief Declaration of the UAVNetPacket class
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef _UAVNETPACKET_H_
#define _UAVNETPACKET_H_

#include <istream>
#include <string>
#include "UAV_PACKET_TYPES.h"


namespace UAVnet
{

class UAVNetPacket
{
public:

	//=========================================================================
	/** Default constructor.  Creates an empty packet of type TYPE_GENERIC and
	 * SourceId = DestId = 0
	 */
	UAVNetPacket( void );


	//=========================================================================
	/** Creates a UAVNetPacket object by reading data from a binary input
	 * stream
	 *
	 * @throw
	 *	A std::exception if InputStream is bad or eof.
	 */
	UAVNetPacket( std::istream& InputStream );


	//=========================================================================
	/** Copy constructor */
	UAVNetPacket( const UAVNetPacket& SourceObj );


	//=========================================================================
	const PacketHeader_t* Header( void ) const;

	//=========================================================================
	void FromRawBytes( void* pRawPacketBytes );


	//=========================================================================
	/** Returns a pointer to packet data as a generic packet */
	const GenericPacket_t* AsGenericPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a sensor packet */
	const SensorPacket_t* AsSensorPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a controls packet */
	const ControlsPacket_t* AsControlsPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a command packet */
	const CommandPacket_t* AsCommandPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as an LBL estimated position packet */
	const LblEstPositionPacket_t* AsLblEstPositionPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a WHOI message packet */
	const WhoiMsgPacket_t* AsWhoiMsgPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a GPS GPGGA message packet */
	const GpsGPGGAPacket_t* AsGpsGPGGAPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as an LBL beacon location packet */
	const LblBeaconLocationPacket_t* AsLblBeaconLocationPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a vehicle EKF packet */
	const VehicleEKFPacket_t* AsVehicleEKFPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a ship EKF packet */
	const ShipEKFPacket_t* AsShipEKFPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a depth packet */
	const DepthPacket_t* AsDepthPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as an IMU message packet */
	const ImuPacket_t* AsImuPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a single-message IMU packet */
	const SingleImuPacket_t* AsSingleImuPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a DSP telemetry packet */
	const DspTelemetryPacket_t* AsDspTelemetryPacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as a synchronous range packet */
	const SynchronousRangePacket_t* AsSynchronousRangePacket(void) const;

	//=========================================================================
	/** Returns a pointer to packet data as raw Bytes */
	const void* AsRawBytes(void) const;


	//=========================================================================
	/** Overload of comparison "<" operator that compares packet time stamps */
	bool operator<(const UAVNetPacket& Rhs) const;

	//=========================================================================
	/** Overload of comparison ">" operator that compares packet time stamps */
	bool operator>(const UAVNetPacket& Rhs) const;

	//=========================================================================
	/** Overload of comparison "==" operator that compares packet time stamps */
	bool operator==(const UAVNetPacket& Rhs) const;

	//=========================================================================
	/** Overload of assignment operator for copying packets */
	const UAVNetPacket& operator=(const UAVNetPacket& Rhs);

	//=========================================================================
	friend std::istream& operator>>( std::istream& IStream,
	                                 UAVNetPacket& Packet );


	//===========================================
	// Exception class thrown when stream-based
	// operations on a UAVNet packet fail
	//===========================================
	class UAVNetPacketException : public std::exception
	{
		const char* what() const throw()
		{
			return "Input stream is bad or exhausted";
		}
	};

	// Exception thrown for bad input stream
	static const UAVNetPacketException BadStreamException;

private:
	GenericPacket_t m_Packet;
};






//=============================================================================
/** Stream extraction operator overload: allows a UAVNetPacket object to be
 * read from an input stream.
 *
 * @param IStream
 *	The input stream to read from
 *
 * @param Packet
 *	The packet to read into
 *
 * @throw
 *	A std::exception if the packet could not be read because IStream is bad or
 *	eof.
 */
std::istream& operator>>( std::istream& IStream, UAVNetPacket& Packet );
/*
{

	if ( IStream.eof() || IStream.bad() )
	{
		throw(UAVNetPacket::BadStreamException);
	}
	else
	{
		try
		{
			IStream.read(reinterpret_cast<char*>(&Packet.m_Packet),
						 UAVNET_PACKET_SIZE );
		}
		catch (ios::failure& e) {;}

		if (IStream.gcount() < UAVNET_PACKET_SIZE)
		{
			throw(UAVNetPacket::BadStreamException);
		}
	}

	return IStream; // Enables Istream >> Packet1 >> Packet2 >> Packet3;
}
*/


};  // END namespace UAVnet

#endif 	// END #ifndef _UAVNETPACKET_H_
