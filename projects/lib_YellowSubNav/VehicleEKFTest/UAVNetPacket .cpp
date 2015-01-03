//=============================================================================
/** @file UAVnetPacket.cpp
 *
 * @brief Definition of the UAVNetPacket class
 *
 * @author Dave Billin
 */
//=============================================================================

#include <string.h>
#include "UAVNetPacket.h"

using namespace std;
using namespace UAVnet;

const UAVNetPacket::UAVNetPacketException UAVNetPacket::BadStreamException;


//=============================================================================
UAVNetPacket::UAVNetPacket( void )
{
	memset(&m_Packet, 0, sizeof(GenericPacket_t));
	m_Packet.Header.Type = TYPE_GENERIC;
}




//=============================================================================
UAVNetPacket::UAVNetPacket( std::istream& InputStream )
{
	if ( InputStream.eof() || InputStream.bad() )
	{
		throw(BadStreamException);
	}
	else
	{
		InputStream.read(reinterpret_cast<char*>(&m_Packet),
						 UAVNET_PACKET_SIZE );
	}
}




//=============================================================================
UAVNetPacket::UAVNetPacket( const UAVNetPacket& SourceObj )
{
	memcpy(&m_Packet, &SourceObj.m_Packet, UAVNET_PACKET_SIZE);
}




//=============================================================================
const PacketHeader_t* UAVNetPacket::Header( void ) const
{
	return reinterpret_cast<const PacketHeader_t*>(&m_Packet);
}



//=============================================================================
void UAVNetPacket::FromRawBytes( void* pRawPacketBytes )
{
	if (pRawPacketBytes != NULL)
	{
		memcpy(&m_Packet, pRawPacketBytes, UAVNET_PACKET_SIZE);
	}
}



//=============================================================================
const GenericPacket_t* UAVNetPacket::AsGenericPacket(void) const
{ 
    return reinterpret_cast<const GenericPacket_t*>(&m_Packet); 
}



//=============================================================================
const SensorPacket_t* UAVNetPacket::AsSensorPacket(void) const
{ 
    return reinterpret_cast<const SensorPacket_t*>(&m_Packet); 
}



//=============================================================================
const ControlsPacket_t* UAVNetPacket::AsControlsPacket(void) const
{ 
    return reinterpret_cast<const ControlsPacket_t*>(&m_Packet); 
}



//=============================================================================
const CommandPacket_t* UAVNetPacket::AsCommandPacket(void) const
{ 
    return reinterpret_cast<const CommandPacket_t*>(&m_Packet); 
}



//=============================================================================
const LblEstPositionPacket_t* UAVNetPacket::AsLblEstPositionPacket(void) const
{ 
    return reinterpret_cast<const LblEstPositionPacket_t*>(&m_Packet); 
}



//=============================================================================
const WhoiMsgPacket_t* UAVNetPacket::AsWhoiMsgPacket(void) const
{ 
    return reinterpret_cast<const WhoiMsgPacket_t*>(&m_Packet); 
}



//=============================================================================
const GpsGPGGAPacket_t* UAVNetPacket::AsGpsGPGGAPacket(void) const
{ 
    return reinterpret_cast<const GpsGPGGAPacket_t*>(&m_Packet); 
}


//=============================================================================
const LblBeaconLocationPacket_t* UAVNetPacket::AsLblBeaconLocationPacket(void) const
{ 
    return reinterpret_cast<const LblBeaconLocationPacket_t*>(&m_Packet); 
}


//=============================================================================
const VehicleEKFPacket_t* UAVNetPacket::AsVehicleEKFPacket(void) const
{ 
    return reinterpret_cast<const VehicleEKFPacket_t*>(&m_Packet); 
}


//=============================================================================
const ShipEKFPacket_t* UAVNetPacket::AsShipEKFPacket(void) const
{ 
    return reinterpret_cast<const ShipEKFPacket_t*>(&m_Packet); 
}


//=============================================================================
const DepthPacket_t* UAVNetPacket::AsDepthPacket(void) const
{ 
    return reinterpret_cast<const DepthPacket_t*>(&m_Packet); 
}


//=============================================================================
const ImuPacket_t* UAVNetPacket::AsImuPacket(void) const
{ 
    return reinterpret_cast<const ImuPacket_t*>(&m_Packet); 
}


//=============================================================================
const SingleImuPacket_t* UAVNetPacket::AsSingleImuPacket(void) const
{ 
    return reinterpret_cast<const SingleImuPacket_t*>(&m_Packet); 
}


//=============================================================================
const DspTelemetryPacket_t* UAVNetPacket::AsDspTelemetryPacket(void) const
{ 
    return reinterpret_cast<const DspTelemetryPacket_t*>(&m_Packet); 
}


//=============================================================================
const SynchronousRangePacket_t* UAVNetPacket::AsSynchronousRangePacket(void) 
const
{ 
    return reinterpret_cast<const SynchronousRangePacket_t*>(&m_Packet); 
}


//=============================================================================
const void* UAVNetPacket::AsRawBytes(void) const
{ 
    return reinterpret_cast<const void*>(&m_Packet); 
}


//=============================================================================
bool UAVNetPacket::operator<(const UAVNetPacket& Rhs) const
{ 
    return m_Packet.Header.MsTimeStamp > Rhs.m_Packet.Header.MsTimeStamp; 
}


//=============================================================================
bool UAVNetPacket::operator>(const UAVNetPacket& Rhs) const
{ 
    return m_Packet.Header.MsTimeStamp < Rhs.m_Packet.Header.MsTimeStamp; 
}


//=============================================================================
bool UAVNetPacket::operator==(const UAVNetPacket& Rhs) const
{ 
    return m_Packet.Header.MsTimeStamp == Rhs.m_Packet.Header.MsTimeStamp; 
}


//=============================================================================
const UAVNetPacket& UAVNetPacket::operator=(const UAVNetPacket& Rhs)
{ 
    memcpy(&m_Packet, &Rhs.m_Packet, UAVNET_PACKET_SIZE); return *this; 
}




//=============================================================================
istream& UAVnet::operator>>( std::istream& IStream, UAVNetPacket& Packet )
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





