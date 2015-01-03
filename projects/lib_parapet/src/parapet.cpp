//=============================================================================
/** @file parapet.cpp
 *
 * @brief
 *	Implementation of static functions defined in parapet.h
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#include <cstring>
#include <cassert>

#include "parapet.h"

using namespace std;

namespace parapet
{

//=============================================================================
bool HasPayload( parapet_packet_t const& Packet )
{
    parapet_header_t const& Header = Packet.Header;
    bool HasPayload = true;

    // CONTINUE packets never have a payload
    // NOTE: this handles all request packets - only response packets
    //       need be considered from here

    uint8_t Opcode = (Header.u32 >> HEADER_BIT_OPCODE) & 0x03;
    if ( Opcode == CONTINUE )
    {
        HasPayload = false;
    }
    else
    {
        if (Header.fields.M == 0)
        {
            // STATUS: Packet is a response

            if ( Header.fields.A == 1 )
            {
                // STATUS: Packet is a response with an ACK
                if ( Opcode == WRITE_PARAM )
                {
                    // Write responses with an ACK have no payload
                    HasPayload = false;
                }
            }
            else
            {
                // NACK responses always have no payload
                HasPayload = false;
            }
        }
    }

    return HasPayload;
}


//=============================================================================
uint16_t PacketSize( parapet_packet_t const& Packet )
{
    uint32_t NumBytes = sizeof(parapet_header_t);
    parapet_header_t const& Header = Packet.Header;

    if ( HasPayload(Packet) == true )
    {
        NumBytes += Header.fields.NumBytesToFollow + 1;
    }

    return NumBytes;
}




//=============================================================================
parapet_header_t& InitRequestHeader( parapet_header_t& Header,
                                     uint8_t DeviceID,
                                     e_OpcodeIDs Opcode,
                                     uint8_t RequestID )
{

    Header.u32 = 0;     // Zero out header fields
    Header.u32 = parapet_header_t::SYNC_BYTE |
                 ( ((uint8_t)Opcode & 0x03) << HEADER_BIT_OPCODE ) |
                 ( 1 << HEADER_BIT_M ) |
                 ( (DeviceID & 0x07) << HEADER_BIT_DEVICEID ) |
                 ( (RequestID & 0x1f) << HEADER_BIT_REQUESTID );

    /*
    Header.fields.Sync = parapet_header_t::SYNC_BYTE;
    Header.fields.Opcode = Opcode;
    Header.fields.M = M;
    Header.fields.DeviceID = DeviceID;
    Header.fields.RequestID = RequestID;
    */
    return Header;
}




//=============================================================================
void BuildExecuteRequest( parapet_packet_t& Packet,
                          uint8_t DeviceID,
                          uint8_t RequestID,
                          uint32_t FunctionID,
                          parapet_param_t* const Params,
                          uint16_t NumParams )
{
    parapet::execute_request_t& ExecPacket = Packet.ExecuteRequest;

    InitRequestHeader( ExecPacket.Header, DeviceID, parapet::EXECUTE,
                       RequestID );

    parapet_header_t& Header = ExecPacket.Header;
    Header.fields.NumBytesToFollow = sizeof(uint32_t) - 1;
    ExecPacket.FunctionID = FunctionID;

    if ( NumParams != 0 )
    {
        // Copy in param data and populate NumBytesToFollow
        // NOTE: Params pointer and NumParams are validated inside AddParam()
        AddParam( Packet, Params, NumParams );
    }
    else
    {
        Header.fields.NumBytesToFollow = sizeof(uint32_t) - 1;
    }

}



//=============================================================================
void BuildReadRequest( parapet_packet_t& Packet,
                       uint8_t DeviceID,
                       uint8_t RequestID,
                       uint16_t const* ParamIDs,
                       uint16_t NumParamIDs )
{
    parapet::read_request_t& ReadPacket = Packet.ReadRequest;
    parapet_header_t& Header = InitRequestHeader( ReadPacket.Header,
                                                  DeviceID,
                                                  parapet::READ_PARAM,
                                                  RequestID );

    // Bound max number of param ID's
    if (read_request_t::MAX_NUM_PARAM_IDS < NumParamIDs)
    {
        NumParamIDs =  read_request_t::MAX_NUM_PARAM_IDS;
    }

    // Read requests must have at least one param ID
    assert( NumParamIDs > 0 );
    assert( ParamIDs != NULL );

    // Copy in param ID data and populate NumBytesToFollow
    uint32_t NumParamIDBytes = NumParamIDs * sizeof(uint16_t);
    memcpy( ReadPacket.ParamID, ParamIDs, NumParamIDBytes );

    Header.u32 &= ~(0x7ff << HEADER_BIT_NUMBYTESTOFOLLOW);
    Header.u32 |= ((NumParamIDBytes - 1) << HEADER_BIT_NUMBYTESTOFOLLOW);

    //Header.fields.NumBytesToFollow = NumParamIDBytes;
}





//=============================================================================
void BuildWriteRequest( parapet_packet_t& Packet, uint8_t DeviceID,
                        uint8_t RequestID, parapet_param_t* const Params,
                        uint16_t NumParams )
{
    InitRequestHeader( Packet.WriteRequest.Header, DeviceID,
                       parapet::WRITE_PARAM, RequestID );

    // Copy in param data and populate NumBytesToFollow
    // NOTE: Params pointer and NumParams are validated inside AddParam()
    AddParam( Packet, Params, NumParams );
}




//=============================================================================
void BuildContinueRequest( parapet_packet_t& Packet, uint8_t DeviceID,
                           uint8_t RequestID )
{
    // Populate header
    InitRequestHeader( Packet.ContinueRequest.Header, DeviceID,
                       parapet::CONTINUE, RequestID );

    // NOTE: continue packets have no payload
}







//=============================================================================
void InitResponseHeader( parapet_packet_t& ResponsePacket,
                         parapet_packet_t const& Request,
                         bool IsAck,
                         bool IsLastPacket )
{
    parapet_header_t& ResponseHeader = ResponsePacket.ShortResponse.Header;

    // Parapet slave devices must never, never, ever respond to
    // a request packet with a DeviceID of zero (broadcast)!
    //assert( Request.ContinueRequest.Header.fields.DeviceID != 0 );
    assert( Request.Header.fields.DeviceID != 0);

    // Copy header fields from the packet being responded to
    ResponseHeader.u32 = Request.ContinueRequest.Header.u32;

    // Clear M, A, and NumBytesToFollow fields in header
    ResponseHeader.u32 &= ~( (1 << HEADER_BIT_M) |
                             (1 << HEADER_BIT_A) |
                             (1 << HEADER_BIT_L) |
                             (0x7ffUL << HEADER_BIT_NUMBYTESTOFOLLOW) );

    if ( IsAck )
    {
        ResponseHeader.u32 |= (1 << HEADER_BIT_A);
    }

    // NOTE: L flag should always be set for a NACK
    if ( IsLastPacket || (IsAck == false) )
    {
        ResponseHeader.u32 |= (1 << HEADER_BIT_L);
    }


    //ResponseHeader.fields.M = 0;
    //ResponseHeader.fields.A = (IsAck == true) ? 1 : 0;
    //ResponseHeader.fields.L = (IsLastPacket == true) ? 1 : 0;
    //ResponseHeader.fields.NumBytesToFollow = 0;
}




//=============================================================================
void BuildShortResponse( parapet_packet_t& ResponsePacket,
                         parapet_packet_t const& Request,
                         bool IsAck )
{
    // Short responses should only be sent in response to a WRITE_PARAM
    // request, or a CONTINUE request where no data is available to continue
    assert( (Request.Header.fields.Opcode == CONTINUE) ||
            (Request.Header.fields.Opcode == WRITE_PARAM) );

    InitResponseHeader( ResponsePacket, Request, IsAck, true );
}





//=============================================================================
void BuildExecuteResponse( parapet_packet_t& ResponsePacket,
                           parapet_packet_t const& Request,
                           bool IsAck, uint32_t ResultCode )
{
    assert( Request.ExecuteRequest.Header.fields.Opcode == EXECUTE );

    InitResponseHeader( ResponsePacket, Request, IsAck, true );
    execute_response_t& ExecResponse = ResponsePacket.ExecuteResponse;
    //ResponsePacket.ExecuteResponse.Header.fields.NumBytesToFollow =
    //                                                    sizeof(uint32_t) - 1;
    parapet_header_t& Header = ResponsePacket.ExecuteResponse.Header;
    Header.u32 &= ~(0x7ff << HEADER_BIT_NUMBYTESTOFOLLOW);
    Header.u32 |= ((sizeof(uint32_t) - 1) << HEADER_BIT_NUMBYTESTOFOLLOW);

    // Only populate the result code if packet contains an ACK
    if ( IsAck )
    {
        ExecResponse.ResultCode = ResultCode;
    }
}





//=============================================================================
void BuildReadResponse( parapet_packet_t& ResponsePacket,
                        parapet_packet_t const& Request,
                        bool IsAck,
                        bool IsLastPacket,
                        parapet_param_t* const Params,
                        uint16_t NumParams )
{
    // Init response header, including A and L flags
    InitResponseHeader( ResponsePacket, Request, IsAck, IsLastPacket );

    if ( IsAck )
    {
        // Copy in param data and populate NumBytesToFollow
        // NOTE: Params pointer and NumParams are validated inside AddParam()
        AddParam( ResponsePacket, Params, NumParams );
    }
    else
    {
        // If the response is a NACK, zero NumBytesToFollow
        ResponsePacket.ReadResponse.Header.fields.NumBytesToFollow = 0;
    }

}





//=============================================================================
uint16_t NumParams( parapet_packet_t const& Packet )
{
    parapet_header_t const& Header = Packet.Header;
    uint16_t ParamCount = 0;
    bool PacketHasParameters = true;
    uint8_t Opcode = Header.fields.Opcode;

    // Determine whether the packet can have parameters
    if ( Header.fields.M == 1 )
    {
        // Read requests don't have parameters
        PacketHasParameters = (Opcode != READ_PARAM);
    }
    else
    {
        // Only Read responses with an ACK have parameters
        if ( (Header.fields.A == 0) || (Opcode != READ_PARAM) )
        {
            PacketHasParameters = false;
        }
    }

    if ( PacketHasParameters == true )
    {
        // Obtain the number of parameter Bytes in the packet payload
        ParamCount = Header.fields.NumBytesToFollow + 1;

        if ( Header.fields.Opcode == EXECUTE )
        {
            ParamCount -= sizeof(uint32_t);
        }

        // Divide param Bytes by 8 to obtain number of params
        //ParamCount /= sizeof(parapet_param_t);
        ParamCount >>= 3;
    }

    return ParamCount;
}




//=============================================================================
uint16_t AddParam( parapet_packet_t& Packet, parapet_param_t* const Params,
                   uint16_t NumParamsToAdd )
{
    parapet_header_t& Header = Packet.Header;
    uint16_t MaxNumParams = 0;
    parapet_param_t* p = NULL;

    assert( Params != NULL );   // Validate pointer
    assert( NumParamsToAdd > 0 );    // Cannot add zero parameters!

    uint8_t Opcode = (Header.u32 >> HEADER_BIT_OPCODE) & 0x03;

    //if ( Header.fields.M == 1 )
    if ( (Header.u32 & (1 << HEADER_BIT_M)) != 0 )
    {
        // Process a request packet
        switch (Opcode)
        //switch (Header.fields.Opcode)
        {
            case parapet::EXECUTE:
            {
                p = Packet.ExecuteRequest.Param;
                MaxNumParams = execute_request_t::MAX_NUM_PARAMS;
                break;
            }

            case parapet::WRITE_PARAM:
            {
                p = Packet.WriteRequest.Param;
                MaxNumParams = write_request_t::MAX_NUM_PARAMS;
                break;
            }

            default:
            {
                // CONTINUE and READ requests have no params!!
                return 0;
                break;
            }
        }
    }
    else
    {
        // Process a response packet
        if ( Header.fields.Opcode == parapet::READ_PARAM )
        {
            if ( Header.fields.A == 1 )
            {
                p = Packet.ReadResponse.Param;
                MaxNumParams = read_response_t::MAX_NUM_PARAMS;
            }
            else
            {
                return 0;   // Don't add params to a NACK response!
            }

        }
        else
        {
            return 0;   // Only READ_PARAM responses have params!
        }
    }


    // Adjust the max number of parameters available and the pointer p
    // based on the number of parameters currently in the packet
    uint16_t NumParamsInUse = NumParams(Packet);
    MaxNumParams -= NumParamsInUse;
    p += NumParamsInUse;

    // Saturate the number of parameters that may be written to the packet
    if ( MaxNumParams < NumParamsToAdd )
    {
        NumParamsToAdd = MaxNumParams;
    }

    // Copy in parameter data
    uint32_t NumParamBytes = NumParamsToAdd * sizeof(parapet_param_t);
    memcpy( p, Params, NumParamBytes );

    // Update the packet's NumBytesToFollow field
    NumParamBytes += NumParamsInUse * sizeof(parapet_param_t);

    // Must account for the extra uint32_t in EXECUTE requests
    if ( Opcode == EXECUTE )
    {
        NumParamBytes += sizeof(uint32_t);
    }

    //Header.fields.NumBytesToFollow += NumParamBytes;
    Header.u32 &= ~(0x7ff << HEADER_BIT_NUMBYTESTOFOLLOW);
    Header.u32 |= ((NumParamBytes - 1) << HEADER_BIT_NUMBYTESTOFOLLOW);

    return NumParamsToAdd;
}




//=============================================================================
uint16_t NumParamID( parapet_packet_t const& Packet )
{
    parapet_header_t const& Header = Packet.Header;
    uint16_t ParamIDCount = 0;

    // This function only applies to READ_PARAM requests
    if ( (Header.fields.Opcode == READ_PARAM) &&
         (Header.fields.M == 1) )
    {
        ParamIDCount = Header.fields.NumBytesToFollow + 1;
        //ParamIDCount /= sizeof(uint16_t);
        ParamIDCount >>= 1;     // Divide by 2 to obtain number of Param ID's
    }

    return ParamIDCount;
}




//=============================================================================
uint16_t AddParamID( parapet_packet_t& Packet, uint16_t const* ParamIDs,
                     uint16_t NumParamIDs )
{
    parapet_header_t& Header = Packet.Header;
    uint16_t NumIDsToAdd = 0;

    // This function only applies to READ_PARAM requests
    if ( (Header.fields.Opcode == READ_PARAM) &&
         (Header.fields.M == 1) )
    {
        // Determine the maximum number of ID's that can be added to the
        // packet
        uint16_t NumIDsInUse = NumParamID(Packet);
        uint16_t MaxIDs = read_request_t::MAX_NUM_PARAM_IDS - NumIDsInUse;

        NumIDsToAdd = (MaxIDs < NumParamIDs) ? MaxIDs : NumParamIDs;

        // Copy in the ID's
        memcpy( Packet.ReadRequest.ParamID + NumIDsInUse,
                ParamIDs,
                (NumIDsToAdd << 1) ); // Multiply by 2 to get # ParamID Bytes

        // Adjust the NumBytesToFollow field
        // Multiply by 2 to get Number of ParamID Bytes
        Header.fields.NumBytesToFollow =
                            ((NumIDsInUse + NumIDsToAdd) << 1) - 1;
    }

    return NumIDsToAdd;
}


}   // END namespace parapet
