//=============================================================================
/** @file ParapetSlave.cpp
 *
 * @brief
 *  Implementation of API functions used to implement a parapet Slave device
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================
#include <string.h>
#include <assert.h>

#include "parapet_node.h"
#include "parapet_callbacks.h"

typedef enum
{
    INVALID_HEADER = -1,
    HEADER_NO_PAYLOAD = 0,
    HEADER_WITH_PAYLOAD = 1
} header_status_t;

//---------------------------------------
// Local (private) function prototypes
//---------------------------------------
int ProcessRxFSMState( parapet_node_t* Node );
header_status_t ProcessRxHeader( parapet_node_t* Node );
void ProcessRxPacket( parapet_node_t* Node );



//=============================================================================
void parapet_node_init( parapet_node_t* Node, uint8_t DeviceID )
{
    // Initialize node fields
    memset( Node, 0, sizeof(parapet_node_t) );

    // Assign initial node parameters
    Node->DeviceID = DeviceID;
    Node->RxState = RX_HEADER;
    Node->NumRxBytesNeeded = sizeof(parapet_header_t);
}



//=============================================================================
void parapet_reset_node( parapet_node_t* Node )
{
    // Assign initial node parameters
    memset( &Node->RxPacket, 0, sizeof(parapet_packet_t) );
    Node->RxState = RX_HEADER;
    Node->NumRxBytesNeeded = sizeof(parapet_header_t);
}




//=============================================================================
void parapet_SetRxHandler( parapet_node_t* Node, parapet_opcode_t Opcode,
                           parapet_rx_callback_fn RxCallback )
{
    Node->RxCallback[Opcode] = RxCallback;
}


//=============================================================================
void parapet_SetEventHandler( parapet_node_t* Node,
                              parapet_event_callback_fn EventCallback )
{
    Node->EventCallback = EventCallback;
}


//=============================================================================
void parapet_enable_promiscuous_mode( parapet_node_t* Node,
                                      uint8_t PromiscuityShouldBeEnabled )
{
    Node->PromiscuousModeIsEnabled = PromiscuityShouldBeEnabled;
}



//=============================================================================
uint8_t parapet_promiscuous_mode_is_enabled( parapet_node_t* Node )
{
    return Node->PromiscuousModeIsEnabled;
}




//=============================================================================
uint8_t parapet_process_node( parapet_node_t* Node, char* RxData,
                         uint16_t NumBytesToProcess )
{
    rx_state_t NextState = Node->RxState;
    parapet_packet_t* RxPacket = &Node->RxPacket;

    uint8_t retval = 0;

    while ( NumBytesToProcess > 0 )
    {
        // Copy the required number of Bytes into the Rx packet buffer
        uint16_t NumBytesToCopy =
                ( Node->NumRxBytesNeeded < NumBytesToProcess ) ?
                            Node->NumRxBytesNeeded : NumBytesToProcess;

        memcpy( &Node->RxPacket.u8[ Node->NumBytesReceived ],
                RxData,
                NumBytesToCopy );

        RxData += NumBytesToCopy;
        Node->NumBytesReceived += NumBytesToCopy;
        Node->NumRxBytesNeeded -= NumBytesToCopy;
        NumBytesToProcess -= NumBytesToCopy;

        if ( Node->NumRxBytesNeeded == 0 )
        {
            NextState = ProcessRxFSMState( Node );   // Get the next state
            Node->RxState = NextState;
        }

    }
    return retval;
}



//=============================================================================
/// Implements state transition logic used for the Rx state machine
/// @param [in,out] Node    Node to process
/// @return     The Rx FSM state following state transition logic processing
int ProcessRxFSMState( parapet_node_t* Node )
{
    rx_state_t NextState = Node->RxState;

    // Implement FSM state transition logic
    switch ( Node->RxState )
    {
        //-------------------------------------------------
        // STATUS:
        //   A 4-Byte header has been received.  Validate
        //   and (if it has no payload) process it.
        //-------------------------------------------------
        case RX_HEADER:
        {
            header_status_t HeaderStatus = ProcessRxHeader( Node );

            if ( HeaderStatus == HEADER_WITH_PAYLOAD )
            {
                // Header is valid and packet has payload data
                // NumRxBytesNeeded was set by ProcessRxHeader
                NextState = RX_PAYLOAD;
            }
            else if ( HeaderStatus == HEADER_NO_PAYLOAD )
            {
                // Header is valid and packet has no payload:
                // Process the packet and set up to receive a new header
                ProcessRxPacket( Node );
                Node->NumRxBytesNeeded = sizeof(parapet_header_t);
                Node->NumBytesReceived = 0;
                NextState = RX_HEADER;
            }
            else    // HeaderStatus == INVALID_HEADER
            {
                // Header is invalid.  Attempt to sync using the next Byte
                Node->RxPacket.Header.u32 >>= 8;
                Node->NumRxBytesNeeded = 1;
                Node->NumBytesReceived = 3;
                NextState = RX_HEADER;
            }

            break;
        }


        //-------------------------------------------------
        // STATUS:
        //   Header with payload has been received
        //-------------------------------------------------
        case RX_PAYLOAD:
        {
            ProcessRxPacket( Node );
        }   // Intentional fall-through for common processing

        default:
        {
            Node->NumBytesReceived = 0;
            Node->NumRxBytesNeeded = sizeof(parapet_header_t);
            NextState = RX_HEADER;
            break;
        }
    }

    return NextState;
}






//=============================================================================
/// Validates a received parapet header and its payload Byte count
/// @param [in] Header      Pointer to the header to validate
/// @return
///   An element from header_status_t indicating the status of the header
///
/// @post
///   If the packet is valid and indicates payload data, Node->NumRxBytesNeeded
///   is set to the number of payload data Bytes
header_status_t ProcessRxHeader( parapet_node_t* Node )
{
    header_status_t Status = INVALID_HEADER;
    parapet_header_t* Header = &Node->RxPacket.Header;
    uint8_t ChecksumIsValid = PARAPET_HEADER_CHECKSUM_IS_VALID( *Header );
    if ( ChecksumIsValid != 0 )
    {
        // Validate payload Byte count
        uint16_t PayloadByteCount =
                PARAPET_HEADER_GET_NUM_PAYLOAD_BYTES( *Header );

        if ( PayloadByteCount <= PARAPET_MAX_NUM_PAYLOAD_BYTES )
        {
            // Determine whether the packet has payload data
            if ( PayloadByteCount != 0 )
            {
                // Packet header is valid and indicates a packet
                // with payload data
                Node->NumRxBytesNeeded = PayloadByteCount;
                Status = HEADER_WITH_PAYLOAD;
            }
            else
            {
                // Packet header is valid and indicates no payload data
                Status = HEADER_NO_PAYLOAD;
            }
        }
        else
        {
            // STATUS: packet header contains an invalid payload Byte count
            if ( Node->EventCallback != NULL )
            {
                // Post error event if an event callback is assigned
                (*Node->EventCallback)( Node,
                                        EVENT_INVALID_PAYLOAD_COUNT );
            }
        }
    }
    else
    {
        // STATUS: packet header contains an invalid payload Byte count
        if ( Node->EventCallback != NULL )
        {
            // Post error event if an event callback is assigned
            (*Node->EventCallback)( Node,
                                    EVENT_HEADER_CHECKSUM_FAILURE );
        }
    }

    return Status;
}



//=============================================================================
/// Process a received packet whose header is already validated
void ProcessRxPacket( parapet_node_t* Node )
{
    uint8_t Opcode =
            (uint8_t)PARAPET_HEADER_GET_OPCODE( Node->RxPacket.Header );

    parapet_rx_callback_fn RxCallback = Node->RxCallback[Opcode];

    if ( RxCallback != (parapet_rx_callback_fn)NULL )
    {
        // Call the Rx callback function associated with the packet's opcode
        RxCallback( Node, &Node->RxPacket );
    }
}
