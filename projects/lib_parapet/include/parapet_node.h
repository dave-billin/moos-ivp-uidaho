//=============================================================================
/** @file parapet_node.h
 *
 * @brief
 *  Declaration of data structures and associated API functions used to
 *  implement a parapet device node
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================
#ifndef _PARAPET_NODE_H_
#define _PARAPET_NODE_H_

#include <stdint.h>
#include "parapet.h"
#include "parapet_callbacks.h"

#ifdef __cplusplus
using namespace parapet;
#endif


/// States that the decoder FSM may take on
typedef enum
{
    RX_HEADER,      ///< Receive 4 Bytes comprising a packet header
    RX_PAYLOAD      ///< Receiving packet payload
} rx_state_t;


//-----------------------------------------------------------------------------
/** @typedef ParapetSlave_t
 *  @brief
 *     Data structure used to store the state of a parapet device node
 */
typedef struct
{
    rx_state_t RxState;  ///< Current state of the Rx finite state machine

    /// The number of Bytes received into the Rx packet buffer
    uint16_t NumBytesReceived;

    /// The number of received Bytes needed before the Rx FSM
    /// will transition to the next state
    uint16_t NumRxBytesNeeded;

    uint8_t DeviceID;     /// Device ID of the node

    /// Non-zero if promiscuous receive mode is enabled; else zero
    uint8_t PromiscuousModeIsEnabled;

    /// Buffer incoming packets are received in
    parapet_packet_t RxPacket;

    /// Packet Rx callback function pointers - one for each opcode
    parapet_rx_callback_fn RxCallback[7];

    /// Event callback function pointer
    parapet_event_callback_fn EventCallback;

} parapet_node_t;
//-----------------------------------------------------------------------------


#ifdef __cplusplus
extern "C" {
#endif


//=============================================================================
/** Initializes a parapet device node and clears its callback functions
 *
 * @param [in,out] Node
 *  A pointer to the device node data of the node to be initialized
 *
 * @param [in] DeviceID
 *  Device ID the node should use
 */
void parapet_node_init( parapet_node_t* Node, uint8_t DeviceID );




//=============================================================================
/** Assigns an Rx callback function to the node
 *
 * @param [in] Node
 *   Node to assign to
 *
 * @param [in] Opcode
 *   Opcode to assign an Rx callback function to
 *
 * @param [in] RxCallback
 *   Pointer to the function that should be called when a packet with the
 *   specified opcode is received by the Node, or NULL to disable handling of
 *   the specified Opcode
 */
void parapet_SetRxHandler( parapet_node_t* Node, parapet_opcode_t Opcode,
                           parapet_rx_callback_fn RxCallback );


//=============================================================================
/** Assigns an Event callback function to the node
 *
 * @param [in] Node
 *   Node to assign to
 *
 * @param [in] EventCallback
 *   Pointer to the function that should be called when an Event occurs on the
 *   Node, or NULL to disable event handling
 */
void parapet_SetEventHandler( parapet_node_t* Node,
                              parapet_event_callback_fn EventCallback );


//=============================================================================
/** Enables or disables promiscuous receive mode on a Parapet Slave node
 *
 * @param [in,out] Node
 *  Pointer to the node to operate on
 *
 * @remarks
 *  Promiscuous receive mode is disabled when a device node is initialized
 *
 * @param [in] PromiscuityShouldBeEnabled
 *  Non-zero if Promiscuous receive mode should be enabled, allowing the node
 *  to process any packet that is received; regardless of whether its DEST_ID
 *  matches the node's DeviceID field.  Else zero if the node should ONLY
 *  process packets whose DEST_ID field matches its DeviceID
 */
void parapet_enable_promiscuous_mode( parapet_node_t* Node,
                                      uint8_t PromiscuityShouldBeEnabled );



//=========================================================================
/** @return non-zero if Promiscuous receive mode is enabled for a node
 * @param [in] Node  Pointer to the node to check
 */
uint8_t parapet_promiscuous_mode_is_enabled( parapet_node_t* Node );




//=============================================================================
/** Passes data to a parapet node to be received into its buffer and parsed
 *  into individual packets
 *
 * @details
 *  This function parses a supplied buffer of received data into packets.  When
 *  a full packet has been received, the receive handler function corresponding
 *  to the packet's OPCODE is called if it has been registered.  On failure to
 *  process a packet, the node's Event handler function may be called if it has
 *  been registered.
 *
 * @param [in] Node
 *  Pointer to the node that should receive and parse the data
 *
 * @param [in] RxData
 *  Pointer to packet data to be processed
 *
 * @param [in] NumBytes
 *  Number of data Bytes pointed to by RxData
 *
 * @return
 *  TRUE if one or more complete packets were received and processed
 */
uint8_t parapet_process_node( parapet_node_t* Node, char* RxData,
                              uint16_t NumBytesToProcess );

#ifdef __cplusplus
}
#endif


#endif /* _PARAPET_NODE_H_ */
