//=============================================================================
/** @file parapet_callbacks.h
 *
 * @brief
 *	Declaration of signatures for parapet Master/Slave callback functions
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef PARAPET_CALLBACKS_H_
#define PARAPET_CALLBACKS_H_

#include <stdint.h>

#ifdef __cplusplus
namespace parapet
{
#endif


//-----------------------------------------------------------------------------
/** @enum parapet_event_id_t
 *  @brief
 *     Event ID's that may be reported by a parapet device node
 */
typedef enum
{
    EVENT_HEADER_CHECKSUM_FAILURE = -1, /**< A header failed checksum
                                             validation */

    EVENT_INVALID_PAYLOAD_COUNT = -1,   /**< A header with an invalid Byte
                                             Count was detected */
} parapet_event_id_t;




//-----------------------------------------------------------------------------
/** @typedef parapet_rx_callback_fn
 *
 * @brief
 *  Signature of a callback function called when a parapet packet is received
 *
 * @param Receiver
 *  Pointer to the parapet_master_t or parapet_slave_t that received the packet
 *
 * @param Packet
 *  Pointer to received packet data
 */
typedef void (*parapet_rx_callback_fn)( void* Receiver,
                                        parapet_packet_t* Packet );
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
/** @typedef ParapetEventCallback
 *
 * @brief
 *  Signature of a callback function called when an error or event occurs
 *  while receiving parapet packet data
 *
 * @param Receiver
 *  Pointer to the parapet_master_t or parapet_slave_t posting the event
 *
 * @param EventID
 *  ID of the event
 */
typedef void (*parapet_event_callback_fn)( void* Receiver,
                                           parapet_event_id_t EventID );
//-----------------------------------------------------------------------------

#ifdef __cplusplus
}   // END namespace parapet
#endif

#endif /* PARAPET_CALLBACKS_H_ */
