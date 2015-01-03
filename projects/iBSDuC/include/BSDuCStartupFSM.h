//=============================================================================
/** @file BSDuCStartupFSM.h
 *
 * @brief
 *	Declaration of an object to implement BSD microcontroller session startup
 *	logic
 *
 * @author Dave Billin
 */
//=============================================================================
#ifndef BSDUCSTARTUPFSM_H_
#define BSDUCSTARTUPFSM_H_

#include <stdint.h>
#include "ParapetSerialDeviceNode.h"
#include "ParapetListener.h"
#include "PrecisionTime.h"


//=============================================================================
/** An object that implements the BSD microcontroller startup sequence as a
 *  finite state machine
 */
class BSDuC_StartupFSM : public parapet::ParapetListener
{
public:

    //=========================================================================
    /** Creates an instance of the object
     *
     * @param ParapetNode
     *   Reference to the parapet node used to communicate with the BSD
     *   microcontroller
     */
    BSDuC_StartupFSM( ParapetSerialDeviceNode& ParapetNode );

    /** Called when the object goes out of scope */
    virtual ~BSDuC_StartupFSM();


    //=========================================================================
    /** Processes a single iteration of the BSDuC startup sequence
     *
     * @return
     *   true if the startup sequence is complete and a comms session is
     *   online; else false
     */
    bool Process( void );


    //=========================================================================
    /** Resets the object to the beginning of the startup sequence */
    void Reset( void );


    //=========================================================================
    /** @return true if the startup sequence is finished; else false */
    bool IsOnline( void ) const { return SESSION_ONLINE == m_State; }

    //================================
    // ParapetListener API methods
    //================================

    //=========================================================================
    /** This function will be called when a parapet packet has been received
     *  on the serial port connected to the BSD microcontroller
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object  that received the
     *  packet.
     *
     * @param [in] Opcode
     *  The value of the OPCODE field in the received packet's header
     *
     * @param [in] Packet
     *  A reference to the packet that was received.
     */
    virtual void OnPacketReceived(
                                parapet::ParapetEventBroadcaster const& Source,
                                parapet::parapet_opcode_t Opcode,
                                uint8_t SequenceID,
                                parapet::parapet_packet_t const& Packet );


    //=========================================================================
    /** This function will be called when a notable event occurs when receiving
     *  parapet packets from the BSD microcontroller
     *
     * @note
     *  This function may be called from a separate thread, depending on the
     *  implementation of the ParapetEventBroadcaster object that has received
     *  the packet.  It is left to derived classes to implement any
     *  synchronization that may be required.
     *
     * @param [in] Source
     *  A reference to the ParapetEventBroadcaster object signaling the event
     *
     * @param [in] EventID
     *  ID of the event
     */
    virtual void OnPacketEvent( parapet::ParapetEventBroadcaster const& Source,
                                int EventID );


private:

    /** Reference to the parapet serial node used to communicate with the
     *  BSD microcontroller */
    ParapetSerialDeviceNode& m_ParapetNode;

    /** States of communication with the BSD microcontroller */
    enum eBSDuC_FSM_States
    {
        ENTRY,   /**< FSM entry state */

        /** Handshake with the BSD Microcontroller to open
         *  a new comms session */
        OPEN_SESSION,

        /**< Read BSD microcontroller firmware version */
        READ_FIRMWARE_VERSION,

        /**< Read initial sensor values from BSD microcontroller */
        READ_INITIAL_SENSORS,

        /** Read initial battery monitor readings from BSD microcontroller */
        READ_INITIAL_BATTERY_MONITOR,

        /**< Service regular sensor and battery monitor polling */
        SESSION_ONLINE,
    };

    /** Current state of communications with the BSD microcontroller */
    eBSDuC_FSM_States m_State;

    /** Used to implement communication timeouts */
    YellowSubUtils::PrecisionTime m_ReferenceTime;

    /** Used to specify the maximum time to wait for the expected packet */
    YellowSubUtils::PrecisionTimeInterval m_Timeout;

    /** Opcode of an expected reply from the BSD microcontroller */
    parapet::parapet_opcode_t m_ExpectedOpcode;

    /** Sequence ID of an expected reply from the BSD microcontroller */
    uint8_t m_ExpectedSequenceID;

    /** Used to signal receipt of a packet from the BSD microcontroller whose
     *  opcode and Sequence ID are given by m_ExpectedOpcode and
     *  m_ExpectedSequenceID respectively
     */
    bool m_ExpectedPacketWasReceived;


    //----------------------------------------------------------------
    // Default constructor, copy constructor, and assignment
    // operator are disallowed
    BSDuC_StartupFSM( void );
    BSDuC_StartupFSM( BSDuC_StartupFSM const& );
    BSDuC_StartupFSM& operator=( BSDuC_StartupFSM const& );
    //----------------------------------------------------------------
};

#endif /* BSDUCSTARTUPFSM_H_ */
