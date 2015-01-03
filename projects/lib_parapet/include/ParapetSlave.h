//=============================================================================
/** @file ParapetSlave.h
 *
 * @brief
 *	Declaration of a base class for objects that implement a parapet Slave
 *	device
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#ifndef PARAPETSLAVE_H_
#define PARAPETSLAVE_H_

#include "parapet.h"
#include "ParapetEventBroadcaster.h"


namespace parapet
{

class ParapetSlave : public ParapetEventBroadcaster
{
public:

    /** Creates a ParapetSlave object with a specified Device ID
     *
     * @param DeviceID
     *  5-bit device address of the Slave Device implemented by the object
     *  (1..31)
     */
    ParapetSlave( uint8_t DeviceID );

    /** Destructor */
    virtual ~ParapetSlave();


    //=============================
    // Virtual Methods
    //=============================

    //=========================================================================
    /** Transmits a parapet packet to a Master device
     *
     * @param [in] Packet
     *  Reference to the packet to transmit
     */
    virtual bool Transmit( parapet_packet_t& Packet ) = 0;

    //=========================================================================
    /** Called to receive and process data from a parapet Master device
     * @return
     *  true if one or more packets were received and processed
     */
    virtual bool Receive( void ) = 0;

    //=============================
    // END Virtual Methods
    //=============================



    enum eParapetSlaveEvents
    {
        INVALID_NUMBYTESTOFOLLOW = -100,    /**< Received a packet header with
                                                 an invalid NumBytesToFollow
                                                 field */

        MASTER_M_BIT_CLEARED = -102,        /**< Received a packet without its
                                                 M (master) bit set */
    };


    //=========================================================================
    /** Enables or disables promiscuous receive mode
     *
     * @remarks
     *  Promiscuous receive mode is initially disabled when Slave object is
     *  created
     *
     * @param PromiscuityShouldBeEnabled
     *  true if Promiscuous receive mode should be enabled, allowing the Slave
     *  to receive packets addressed to any device; else false if the Slave
     *  should only receive packets addressed to its configured Device ID
     */
    void SetPromiscuousModeEnabled( bool PromiscuityShouldBeEnabled )
    {
        m_PromiscuityIsEnabled = PromiscuityShouldBeEnabled;
    }


    //=========================================================================
    /** Returns true if Promiscuous receive mode is enabled */
    bool PromiscuousModeIsEnabled( void ) const
    {
        return m_PromiscuityIsEnabled;
    }


protected:

    //=========================================================================
    /** Parses received data into parapet packets
     *
     * @details
     *  Derived classes can call this function to process data received on the
     *  communications channel connected to a parapet Master device.  This
     *  function will parse the received data into packets and pass the packet
     *  data to registered ParapetListener objects.
     *
     * @param RxData
     *  Pointer to a buffer of received data to be processed
     *
     * @param NumBytes
     *  Number of Bytes pointed to by RxData
     *
     * @return
     *  true if one or more packets were received and processed; else false
     */
    bool ProcessRxData( char const* RxData, uint32_t NumBytesToProcess );



    //=========================================================================
    /** Returns the number of incoming Bytes that must be processed before a
     *  complete parapet packet has been received
     *
     * @return
     *  The number of Bytes required before a full packet has been received
     */
    inline uint32_t NumRxBytesNeeded( void ) const
    { return m_NumRxBytesNeeded; }


private:

    /** States that the Rx FSM may take on */
    typedef enum
    {
        RX_SYNC = 0,    /**< Receiving for SYNC Byte */
        RX_HEADER,      /**< Receiving the remainder of the packet header */
        RX_PAYLOAD,     /**< Receiving packet payload Bytes */
        NUM_RX_STATES   /**< NOT A VALID STATE - used for bounds-checking */
    } rx_state_t;

    rx_state_t m_RxState;        /**< Current state of the Rx FSM */
    uint8_t m_DeviceID;          /**< Device ID of the Slave object */
    uint32_t m_NumBytesReceived; /**< Count of Bytes received in the Rx packet
                                      buffer */
    uint32_t m_NumRxBytesNeeded; /**< Number of received Bytes needed for the
                                      Rx FSM to transition to the next state */

    parapet_packet_t m_RxPacket;    /**< Buffer used to receive packets */

    bool m_PromiscuityIsEnabled; /**< true if promiscuous mode is enabled */


};


}   // END namespace parapet

#endif /* PARAPET_SLAVE_H_ */
