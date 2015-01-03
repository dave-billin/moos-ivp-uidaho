//=============================================================================
/*  Copyright (C) 2013  Dave Billin

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
/** @file ParapetDeviceNode.h
 *
 * @brief
 *   A short description of the file
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#ifndef PARAPETDEVICENODE_H_
#define PARAPETDEVICENODE_H_

#include <stdint.h>
#include <map>
#include "parapet.h"
#include "parapet_header.h"
#include "parapet_node.h"
#include "ParapetEventBroadcaster.h"


namespace parapet
{

//=============================================================================
class ParapetDeviceNode : public ParapetEventBroadcaster
{
public:

    //=========================================================================
    ParapetDeviceNode( uint8_t DeviceID );


    //=========================================================================
    virtual ~ParapetDeviceNode();


    //=========================================================================
    /** Sets the node's Device ID
     *
     * @param DeviceID
     *  Device ID the node should use
     */
    void set_device_id( int8_t DeviceID ) { m_Node.DeviceID = DeviceID; }

    /** @return The node's Device ID */
    uint8_t get_device_id( void ) { return m_Node.DeviceID; }


    //=========================================================================
    /** Enables or disables promiscuous listening mode
     *
     * @param ShouldEnable
     *   true if Promiscuous receive mode should be enabled, allowing the node
     *   to process any packet that is received; regardless of whether its
     *   DEST_ID matches the node's DeviceID field.  Else zero if the node
     *   should ONLY process packets whose DEST_ID field matches its DeviceID
     */
    void enable_promiscuous_listening( bool ShouldEnable )
    { parapet_enable_promiscuous_mode( &m_Node, ShouldEnable ); }

    /** @return true if promiscuous mode is enabled; else false */
    bool promiscuous_listening_is_enabled( void )
    { return parapet_promiscuous_mode_is_enabled( &m_Node ); }


    //=========================================================================
    /** Decodes parapet packets from a raw data stream
     *
     * @details
     *  This function parses a supplied buffer of received data into packets.
     *  When a complete packet is decoded, it is passed to the OnPacketReceived
     *  method of registered listener objects.  If a decoding error occurs, it
     *  is passed to the OnEvent method of registered listener objects.
     *
     * @param [in] RxData
     *  Pointer to packet data to be processed
     *
     * @param [in] NumBytes
     *  Number of data Bytes pointed to by RxData
     *
     * @return
     *  true if one or more complete packets were received and processed; else
     *  false
     */
    bool process( char const* RxData, uint16_t NumBytes );


protected:
    parapet_node_t m_Node;


private:
    // Static interface used to adapt from the parapet_callbacks C
    // implementation to C++ object methods
    static std::map<uint8_t, ParapetDeviceNode*> s_DeviceID_to_Object_Map;

    // Rx callback function adapter
    static void RxCallbackAdapter( void* Receiver, parapet_packet_t* Packet );

    // Event callback function adapter
    static void EventCallbackAdapter( void* Receiver,
                                      parapet_event_id_t EventID );

    // Returns a reference to the device node mapped to the DeviceID in a
    // parapet_node_t structure
    static ParapetDeviceNode& LookupDeviceNode( parapet_node_t const* Node );

    //------------------------------------------------------
    // Default constructor, copy constructor, and
    // assignment operator are disallowed
    //------------------------------------------------------
    ParapetDeviceNode( void );
    ParapetDeviceNode( ParapetDeviceNode const& );
    ParapetDeviceNode const& operator=( ParapetDeviceNode const& );
};


} /* namespace parapet */


#endif /* PARAPETDEVICENODE_H_ */
