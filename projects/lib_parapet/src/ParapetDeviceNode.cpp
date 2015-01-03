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
/** @file ParapetDeviceNode.cpp
 *
 * @brief
 *   A short description of the file
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================
#include <cassert>
#include <cstddef>
#include "parapet_header.h"
#include "ParapetDeviceNode.h"

namespace parapet
{

std::map<uint8_t, ParapetDeviceNode*>
    ParapetDeviceNode::s_DeviceID_to_Object_Map;


//=============================================================================
ParapetDeviceNode::ParapetDeviceNode( uint8_t DeviceID )
{
    parapet_node_init( &m_Node, DeviceID );

    //-----------------------------------------------------------
    // Add a mapping between the device ID and the node object
    //-----------------------------------------------------------

    // If you hit this assert, you've tried to create two ParapetDeviceNode
    // objects with the same DeviceID.  This is not permitted in the parapet
    // protocol.  All device ID's in a system must be unique!
    assert( s_DeviceID_to_Object_Map.end() ==
            s_DeviceID_to_Object_Map.find( DeviceID ) );
    s_DeviceID_to_Object_Map[ DeviceID ] = this;

    // Assign handler callbacks
    parapet_SetEventHandler( &m_Node, ParapetDeviceNode::EventCallbackAdapter );

    for ( int i = 0; i < 8; ++i )
    {
        parapet_SetRxHandler( &m_Node, static_cast<parapet_opcode_t>(i),
                              ParapetDeviceNode::RxCallbackAdapter );
    }

}


//=============================================================================
ParapetDeviceNode::~ParapetDeviceNode()
{
}

//=============================================================================
bool ParapetDeviceNode::process( char const* RxData, uint16_t NumBytes )
{
    return parapet_process_node( &m_Node, const_cast<char*>(RxData), NumBytes );
}


//=============================================================================
void ParapetDeviceNode::RxCallbackAdapter( void* Receiver,
                                           parapet_packet_t* Packet )
{
    assert( (Receiver != NULL) && (Packet != NULL) );

    parapet_node_t* ParapetNode = reinterpret_cast<parapet_node_t*>(Receiver);

    ParapetDeviceNode& DeviceNode = LookupDeviceNode( ParapetNode );
    DeviceNode.PostReceivedPacket( *Packet );
}


//=============================================================================
void ParapetDeviceNode::EventCallbackAdapter( void* Receiver,
                                      parapet_event_id_t EventID )
{
    assert( Receiver != NULL );

    parapet_node_t* ParapetNode = reinterpret_cast<parapet_node_t*>(Receiver);

    ParapetDeviceNode& DeviceNode = LookupDeviceNode( ParapetNode );
    DeviceNode.PostEvent( EventID );
}


//=============================================================================
ParapetDeviceNode& ParapetDeviceNode::LookupDeviceNode(
                                                parapet_node_t const* Node )
{
    assert( Node != NULL );

    std::map<uint8_t, ParapetDeviceNode*>::iterator DeviceNodeMapItem =
            ParapetDeviceNode::s_DeviceID_to_Object_Map.find( Node->DeviceID );

    assert( s_DeviceID_to_Object_Map.end() != DeviceNodeMapItem );

    return *(DeviceNodeMapItem->second);
}

} /* namespace parapet */
