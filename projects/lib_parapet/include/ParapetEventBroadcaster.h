//=============================================================================
/** @file ParapetEventBroadcaster.h
 *
 * @brief
 *	Declaration of a base class for objects that receive parapet packets and
 *	need to pass them on to registered listeners
 *
 * @author Dave Billin (david.billin@vandals.uidaho.edu)
 */
//=============================================================================

#ifndef PARAPET_EVENT_BROADCASTER_H
#define PARAPET_EVENT_BROADCASTER_H

#include <tr1/unordered_set>

#include "parapet.h"
#include "ParapetListener.h"

namespace parapet
{

class ParapetListener;     // Forward declaration


//=============================================================================
/** Base class for objects that receive parapet packets and pass them on to
 *  registered listeners
 */
class ParapetEventBroadcaster
{
public:

    ParapetEventBroadcaster( void ) {}       /**< Default constructor */
    virtual ~ParapetEventBroadcaster() {}    /**< Destructor */

    //=========================================================================
    /** Registers a listener object to be notified of received packets and
     *  events
     *
     * @param [in] NewListener
     *  A pointer to the listener object to register
     */
    void AddListener( ParapetListener* NewListener );


    //=========================================================================
    /** Un-registers a listener object
     *
     * @param [in] ListenerToRemove
     *  A pointer to the listener object to un-register
     */
    void RemoveListener( ParapetListener* ListenerToRemove );


    //=========================================================================
    /** Returns true if a specified listener object is registered with the
     *  ParapetEventBroadcaster object
     */
    bool ListenerIsRegistered( ParapetListener* Listener );


protected:

    //=========================================================================
    /** Posts a received packet to registered listeners
     *
     * @param Packet
     *  Received packet to be passed to listeners
     */
    void PostReceivedPacket( parapet_packet_t const& Packet );


    //=========================================================================
    /** Posts an event to registered listeners
     *
     * @param EventID
     *  ID of the event to post
     *
     * @param Packet
     *  Pointer to a packet associated with the event or NULL if no packet is
     *  associated with the event
     */
    void PostEvent( int EventID );


    //=========================================================================
    /** Generates a timestamp that may be used to report a received packet or
     *  event
     */
    static double MakeTimestamp( void );


private:
    typedef std::tr1::unordered_set<ParapetListener*> ListenerSet;
    ListenerSet m_Listeners;

    //=========================================================================
    /** Returns an iterator that points to a registered listener in the
     *  listener list, or the end of the listener list if the listener was
     *  not found
     */
    ListenerSet::iterator FindListener( ParapetListener* Target);

};


//=============================================================================
inline
void ParapetEventBroadcaster::AddListener( ParapetListener* NewListener )
{

    // Register the listener
    m_Listeners.insert( NewListener );
}


//=============================================================================
inline void ParapetEventBroadcaster::RemoveListener(
                                            ParapetListener* ListenerToRemove )
{
    m_Listeners.erase( ListenerToRemove );
}


//=============================================================================
inline bool ParapetEventBroadcaster::ListenerIsRegistered(
                                                    ParapetListener* Listener )
{
    return m_Listeners.end() != FindListener(Listener);
}


}   // END namespace parapet

#endif /* PARAPET_PACKETBROADCASTER_H_ */
