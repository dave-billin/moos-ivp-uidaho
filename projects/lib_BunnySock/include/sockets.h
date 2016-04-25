//=============================================================================
/*    Copyright (c) 2016  Dave Billin <david.billin@vandals.uidaho.edu>

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
/// @file
/// @brief  Declaration of a UDP socket class
/// @author Dave Billin <david.billin@vandals.uidaho.edu>
//=============================================================================
#ifndef _UDP_SOCKET_H_
#define _UDP_SOCKET_H_

#include <string>
#include <stdexcept>
#include <stdint.h>

namespace UnitTest
{
   struct Socket_accessor;
}

namespace BunnySock { namespace Sockets
{

typedef int Socket_fd;     ///< Socket file descriptor type

/// Placeholder for an invalid (i.e. closed) socket file descriptor
static Socket_fd const INVALID_FILE_DESCRIPTOR = -1;

/////////////////////////////////////////////////////////////////////
// Exception class that may be thrown by socket methods
/////////////////////////////////////////////////////////////////////
class Socket_exception : std::exception
{
public:
   //////////////////////////////////////////////////////////////////
   /// Creates an instance of the object
   /// @param description     Description of the error
   explicit Socket_exception( char const* description );

   //////////////////////////////////////////////////////////////////
   /// Creates an instance of the object
   /// @param errno_value  errno value used to create the description
   /// @param prefix       Text prepended to errno description
   Socket_exception( int errno_value, char const* prefix = NULL );

   //////////////////////////////////////////////////////////////////
   /// Called when the object goes out of scope
   virtual ~Socket_exception() throw()
   {}

   //////////////////////////////////////////////////////////////////
   virtual char const* what() const throw()
   { return m_description.c_str(); }

   //////////////////////////////////////////////////////////////////
   int errno_value() const
   { return m_errno; }

private:
   std::string m_description;
   int m_errno;
};



class Socket_address_impl;    // Forward-declaration

/////////////////////////////////////////////////////////////////////
/// Data structure used to resolve and wrap a socket address
///
/// @remarks
///   This class provides an abstraction of the details (and
///   platform-specific header files) of an IP address passed
///   to socket functions.
/////////////////////////////////////////////////////////////////////
class Socket_address
{
public:

   typedef uint16_t Network_port;   ///< Network port number type

   static Socket_address const LOOPBACK_IPv4;   ///< IPv4 Loopback address
   static Socket_address const LOOPBACK_IPv6;   ///< IPv6 Loopback address

   /// IPv4 wildcard used to specify "any IP address"
   static Socket_address const IPv4_ANY_ADDRESS;

   /// IPv6 wildcard used to specify "any IP address"
   static Socket_address const IPv6_ANY_ADDRESS;

   /// IPv4 UDP global broadcast address (255.255.255.255)
   static Socket_address const GLOBAL_BROADCAST;

   /// Wildcard used to indicate "any available port"
   static Network_port const ANY_PORT = 0;


   //////////////////////////////////////////////////////////////////
   /// Create, resolving a hostname to an address
   ///
   /// @param hostname  Hostname to resolve using using getaddrinfo()
   /// @param port      Network port number
   ///
   /// @throw     Socket_exception on failure to resolve hostname
   ///
   Socket_address( std::string const& hostname,
                   Network_port port = ANY_PORT );

   //////////////////////////////////////////////////////////////////
   /// Create from a specified address
   ///
   /// @param address_obj
   ///   Object containing the network address to use
   ///
   /// @param port
   ///   Network port number or ANY_PORT to use the port number given
   ///   by address_obj
   ///
   /// @throw     Socket_exception if address_obj or port is invalid
   ///
   Socket_address( Socket_address const& address_obj,
                   Network_port port = ANY_PORT );

   //////////////////////////////////////////////////////////////////
   /// Copy constructor
   //Socket_address( Socket_address const& other );

   /// Called when the object goes out of scope
   ~Socket_address();

   //////////////////////////////////////////////////////////////////
   /// Assignment operator overload
   Socket_address& operator=( Socket_address const& other );


   //////////////////////////////////////////////////////////////////
   /// @return
   ///   A textual representation of the network address
   ///   (e.g. dotted quad representation for an IPv4 address)
   std::string as_string() const;


   //////////////////////////////////////////////////////////////////
   /// @return the network port indicated by the address
   Network_port port() const;

   /// Sets the network port indicated by the address
   void port( Network_port value );


   //////////////////////////////////////////////////////////////////
   /// @return The underlying implementation of the address object
   ///
   /// @remarks
   ///   The returned object is only intended to be used directly by
   ///   Socket classes.  The address and port of an implementation
   ///   object may be accessed using the static impl_address() and
   ///   impl_port() methods.
   ///
   struct Socket_address_impl const& impl() const;

   /// @return
   ///   A mutable reference to the underlying address implementation
   struct Socket_address_impl& impl();

   //////////////////////////////////////////////////////////////////
   /// Populates a string with the network address (e.g. dotted quad)
   /// in a Socket_address_impl object
   ///
   /// @param impl
   ///   Object to extract an address from
   ///
   /// @param[out] address
   ///   String to populate with the address from impl
   ///
   static void impl_address( Socket_address_impl const& impl,
                             std::string& address );

   //////////////////////////////////////////////////////////////////
   /// @return
   ///   A string containing the network address (e.g. dotted quad)
   ///   in a Socket_address_impl object
   ///
   /// @param
   ///   impl Socket_address_impl object whose address is to be
   ///   returned
   ///
   static std::string impl_address( Socket_address_impl const& impl );

   //////////////////////////////////////////////////////////////////
   /// @return The network port in a specified Socket_address_impl
   /// @param impl
   ///   impl Socket_address_impl object whose network port is to be
   ///   returned
   ///
   static Network_port impl_port( Socket_address_impl const& impl );

private:

   // @internal   Internal abstraction of a socket address
   Socket_address_impl* m_address_impl;

   /// Delete heap memory in m_addr_info
   void delete_info();

   /// Validate object accesses
   void throw_if_invalid() const;

   /// Private constructor used for address constants
   /// @param impl   Address to use
   Socket_address( Socket_address_impl const& impl );
};

/////////////////////////////////////////////////////////////////////
inline std::string Socket_address::impl_address(
                                    Socket_address_impl const& impl )
{
   std::string address;
   impl_address(impl, address);
   return address;
}



//=============================================================================
/// A class encapsulating a UDP socket
///
/// @remarks
///   This class may be used to send and receive unicast UDP datagrams
//=============================================================================
class UDP_socket
{
public:

   //////////////////////////////////////////////////////////////////
   /// Creates and opens a UDP socket
   ///
   /// @param address
   ///   Object containing the address/port the socket binds to
   ///
   /// @remarks
   ///   If address is one of the IPvX_ANY_IP variants, the socket
   ///   will receive from any address.  Otherwise, the socket only
   ///   receives datagrams from the specified address
   ///
   UDP_socket(
         Socket_address const& address = Socket_address::IPv4_ANY_ADDRESS );

   //////////////////////////////////////////////////////////////////
   /// Called when the object goes out of scope
   /// @remarks   Automatically closes the socket if it is open
   virtual ~UDP_socket();


   //////////////////////////////////////////////////////////////////
   /// Sends a UDP datagram over the socket
   ///
   /// @param address    Address and port to send the datagram to
   /// @param buf        Pointer to the data to send in the datagram
   /// @param num_bytes  Number of Bytes in buf to send
   ///
   /// @throw Socket_exception on failure to send the datagram
   ///
   void sendto( Socket_address const& address, uint8_t const* buf,
                size_t num_bytes );

   //////////////////////////////////////////////////////////////////
   /// Receives a UDP datagram over the socket
   ///
   /// @param buf        Pointer to a buffer to receive the datagram
   /// @param num_bytes  Size of buf in Bytes
   ///
   /// @param[out] sender_address
   ///   Object to populate with the address of the host that sent
   ///   the datagram
   ///
   /// @return The number of Bytes read into buf
   ///
   /// @throw Socket_exception on error
   ///
   /// @remarks
   ///   Caller is blocked until a datagram is received or the
   ///   socket is closed
   ///
   uint32_t recvfrom( uint8_t* buf, size_t num_bytes,
                      Socket_address& sender_address );

   //////////////////////////////////////////////////////////////////
   /// @return The socket's file descriptor
   /// @remarks
   ///   This method is provided so that applications can perform a
   ///   select() on the socket.
   ///
   Socket_fd const& file_descriptor() const
   { return m_socket_fd; }

protected:

   /// @return The socket's file descriptor
   Socket_fd& socket_fd()
   { return m_socket_fd; }

private:
   /// The socket's file descriptor or INVALID_FILE_DESCRIPTOR if not open
   Socket_fd m_socket_fd;

   friend struct UnitTest::Socket_accessor;
};


//=============================================================================
/// A specialization of UDP_socket that sends broadcast datagrams
///
/// @remarks
///   This class provides a specialization of UDP_socket that can send UDP
///   broadcast datagrams.
//=============================================================================
class UDP_broadcast_socket : public UDP_socket
{
public:

   //////////////////////////////////////////////////////////////////
   /// Creates and opens a UDP socket
   ///
   /// @param address
   ///   Object containing the address/port the socket binds to
   ///
   /// @remarks
   ///   If address is one of the IPvX_ANY_IP variants, the socket
   ///   will receive from any address.  Otherwise, the socket only
   ///   receives datagrams from the specified address
   ///
   UDP_broadcast_socket(
            Socket_address const& address = Socket_address::IPv4_ANY_ADDRESS );

   //////////////////////////////////////////////////////////////////
   /// Called when the object goes out of scope
   /// @remarks   Automatically closes the socket if it is open
   virtual ~UDP_broadcast_socket()
   {}

   //////////////////////////////////////////////////////////////////
   /// Sends a UDP broadcast datagram over the socket
   ///
   /// @param port       Destination port for the UDP datagram
   /// @param buf        Pointer to the data to send in the datagram
   /// @param num_bytes  Number of Bytes in buf to send
   ///
   /// @throw Socket_exception on failure to send the datagram
   ///
   void send( Socket_address::Network_port port, uint8_t const* buf,
              size_t num_bytes );

private:
   /// Address used to send broadcast packets
   Socket_address m_broadcast_address;

   friend struct UnitTest::Socket_accessor;
};


}} // END namespace BunnySock::Sockets

#endif // END #indef _UDP_SOCKET_H_
