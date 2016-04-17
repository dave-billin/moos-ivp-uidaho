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



struct addr_info;    // Forward-declaration

/////////////////////////////////////////////////////////////////////
// Data structure used to resolve and wrap a socket address
/////////////////////////////////////////////////////////////////////
class Address_info
{
public:

   /// IPv4 Loopback address (127.0.0.1)
   static Address_info const LOOPBACK;

   /// IPv4 UDP global broadcast address (255.255.255.255)
   static Address_info const GLOBAL_BROADCAST;

   /// Wildcard used to specify "any IP address"
   static Address_info const ANY_ADDRESS;

   enum Socket_type
   {
      TCP_SOCKET = 0,    ///< TCP (streaming) socket
      UDP_SOCKET = 1,    ///< UDP (datagram) socket
   };

   //////////////////////////////////////////////////////////////////
   /// Create, resolving a hostname to an address
   /// @param hostname  Hostname to resolve using using getaddrinfo()
   /// @param port      Network port or zero to indicate "any" port
   /// @throw     Socket_exception on failure to resolve hostname
   /// @remarks   The socket type of the object defaults to TCP_SOCKET
   Address_info( std::string const& hostname, uint16_t port = 0 );

   //////////////////////////////////////////////////////////////////
   /// Copy constructor
   Address_info( Address_info const& other );

   /// Called when the object goes out of scope
   ~Address_info();

   //////////////////////////////////////////////////////////////////
   /// Assignment operator overload
   Address_info& operator=( Address_info const& other );


   //////////////////////////////////////////////////////////////////
   /// @return
   ///   A textual representation of the network address
   ///   (e.g. dotted quad representation for an IPv4 address)
   std::string as_string() const;


   //////////////////////////////////////////////////////////////////
   /// @return the type of socket indicated by the address
   Socket_type socket_type() const;

   /// Sets the type of socket indicated by the address
   void socket_type(Socket_type type);


   //////////////////////////////////////////////////////////////////
   /// @return the network port indicated by the address
   uint16_t port() const;

   /// Sets the network port indicated by the address
   void port( uint16_t value );


   //////////////////////////////////////////////////////////////////
   /// @internal
   struct addr_info const& addrinfo() const;

private:
   struct addr_info* m_addr_info;

   /// Delete heap memory in m_addr_info
   void delete_info();

   /// Validate object accesses
   void throw_if_invalid() const;
};




//=============================================================================
/// A class encapsulating a UDP socket
//=============================================================================
class UDP_socket
{
public:

   //////////////////////////////////////////////////////////////////
   /// Creates and opens a UDP socket
   /// @param address
   ///   Object containing the address and port used to open the socket
   ///
   /// @remarks
   ///   UDP broadcast is automatically enabled if address is
   ///   Address_info::GLOBAL_BROADCAST.
   ///
   UDP_socket( Address_info const& address );

   //////////////////////////////////////////////////////////////////
   /// Called when the object goes out of scope
   /// @remarks   Automatically closes the socket if it is open
   ~UDP_socket();


   //////////////////////////////////////////////////////////////////
   /// Sends a UDP datagram over the socket
   ///
   /// @param address    Address and port to send the datagram to
   /// @param buf        Pointer to the data to send in the datagram
   /// @param num_bytes  Number of Bytes in buf to send
   ///
   /// @throw Socket_exception on failure to send the datagram
   ///
   /// @remarks
   ///   If UDP broadcast packets have been enabled on the socket,
   ///   the contents of address are ignored.
   ///
   void sendto( Address_info const& address, uint8_t const* data,
                size_t num_bytes );


   //////////////////////////////////////////////////////////////////
   /// Sets whether write() sends UDP broadcast datagrams
   ///
   /// @param enable_broadcast
   ///   true if write() should send UDP broadcast datagrams; else
   ///   false for normal (unicast) datagrams
   ///
   void should_broadcast( bool enable_broadcast );

   //////////////////////////////////////////////////////////////////
   /// @return
   ///   true if write() is configured to send UDP broadcast
   ///   datagrams; else false
   ///
   bool should_broadcast() const;


private:
   /// The socket's file descriptor or INVALID_FILE_DESCRIPTOR if not open
   Socket_fd m_socket_fd;

   /// true if send() writes UDP broadcast datagrams; else false for
   /// normal unicast datagrams
   bool m_should_broadcast;

   friend struct UnitTest::Socket_accessor;
};

}} // END namespace BunnySock::Sockets

#endif // END #indef _UDP_SOCKET_H_
