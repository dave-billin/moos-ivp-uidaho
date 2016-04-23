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
/// @brief  Implementation of the BunnySock::UDP_socket class
/// @author Dave Billin <david.billin@vandals.uidaho.edu>
//=============================================================================

#include "sockets.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <sstream>
#include <cassert>
#include <cerrno>
#include <cstring>
#include "sockets.h"


using namespace BunnySock::Sockets;

//=============================================================================
inline static std::string uint16_to_string( uint16_t value )
{
   std::ostringstream oss;
   oss << static_cast<uint32_t>(value);
   std::string temp = oss.str();
   return temp;
}



namespace BunnySock { namespace Sockets
{

///////////////////////////////////////////////////////////////////////////////
Socket_exception::Socket_exception(char const* description)
  : m_description(description),
    m_errno(0)
{
}

///////////////////////////////////////////////////////////////////////////////
Socket_exception::Socket_exception( int errno_value, char const* prefix )
  : m_errno(errno_value)
{
   std::ostringstream oss;
   if ( NULL != prefix )
   {
      oss << prefix << ": ";
   }
   oss << strerror(errno);

   m_description = oss.str();
}

///////////////////////////////////////////////////////////////////////////////
/// Helper class that wraps the implementation of a Socket_address
///////////////////////////////////////////////////////////////////////////////
class Socket_address_impl
{
public:

   /// Creates an empty address object
   Socket_address_impl()
   {
      memset(&m_addr.ipv6, 0, sizeof(sockaddr_in6) );
   }

   //////////////////////////////////////////////////////////////////
   /// Creates the object from an existing socket address
   /// @param address   Socket address to copy
   Socket_address_impl( struct sockaddr const& address )
   {
      // Allocate for worst-case size for simpler copy/assignment
      memset(&m_addr.ipv6, 0, sizeof(sockaddr_in6) );
      copy_address(address);
   }

   //////////////////////////////////////////////////////////////////
   /// Called when the object goes out of scope
   ~Socket_address_impl()
   {}

   //////////////////////////////////////////////////////////////////
   /// Assignment operator overload
   Socket_address_impl& operator=( Socket_address_impl const& other )
   {
      if ( &other != this )
      {
         copy_address( other.addr() );
      }

      return *this;
   }

   //////////////////////////////////////////////////////////////////
   /// Helper function to copy a socket address struct
   /// @param address   Address to copy into the object
   void copy_address( struct sockaddr const& address )
   {
      switch (address.sa_family)
      {
         case AF_INET:  // IPv4
         {
            memcpy(&m_addr.ipv4,
                   reinterpret_cast<struct sockaddr_in const*>(&address),
                   sizeof(struct sockaddr_in));
            break;
         }

         case AF_INET6:  // IPv6
         {
            memcpy(&m_addr.ipv6,
                   reinterpret_cast<struct sockaddr_in6 const*>(&address),
                   sizeof(struct sockaddr_in6));
            break;
         }

         default:
         {
            assert(false); // Invalid address family!
            break;
         }
      }
   }


   //////////////////////////////////////////////////////////////////
   /// @return A reference to the object's socket address
   struct sockaddr const& addr() const
   { return m_addr.generic; }

   //////////////////////////////////////////////////////////////////
   /// @return A mutable reference to the object's socket address
   struct sockaddr& addr()
   { return m_addr.generic; }

   //////////////////////////////////////////////////////////////////
   /// @return The size of the socket address structure in Bytes
   socklen_t addr_len() const
   {
      return (AF_INET == m_addr.generic.sa_family) ?
         sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
   }

   //////////////////////////////////////////////////////////////////
   /// @return A reference to the network address as an IPv4 address
   struct in_addr const& ipv4_in_addr() const
   { return m_addr.ipv4.sin_addr; }

   //////////////////////////////////////////////////////////////////
   /// @return A reference to the network address as an IPv6 address
   struct in6_addr const& ipv6_in_addr() const
   { return m_addr.ipv6.sin6_addr; }


   //////////////////////////////////////////////////////////////////
   /// @return  The port number from the socket address
   uint16_t port() const
   {
      uint16_t port_number = 0;

      // Extract port number from the active socket address structure
      if ( AF_INET == m_addr.generic.sa_family )
      {
         port_number = m_addr.ipv4.sin_port;
      }
      else if ( AF_INET6 == m_addr.generic.sa_family )
      {
         port_number = m_addr.ipv6.sin6_port;
      }
      else
      {
         assert(false);    // Invalid socket family!
      }

      return port_number;
   }


   //////////////////////////////////////////////////////////////////
   /// Set the port number in the socket address
   /// @param value  Port number to assign
   void port( uint16_t value )
   {
      // Assign port number to the active socket address
      if ( AF_INET == m_addr.generic.sa_family )
      {
         m_addr.ipv4.sin_port = value;
      }
      else if ( AF_INET6 == m_addr.generic.sa_family )
      {
         m_addr.ipv6.sin6_port = value;
      }
      else
      {
         assert(false);    // Invalid socket family!
      }
   }


private:

   /// A union of all sockaddr pointer types for easy translation
   union Address_union
   {
      struct sockaddr     generic;
      struct sockaddr_in  ipv4;
      struct sockaddr_in6 ipv6;
   };

   Address_union m_addr;

};


///////////////////////////////////////////////////////////////////////////////
Socket_address const Socket_address::LOOPBACK("127.0.0.1");
Socket_address const Socket_address::GLOBAL_BROADCAST("255.255.255.255");
Socket_address const Socket_address::ANY_ADDRESS("0.0.0.0");


///////////////////////////////////////////////////////////////////////////////
Socket_address::Socket_address( std::string const& hostname, uint16_t port )
  : m_address_impl(NULL)
{
   struct addrinfo hints;
   struct addrinfo* servinfo = NULL;  // will point to the results

   memset(&hints, 0, sizeof(struct addrinfo)); // Initialize the struct
   hints.ai_family = AF_UNSPEC;       // don't care IPv4 or IPv6
   hints.ai_socktype = SOCK_STREAM;   // Assume TCP socket type
   hints.ai_flags = AI_PASSIVE;       // fill in my IP for me

   int status = getaddrinfo( hostname.c_str(),
                             (0 == port) ? NULL : uint16_to_string(port).c_str(),
                             &hints, &servinfo);
   if (0 != status)
   {
      // STATUS: failed to resolve the specified host address info
      freeaddrinfo(servinfo); // free heap memory pointed to by servinfo
      throw Socket_exception(gai_strerror(status));
   }

   // Validate address info length
   if ( AF_INET == servinfo->ai_addr->sa_family )
   {
      assert( sizeof(struct sockaddr_in) == servinfo->ai_addrlen );  // IPv4
   }
   else
   {
      assert( sizeof(struct sockaddr_in) == servinfo->ai_addrlen );  // IPv6
   }

   // Copy in the address
   m_address_impl = new Socket_address_impl(*servinfo->ai_addr);

   freeaddrinfo(servinfo); // free heap memory pointed to by servinfo
}

///////////////////////////////////////////////////////////////////////////////
Socket_address::Socket_address( Socket_address const& other )
  : m_address_impl(NULL)
{
   assert( NULL != other.m_address_impl );
   m_address_impl = new Socket_address_impl( other.address_impl().addr() );
}

///////////////////////////////////////////////////////////////////////////////
Socket_address::~Socket_address()
{
   delete m_address_impl;  // Delete the socket address
}

///////////////////////////////////////////////////////////////////////////////
Socket_address& Socket_address::operator=( Socket_address const& other )
{
   if ( &other != this )   // Bypass self-assignment
   {
      delete m_address_impl; // Delete existing address, then copy other's
      m_address_impl = other.m_address_impl;
   }

   return *this;
}

///////////////////////////////////////////////////////////////////////////////
std::string Socket_address::as_string() const
{
   assert( NULL != m_address_impl );
   return to_string( *m_address_impl );
}

///////////////////////////////////////////////////////////////////////////////
uint16_t Socket_address::port() const
{
   assert( NULL != m_address_impl );
   return m_address_impl->port();
}

///////////////////////////////////////////////////////////////////////////////
void Socket_address::port(uint16_t value)
{
   assert( NULL != m_address_impl );
   m_address_impl->port(value);
}

///////////////////////////////////////////////////////////////////////////////
struct Socket_address_impl const& Socket_address::address_impl() const
{
   assert( NULL != m_address_impl );
   return *m_address_impl;
}

///////////////////////////////////////////////////////////////////////////////
std::string Socket_address::to_string( Socket_address_impl const& addr_impl )
{
   char buf[INET6_ADDRSTRLEN];

   switch ( addr_impl.addr().sa_family )
   {
      case AF_INET:
      {
         inet_ntop(AF_INET, &addr_impl.ipv4_in_addr(), buf, INET6_ADDRSTRLEN);
         break;
      }
      case AF_INET6:
      {
         inet_ntop(AF_INET6, &addr_impl.ipv6_in_addr(), buf, INET6_ADDRSTRLEN);
         break;
      }

      default:
      {
         assert(false);    // m_addrinfo.ai_family is not initialized!
      }
   }

   return std::string(buf);
}

///////////////////////////////////////////////////////////////////////////////
UDP_socket::UDP_socket( Socket_address const& address )
  : m_should_broadcast( &Socket_address::GLOBAL_BROADCAST == &address )
{
   struct sockaddr const& sock_addr = address.address_impl().addr();
   m_socket_fd = socket( sock_addr.sa_family, SOCK_DGRAM, 0 );
   if ( -1 == m_socket_fd )
   {
      throw Socket_exception(errno);
   }

   // Bind to the specified address
   // If broadcast is enabled, bind to "any address"; otherwise bind to the specified address
   Socket_address_impl const& bind_addr = m_should_broadcast ?
         Socket_address::ANY_ADDRESS.address_impl() : address.address_impl();
   if ( -1 == bind(m_socket_fd, &bind_addr.addr(), bind_addr.addr_len()) )
   {
      throw Socket_exception(errno, "bind() failed on UDP socket");
   }

   // If the global broadcast address has been specified, make the socket
   // send UDP broadcast datagrams
   if (m_should_broadcast)
   {
      int broadcast = 1;
      int result = setsockopt( m_socket_fd, SOL_SOCKET, SO_BROADCAST,
                               &broadcast, sizeof(broadcast) );
      if ( -1 == result )
      {
         throw Socket_exception(errno, "Failed to enable UDP broadcast");
      }
   }
}

///////////////////////////////////////////////////////////////////////////////
UDP_socket::~UDP_socket()
{
   if ( INVALID_FILE_DESCRIPTOR != m_socket_fd )
   {
      ::close(m_socket_fd);  // Close the socket
   }
}

///////////////////////////////////////////////////////////////////////////////
void UDP_socket::sendto( Socket_address const& address, uint8_t const* buf,
                         size_t num_bytes )
{
   Socket_address_impl const& addr_impl = (m_should_broadcast) ?
                     Socket_address::GLOBAL_BROADCAST.address_impl() : address.address_impl();
   int result = ::sendto( m_socket_fd,
                          buf, num_bytes,
                          0,
                          &addr_impl.addr(), addr_impl.addr_len() );

   if (-1 == result)
   {
      throw Socket_exception(errno, "Failed to send UDP packet");
   }
}

///////////////////////////////////////////////////////////////////////////////
uint32_t UDP_socket::recvfrom( uint8_t* buf, size_t num_bytes,
                               Socket_address_impl const*& address )
{
   static Socket_address_impl sender_address;

   assert( NULL != buf );

   uint32_t num_bytes_received = 0;
   address = NULL;

   if ( (NULL != buf) && (num_bytes > 0) )
   {
      struct sockaddr_storage their_addr;
      socklen_t addr_len = sizeof(their_addr);
      int result = ::recvfrom( m_socket_fd,
                               reinterpret_cast<char*>(buf), num_bytes,
                               0,   // flags
                               reinterpret_cast<struct sockaddr*>(&their_addr),
                               &addr_len );
      if ( -1 != result )
      {
         sender_address.copy_address( reinterpret_cast<struct sockaddr&>(their_addr) );
         num_bytes_received = result;
         address = &sender_address;
      }
      else
      {
         throw Socket_exception(errno, "UDP recvfrom error");
      }
   }

   return num_bytes_received;
}

///////////////////////////////////////////////////////////////////////////////
void UDP_socket::should_broadcast( bool enable_broadcast )
{
   int broadcast = 1;
   if ( -1 == ::setsockopt( m_socket_fd, SOL_SOCKET, SO_BROADCAST,
                            &broadcast, sizeof(broadcast)) )
   {
      throw Socket_exception(errno, "Can't enable UDP broadcast");
   }
}

///////////////////////////////////////////////////////////////////////////////
bool UDP_socket::should_broadcast() const
{
   int optval;
   socklen_t optlen;
   ::getsockopt( m_socket_fd, SOL_SOCKET, SO_BROADCAST, &optval, &optlen);

   return (0 != optval);
}

}}  // END namespace BunnySock::Sockets
