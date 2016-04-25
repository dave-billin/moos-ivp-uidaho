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
inline static std::string port_number_to_string(
                                       Socket_address::Network_port value )
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
      memset(&m_addr, 0, sizeof(Address_union) );
   }

   //////////////////////////////////////////////////////////////////
   /// Creates the object from an existing socket address
   /// @param address   address structure to copy (IPv4 or IPv6)
   Socket_address_impl( struct sockaddr const& address )
   {
      memset(&m_addr, 0, sizeof(Address_union) );
      copy_address(address);
   }

   //////////////////////////////////////////////////////////////////
   /// Creates the object from an existing IPv4 socket address
   /// @param ipv4_address   IPv4 address to copy
   Socket_address_impl( struct in_addr const& ipv4_address )
   {
      memset(&m_addr, 0, sizeof(Address_union) );
      memcpy(&m_addr.ipv4.sin_addr, &ipv4_address, sizeof(struct in_addr));
      m_addr.ipv4.sin_family = AF_INET;
   }

   //////////////////////////////////////////////////////////////////
   /// Creates the object from an existing IPv6 socket address
   /// @param address   IPv6 address to copy
   Socket_address_impl( struct in6_addr const& ipv6_address )
   {
      memset(&m_addr, 0, sizeof(Address_union) );
      memcpy(&m_addr.ipv6.sin6_addr, &ipv6_address, sizeof(struct in6_addr));
      m_addr.ipv6.sin6_family = AF_INET6;
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
   Socket_address::Network_port port() const
   {
      Socket_address::Network_port port_number = 0;

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

      return ntohs(port_number);
   }


   //////////////////////////////////////////////////////////////////
   /// Set the port number in the socket address
   /// @param value  Port number to assign
   void port( Socket_address::Network_port value )
   {
      uint16_t ns_value = htons(value);

      // Assign port number to the active socket address
      if ( AF_INET == m_addr.generic.sa_family )
      {
         m_addr.ipv4.sin_port = ns_value;
      }
      else if ( AF_INET6 == m_addr.generic.sa_family )
      {
         m_addr.ipv6.sin6_port = ns_value;
      }
      else
      {
         assert(false);    // Invalid socket family!
      }
   }

   //////////////////////////////////////////////////////////////////
   /// @return true if the address is an IPv4 address; else false if
   ///         it is IPv6
   bool is_ipv4() const
   {
      return (AF_INET == m_addr.generic.sa_family);
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
Socket_address const Socket_address::LOOPBACK_IPv4("127.0.0.1");
Socket_address const Socket_address::LOOPBACK_IPv6(in6addr_loopback);
Socket_address const Socket_address::IPv4_ANY_ADDRESS("0.0.0.0");
Socket_address const Socket_address::IPv6_ANY_ADDRESS(in6addr_any);
Socket_address const Socket_address::GLOBAL_BROADCAST("255.255.255.255");


///////////////////////////////////////////////////////////////////////////////
Socket_address::Socket_address( std::string const& hostname, Network_port port )
  : m_address_impl(NULL)
{
   struct addrinfo hints;
   struct addrinfo* servinfo = NULL;  // will point to the results

   memset(&hints, 0, sizeof(struct addrinfo)); // Initialize the struct
   hints.ai_family = AF_UNSPEC;       // don't care IPv4 or IPv6
   hints.ai_socktype = SOCK_STREAM;   // Assume TCP socket type
   hints.ai_flags = AI_PASSIVE;       // fill in my IP for me

   int status = getaddrinfo( hostname.c_str(),
                             (0 == port) ? NULL : port_number_to_string(port).c_str(),
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

Socket_address::Socket_address( Socket_address const& address_obj,
                                Network_port port )
  : m_address_impl(NULL)
{
   assert( NULL != address_obj.m_address_impl );
   m_address_impl = new Socket_address_impl( address_obj.impl() );

   if ( ANY_PORT != port )
   {
      m_address_impl->port(port);   // Override address_obj's network port
   }
}

#if 0
///////////////////////////////////////////////////////////////////////////////
Socket_address::Socket_address( Socket_address const& other )
  : m_address_impl(NULL)
{
   assert( NULL != other.m_address_impl );
   m_address_impl = new Socket_address_impl( other.impl().addr() );
}
#endif

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
   return impl_address( *m_address_impl );
}

///////////////////////////////////////////////////////////////////////////////
Socket_address::Network_port Socket_address::port() const
{
   assert( NULL != m_address_impl );
   return m_address_impl->port();
}

///////////////////////////////////////////////////////////////////////////////
void Socket_address::port(Network_port value)
{
   assert( NULL != m_address_impl );
   m_address_impl->port(value);
}

///////////////////////////////////////////////////////////////////////////////
struct Socket_address_impl const& Socket_address::impl() const
{
   assert( NULL != m_address_impl );
   return *m_address_impl;
}

///////////////////////////////////////////////////////////////////////////////
struct Socket_address_impl& Socket_address::impl()
{
   return *const_cast<Socket_address*>(this)->m_address_impl;
}

///////////////////////////////////////////////////////////////////////////////
void Socket_address::impl_address( Socket_address_impl const& impl,
                                   std::string& address )
{
   char buf[INET6_ADDRSTRLEN];

   switch ( impl.addr().sa_family )
   {
      case AF_INET:
      {
         inet_ntop(AF_INET, &impl.ipv4_in_addr(), buf, INET6_ADDRSTRLEN);
         break;
      }
      case AF_INET6:
      {
         inet_ntop(AF_INET6, &impl.ipv6_in_addr(), buf, INET6_ADDRSTRLEN);
         break;
      }

      default:
      {
         assert(false);    // m_addrinfo.ai_family is not initialized!
      }
   }

   address = buf; // Return the address in the provided string object
}

///////////////////////////////////////////////////////////////////////////////
Socket_address::Network_port Socket_address::impl_port(
                                             Socket_address_impl const& impl )
{
   return impl.port();
}

///////////////////////////////////////////////////////////////////////////////
Socket_address::Socket_address( Socket_address_impl const& impl )
{
   m_address_impl = new Socket_address_impl(impl);
}


///////////////////////////////////////////////////////////////////////////////
UDP_socket::UDP_socket( Socket_address const& address )
{
   struct sockaddr const& sock_addr = address.impl().addr();
   m_socket_fd = socket( sock_addr.sa_family, SOCK_DGRAM, 0 );
   if ( -1 == m_socket_fd )
   {
      throw Socket_exception(errno);
   }

   // Bind to the specified address
   Socket_address_impl const& bind_addr = address.impl();
   if ( -1 == bind(m_socket_fd, &bind_addr.addr(), bind_addr.addr_len()) )
   {
      throw Socket_exception(errno, "bind() failed on UDP socket");
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
   Socket_address_impl const& addr_impl = address.impl();
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
                               Socket_address& sender_address )
{
   assert( NULL != buf );

   uint32_t num_bytes_received = 0;

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
         sender_address.impl().copy_address(
                              reinterpret_cast<struct sockaddr&>(their_addr) );
         num_bytes_received = result;
      }
      else
      {
         throw Socket_exception(errno, "UDP recvfrom error");
      }
   }

   return num_bytes_received;
}

///////////////////////////////////////////////////////////////////////////////
UDP_broadcast_socket::UDP_broadcast_socket( Socket_address const& address )
  : UDP_socket(address),
    m_broadcast_address( Socket_address::GLOBAL_BROADCAST,
                         Socket_address::ANY_PORT )
{
   // Enable the socket to send UDP broadcast datagrams
   int broadcast = 1;
   int result = setsockopt( socket_fd(), SOL_SOCKET, SO_BROADCAST,
                            &broadcast, sizeof(broadcast) );
   if ( -1 == result )
   {
      throw Socket_exception(errno, "Failed to enable UDP broadcast");
   }

}

///////////////////////////////////////////////////////////////////////////////
void UDP_broadcast_socket::send( Socket_address::Network_port port,
                                 uint8_t const* buf, size_t num_bytes )
{
   m_broadcast_address.port(port); // Set the port to broadcast on
   UDP_socket::sendto(m_broadcast_address, buf, num_bytes); // Send it
}

}}  // END namespace BunnySock::Sockets
