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
inline static int socket_type_to_int( Address_info::Socket_type type)
{
   return (Address_info::UDP_SOCKET == type) ? SOCK_DGRAM : SOCK_STREAM;
}

//=============================================================================
inline static void copy_addrinfo(struct addrinfo& to_addr,
      struct addrinfo const& from_addr)
{
   memcpy(&to_addr, &from_addr, sizeof(struct addrinfo));

   if (0 != from_addr.ai_addrlen)
   {
      assert(NULL != from_addr.ai_addr);
      to_addr.ai_addr = reinterpret_cast<sockaddr*>(new uint8_t[from_addr.ai_addrlen]);
      memcpy(to_addr.ai_addr, from_addr.ai_addr, from_addr.ai_addrlen);
   }

   if ( NULL != from_addr.ai_canonname)
   {
      size_t canonname_len = strlen(from_addr.ai_canonname);
      to_addr.ai_canonname = new char[canonname_len + 1];
      strcpy(to_addr.ai_canonname, from_addr.ai_canonname);
   }
}

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

//=============================================================================
Socket_exception::Socket_exception(char const* description)
  : m_description(description),
    m_errno(0)
{
}

//=============================================================================
Socket_exception::Socket_exception( int errno_value, char const* prefix )
  : m_errno(errno_value)
{
   std::ostringstream oss;
   oss << ((NULL != prefix) ? prefix : "") << strerror(errno);
   m_description = oss.str();
}

//=============================================================================
/// Helper struct to keep socket headers out of sockets.h
struct addr_info
{
   struct addrinfo info;

};


Address_info const Address_info::LOOPBACK("127.0.0.1");

/// IPv4 UDP global broadcast address (255.255.255.255)
Address_info const Address_info::GLOBAL_BROADCAST("255.255.255.255");

/// Wildcard used to specify "any IP address"
Address_info const Address_info::ANY_ADDRESS("0.0.0.0");

//=============================================================================
Address_info::Address_info( std::string const& hostname, uint16_t port )
  : m_addr_info( new addr_info )
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

   copy_addrinfo(m_addr_info->info, *servinfo);      // Copy in address info

   freeaddrinfo(servinfo); // free heap memory pointed to by servinfo
}

//=============================================================================
Address_info::Address_info( Address_info const& other )
  : m_addr_info( new addr_info )
{
   memset(&m_addr_info, 0, sizeof(struct addrinfo)); // Initialize the struct

   // Copy the address info
   assert( NULL != other.m_addr_info );
   copy_addrinfo(m_addr_info->info, other.m_addr_info->info);
}

//=============================================================================
Address_info::~Address_info()
{
   delete_info();       // Delete heap memory in info
   delete m_addr_info;  // Delete the info
}

//=============================================================================
Address_info& Address_info::operator=( Address_info const& other )
{
   if ( &other != this )   // Bypass self-assignment
   {
      delete_info();    // Delete existing info, then copy other's info
      copy_addrinfo(m_addr_info->info, other.m_addr_info->info);
   }

   return *this;
}

//=============================================================================
std::string Address_info::as_string() const
{
   // Make sure the address is valid
   throw_if_invalid();

   struct addrinfo& info = m_addr_info->info;

   char buf[INET6_ADDRSTRLEN];
   struct sockaddr const* sa = info.ai_addr;

   switch (info.ai_addr->sa_family)
   {
      case AF_INET:
      {
         inet_ntop(AF_INET, &(((struct sockaddr_in *)sa)->sin_addr), buf, INET6_ADDRSTRLEN);
         break;
      }
      case AF_INET6:
      {
         inet_ntop(AF_INET6, &(((struct sockaddr_in6 *) sa)->sin6_addr), buf, INET6_ADDRSTRLEN);
         break;
      }

      default:
      {
         assert(false);    // m_addrinfo.ai_family is not initialized!
      }
   }

   return std::string(buf);
}


//=============================================================================
Address_info::Socket_type Address_info::socket_type() const
{
   throw_if_invalid();
   return (SOCK_DGRAM == m_addr_info->info.ai_socktype) ? UDP_SOCKET : TCP_SOCKET;
}

//=============================================================================
void Address_info::socket_type( Address_info::Socket_type type )
{
   m_addr_info->info.ai_socktype = socket_type_to_int(type);
}

//=============================================================================
uint16_t Address_info::port() const
{
   throw_if_invalid();  // Make sure address info is valid

   struct addrinfo const& info = m_addr_info->info;
   uint16_t port_number = 0;

   // Port number is extracted from the (family-dependent) socket address structure
   switch (info.ai_addr->sa_family)
   {
      case AF_INET:     // IPv4
      {
         assert( sizeof(sockaddr_in) == info.ai_addrlen );
         struct sockaddr_in const* address_in =
                     reinterpret_cast<struct sockaddr_in const*>(info.ai_addr);
         port_number = address_in->sin_port;
         break;
      }

      case AF_INET6:    // IPv6
      {
         assert( sizeof(sockaddr_in6) == info.ai_addrlen );
         struct sockaddr_in6 const* address_in =
                     reinterpret_cast<struct sockaddr_in6 const*>(info.ai_addr);
         port_number = address_in->sin6_port;
         break;
      }

      default:
      {
         assert(false); // Invalid socket family!
         break;
      }
   }

   return port_number;
}

//=============================================================================
void Address_info::port(uint16_t value)
{
   m_addr_info->info.ai_protocol = value;
}

//=============================================================================
struct addr_info const& Address_info::addrinfo() const
{
   throw_if_invalid();
   return *m_addr_info;
}

//=============================================================================
void Address_info::delete_info()
{
   assert( NULL != m_addr_info );
   struct addrinfo& info = m_addr_info->info;

   if ( NULL != info.ai_addr)      // Free struct sockaddr heap memory
   {
      delete info.ai_addr;
      info.ai_addr = NULL;
   }

   if ( NULL != info.ai_canonname) // Free canonical name heap memory
   {
      delete[] info.ai_canonname;
      info.ai_canonname = NULL;
   }
}

//=============================================================================
void Address_info::throw_if_invalid() const
{
   // Make sure address info is valid
   if ( NULL == m_addr_info->info.ai_addr )
   {
      throw Socket_exception("Address_info is not initialized");
   }
}



//=============================================================================
UDP_socket::UDP_socket( Address_info const& address )
  : m_should_broadcast( &Address_info::GLOBAL_BROADCAST == &address )
{
   struct addrinfo const& addrinfo = address.addrinfo().info;
   m_socket_fd = socket( addrinfo.ai_family, addrinfo.ai_socktype,
                         addrinfo.ai_protocol );
   if ( -1 == m_socket_fd )
   {
      throw Socket_exception(errno);
   }

   // Bind to the specified address
   // If broadcast is enabled, bind to "any address"; otherwise bind to the specified address
   Address_info const& bind_addr = m_should_broadcast ? Address_info::ANY_ADDRESS : address;
   struct addrinfo const& bindaddrinfo = bind_addr.addrinfo().info;
   if ( -1 == bind(m_socket_fd, bindaddrinfo.ai_addr, bindaddrinfo.ai_addrlen) )
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

//=============================================================================
UDP_socket::~UDP_socket()
{
   if ( INVALID_FILE_DESCRIPTOR != m_socket_fd )
   {
      ::close(m_socket_fd);  // Close the socket
   }
}

//=============================================================================
void UDP_socket::sendto( Address_info const& address, uint8_t const* data,
                         size_t num_bytes )
{
   struct addrinfo const& info = (m_should_broadcast) ?
                     address.addrinfo().info : Address_info::GLOBAL_BROADCAST.addrinfo().info;
   int result = ::sendto( m_socket_fd,
                          data, num_bytes,
                          0,
                          info.ai_addr, info.ai_addrlen );

   if (-1 == result)
   {
      throw Socket_exception(errno, "Failed to send UDP packet");
   }
}

}
}  // END namespace BunnySock::Sockets
