//=============================================================================
// Unit tests of functions defined in parapet.cpp
//=============================================================================

#include "gtest/gtest.h"
#include "sockets.h"

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <stdint.h>

using namespace BunnySock::Sockets;

namespace UnitTest
{

struct Socket_accessor
{
   static int file_descriptor(UDP_socket const& target )
   { return target.m_socket_fd; }

};

}

//=============================================================================
TEST( Test_Address_info, address_constants )
{
   EXPECT_EQ( std::string("127.0.0.1"), Socket_address::LOOPBACK_IPv4.as_string() );
   EXPECT_EQ( 0, Socket_address::LOOPBACK_IPv4.port() );

   EXPECT_EQ( std::string("255.255.255.255"), Socket_address::GLOBAL_BROADCAST.as_string() );
   EXPECT_EQ( 0, Socket_address::GLOBAL_BROADCAST.port() );

   EXPECT_EQ( std::string("0.0.0.0"), Socket_address::IPv4_ANY_ADDRESS.as_string() );
   EXPECT_EQ( 0, Socket_address::IPv4_ANY_ADDRESS.port() );
}

//=============================================================================
TEST( Test_Address_info, test_udp_broadcast_socket )
{
   Socket_address::Network_port READER_PORT = 9998;

   Socket_address writer_address( Socket_address::LOOPBACK_IPv4,
                                  Socket_address::ANY_PORT );
   UDP_broadcast_socket writer( writer_address );

   Socket_address reader_address( Socket_address::IPv4_ANY_ADDRESS, READER_PORT );
   UDP_socket reader( reader_address );

   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(reader) );
   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(writer) );

   std::string message("Hello Wurld");

   // Write a broadcast message via the 'writer' socket
   writer.send( READER_PORT, reinterpret_cast<uint8_t const*>(message.c_str()),
                (message.size() + 1) );

   {
      // Verify that the 'reader' UDP socket received the message from 'writer'
      Socket_address sender_address(Socket_address::IPv4_ANY_ADDRESS,
                                    Socket_address::ANY_PORT);
      std::vector<char> received_msg(message.size() + 1, ' ');
      uint32_t num_bytes_received = reader.recvfrom( reinterpret_cast<uint8_t*>(&received_msg[0]),
                                                     received_msg.size(),
                                                     sender_address );
      EXPECT_EQ( (message.size() + 1), num_bytes_received );
      EXPECT_EQ( message, std::string(&received_msg[0]) );
      EXPECT_EQ( Socket_address::LOOPBACK_IPv4.as_string(),
                 sender_address.as_string() );
   }
}

//=============================================================================
TEST( Test_Address_info, test_udp_unicast_socket )
{
   Socket_address::Network_port WRITER_PORT = 9998;
   Socket_address::Network_port READER_PORT = 9999;

   Socket_address writer_address( Socket_address::LOOPBACK_IPv4.as_string(),
                                  WRITER_PORT );
   UDP_socket writer( writer_address );

   std::string const LOOPBACK_ADDRESS = Socket_address::LOOPBACK_IPv4.as_string();
   Socket_address reader_address( LOOPBACK_ADDRESS, READER_PORT );
   UDP_socket reader( reader_address );

   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(reader) );
   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(writer) );

   // Write a broadcast message via the 'writer' UDP socket
   std::string message("Hello Wurld");
   writer.sendto(reader_address, reinterpret_cast<uint8_t const*>(message.c_str()),
                 (message.size() + 1) );

   {
      // Verify that the 'reader' UDP socket received the message from 'writer'
      Socket_address sender_address(Socket_address::IPv4_ANY_ADDRESS,
                                    Socket_address::ANY_PORT);
      std::vector<char> received_msg(message.size() + 1, ' ');
      uint32_t num_bytes_received = reader.recvfrom( reinterpret_cast<uint8_t*>(&received_msg[0]),
                                                     received_msg.size(),
                                                     sender_address );
      EXPECT_EQ( (message.size() + 1), num_bytes_received );
      EXPECT_EQ( message, std::string(&received_msg[0]) );
      EXPECT_EQ( LOOPBACK_ADDRESS, sender_address.as_string() );
      EXPECT_EQ( WRITER_PORT, sender_address.port() );
   }
}
