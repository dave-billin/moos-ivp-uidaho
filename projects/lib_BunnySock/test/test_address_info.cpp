//=============================================================================
// Unit tests of functions defined in parapet.cpp
//=============================================================================

#include "gtest/gtest.h"
#include "sockets.h"

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
   EXPECT_EQ( std::string("127.0.0.1"), Address_info::LOOPBACK.as_string() );
   EXPECT_EQ( 0, Address_info::LOOPBACK.port() );
   EXPECT_EQ( Address_info::TCP_SOCKET, Address_info::LOOPBACK.socket_type() );

   EXPECT_EQ( std::string("255.255.255.255"), Address_info::GLOBAL_BROADCAST.as_string() );
   EXPECT_EQ( 0, Address_info::GLOBAL_BROADCAST.port() );
   EXPECT_EQ( Address_info::TCP_SOCKET, Address_info::LOOPBACK.socket_type() );

   EXPECT_EQ( std::string("0.0.0.0"), Address_info::ANY_ADDRESS.as_string() );
   EXPECT_EQ( 0, Address_info::ANY_ADDRESS.port() );
   EXPECT_EQ( Address_info::TCP_SOCKET, Address_info::LOOPBACK.socket_type() );
}

//=============================================================================
TEST( Test_Address_info, test_udp_socket )
{
   Address_info reader_address( Address_info::LOOPBACK.as_string(), 0 );
   UDP_socket writer( Address_info::LOOPBACK );

   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(writer) );

}
