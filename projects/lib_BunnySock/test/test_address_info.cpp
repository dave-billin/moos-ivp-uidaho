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
   EXPECT_EQ( std::string("127.0.0.1"), Socket_address::LOOPBACK.as_string() );
   EXPECT_EQ( 0, Socket_address::LOOPBACK.port() );

   EXPECT_EQ( std::string("255.255.255.255"), Socket_address::GLOBAL_BROADCAST.as_string() );
   EXPECT_EQ( 0, Socket_address::GLOBAL_BROADCAST.port() );

   EXPECT_EQ( std::string("0.0.0.0"), Socket_address::ANY_ADDRESS.as_string() );
   EXPECT_EQ( 0, Socket_address::ANY_ADDRESS.port() );
}

//=============================================================================
TEST( Test_Address_info, test_udp_socket )
{
   Socket_address reader_address( Socket_address::LOOPBACK.as_string(), 0 );

   UDP_socket writer( Socket_address::LOOPBACK );

   EXPECT_NE( -1, UnitTest::Socket_accessor::file_descriptor(writer) );

}
