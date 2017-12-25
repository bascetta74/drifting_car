/*
 * Socket.cpp
 *
 *  Created on: 14.11.2008
 *      Author:
 */

// Implementation of the Socket class.

#include "mocap_optitrack/socket_unicast.h"
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <sstream>

#include <ros/ros.h>

UdpUnicastSocket::UdpUnicastSocket( const int port, const std::string server_ip ) 
{
  // Create a UDP socket
  ROS_INFO( "Creating socket..." );
  m_socket = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
  if( m_socket < 0 ) {
    ROS_ERROR("Cannot create socket");
    //throw SocketException( strerror( errno ) );
  }

  // Fill struct for local address
  ROS_INFO( "Setting socket options..." );
  
  memset ( &m_server_addr, 0, sizeof ( m_server_addr ) );
  m_server_addr.sin_family = AF_INET;
  m_server_addr.sin_port = htons( port );
  if (inet_aton(server_ip.c_str() , &m_server_addr.sin_addr) == 0) 
  {
    std::stringstream error;
    error << "Failed to call to inet_aton";
  }
  ROS_INFO( "Server address: %s:%i", inet_ntoa( m_server_addr.sin_addr ), ntohs( m_server_addr.sin_port ) );
    
  // Make socket non-blocking
  ROS_INFO( "Enabling non-blocking I/O" );
  int flags = fcntl( m_socket, F_GETFL , 0 );
  if( fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) == -1 )
  {
    std::stringstream error;
    error << "Failed to enable non-blocking I/O: " << strerror( errno );
    //throw SocketException( error.str().c_str() );
  }

  // Send a message to the server
  memset ( buf, 0, MAXRECV + 1 );

  int addr_len = sizeof(struct sockaddr);
  int status = sendto(
    m_socket,
    buf,
    MAXRECV,
    0,
    (sockaddr *)&m_server_addr,
    (socklen_t) addr_len);

  if( status > 0 )
    ROS_INFO( "Initialization message sent to the server (%s:%i)", inet_ntoa( m_server_addr.sin_addr ), ntohs( m_server_addr.sin_port ));
  else
    ROS_INFO( "Connection closed by peer");
}

UdpUnicastSocket::~UdpUnicastSocket()
{
  close( m_socket );
}

int UdpUnicastSocket::recv()
{
  memset ( buf, 0, MAXRECV + 1 );

  int addr_len = sizeof(struct sockaddr);
  int status = recvfrom(
    m_socket,
    buf,
    MAXRECV,
    0,
    (sockaddr *)&m_server_addr,
    (socklen_t*)&addr_len);

  if( status > 0 )
    ROS_DEBUG( "%4i bytes received from %s:%i", status, inet_ntoa( m_server_addr.sin_addr ), ntohs( m_server_addr.sin_port ) );
  else if( status == 0 )
    ROS_INFO( "Connection closed by peer" );

  return status;
}
