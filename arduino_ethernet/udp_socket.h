/*
 * udp_socket.h
 *
 *  Created on: Jan 3, 2018
 *      Author: Luca Bascetta
 */

#ifndef UDP_SOCKET_H_
#define UDP_SOCKET_H_

#include <Ethernet.h>
#include <EthernetUdp.h>

void init_udpConnection(byte mac_address[], IPAddress ip_address, unsigned int port);
void wait_client_connection();
size_t send_udpMessage(const uint8_t *buffer, size_t buf_size);
size_t receive_udpMessage(uint8_t *buffer, size_t buf_size);

#endif /* UDP_SOCKET_H_ */
