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
void send_udpMessage(char message[UDP_TX_PACKET_MAX_SIZE]);
void receive_udpMessage(char message[UDP_TX_PACKET_MAX_SIZE]);

#endif /* UDP_SOCKET_H_ */
