/*
 * udp_socket.c
 *
 *  Created on: Jan 3, 2018
 *      Author: Luca Bascetta
 */

#include "udp_socket.h"

#include <avr/interrupt.h>


// Internal variables
EthernetUDP Udp;							// UDP object
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];	// Buffer for incoming messages


void init_udpConnection(byte mac_address[], IPAddress ip_address, unsigned int port)
{
	// Start Ethernet and UDP
	Ethernet.begin(mac_address, ip_address);
	Udp.begin(port);
}

void wait_client_connection()
{
	int packetSize = 0;

	// Wait for connection of a client
	do {
		packetSize = Udp.parsePacket();

		if (packetSize) {
			// Read the packet into packetBufffer
			memset(packetBuffer,'\0', UDP_TX_PACKET_MAX_SIZE);
			Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);

			// Send an ACK over UDP
			Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
			Udp.write(packetBuffer);
			Udp.endPacket();
		}
	} while (packetSize<=0);
}

size_t send_udpMessage(const uint8_t *buffer, size_t buf_size)
{
	// Send a message over UDP
	Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
	size_t byte_sent = Udp.write(buffer, buf_size);
	Udp.endPacket();

	return byte_sent;
}

size_t receive_udpMessage(uint8_t *buffer, size_t buf_size)
{
	size_t byte_read = 0;

	// Receive a message over UDP (blocking call)
	int packetSize = Udp.parsePacket();
	do {
		if (packetSize>=buf_size) {
			// Read the packet into packetBufffer
			memset(buffer,'\0', buf_size);
			byte_read = Udp.read(buffer, buf_size);
		}
		else {
			packetSize = Udp.parsePacket();
		}
	} while (byte_read<buf_size);

	return byte_read;
}
