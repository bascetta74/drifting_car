/*
 * udp_socket.c
 *
 *  Created on: Jan 3, 2018
 *      Author: Luca Bascetta
 */

#include "udp_socket.h"


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

void send_udpMessage(char message[UDP_TX_PACKET_MAX_SIZE])
{
	// Send a message over UDP
	Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
	Udp.write(message);
	Udp.endPacket();
}

void receive_udpMessage(char message[UDP_TX_PACKET_MAX_SIZE])
{
	// Receive a message over UDP (blocking call)
	int packetSize = Udp.parsePacket();
	do {
		if (packetSize) {
			// Read the packet into packetBufffer
			memset(message,'\0', UDP_TX_PACKET_MAX_SIZE);
			Udp.read(message, UDP_TX_PACKET_MAX_SIZE);
		}
		else {
			packetSize = Udp.parsePacket();
		}
	} while (packetSize<=0);
}
