/*
 * telemetry.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#include "telemetry.h"
#include "type.h"


// Internal functions
byte checksum_verify(const unsigned char checksum, const char* message, const size_t bufferSize);
unsigned char checksum_calculate(const char* message, const size_t bufferSize);


void encode_odroid_message(char encoded_message[MESSAGE_SIZE], const telemetry_message* message)
{
	// Code message data
	encoded_message[0] = 0x7E;
    memcpy(&(encoded_message[1]), &(message->steer_cmd), sizeof(unsigned int));
    memcpy(&(encoded_message[3]), &(message->speed_cmd), sizeof(unsigned int));
    memcpy(&(encoded_message[5]), &(message->wheel_dx_speed), sizeof(unsigned int));
    encoded_message[7] = message->wheel_dx_ccw;
    memcpy(&(encoded_message[8]), &(message->wheel_sx_speed), sizeof(unsigned int));
    encoded_message[10] = message->wheel_sx_ccw;
    encoded_message[11] = message->arduino_state;
    encoded_message[12] = message->arduino_state_info;
    encoded_message[13] = message->message_number;

    // Compute checksum
    encoded_message[MESSAGE_SIZE-1] = checksum_calculate(encoded_message, MESSAGE_SIZE);
}

unsigned int decode_odroid_message(const char message[MESSAGE_SIZE], telemetry_message* decoded_message)
{
	/* Verify message checksum and header it */
	if (!checksum_verify(message[MESSAGE_SIZE-1], message, MESSAGE_SIZE))
		return FAULT_COMM_CHECKSUM;
	else if (message[0]!=0x7E)
		return FAULT_COMM_HEADER;

	/* If everithing is ok decode the message */
	memcpy(&(decoded_message->steer_cmd), &(message[1]), sizeof(unsigned int));
	memcpy(&(decoded_message->speed_cmd), &(message[3]), sizeof(unsigned int));
	memcpy(&(decoded_message->wheel_dx_speed), &(message[5]), sizeof(unsigned int));
	decoded_message->wheel_dx_ccw = (bool) message[7];
	memcpy(&(decoded_message->wheel_sx_speed), &(message[8]), sizeof(unsigned int));
	decoded_message->wheel_sx_ccw = (bool) message[10];
	decoded_message->arduino_state = (unsigned char) message[11];
	decoded_message->arduino_state_info = (unsigned char) message[12];
	decoded_message->message_number = (unsigned char) message[13];

	return 0;
}

void initialize_odroid_message(telemetry_message* message)
{
	message->steer_cmd = message->speed_cmd = 0;
	message->wheel_dx_speed = message->wheel_sx_speed = 0;
	message->wheel_dx_ccw = message->wheel_sx_ccw = false;
	message->arduino_state = SAFE;
	message->arduino_state_info = 0;
	message->message_number = 0;
}

void set_odroid_message(telemetry_message* message, unsigned int steer_cmd, unsigned int speed_cmd,
		unsigned int wheel_dx_speed, bool wheel_dx_ccw, unsigned int wheel_sx_speed, bool wheel_sx_ccw,
		unsigned char arduino_state, unsigned char arduino_state_info, unsigned char message_number)
{
	message->steer_cmd          = steer_cmd;
	message->speed_cmd          = speed_cmd;
	message->wheel_dx_speed     = wheel_dx_speed;
	message->wheel_sx_speed     = wheel_sx_speed;
	message->wheel_dx_ccw       = wheel_dx_ccw;
	message->wheel_sx_ccw       = wheel_sx_ccw;
	message->arduino_state      = arduino_state;
	message->arduino_state_info = arduino_state_info;
	message->message_number     = message_number;
}

void get_odroid_message(const telemetry_message* message, unsigned int* steer_cmd, unsigned int* speed_cmd,
		unsigned int* wheel_dx_speed, bool* wheel_dx_ccw, unsigned int* wheel_sx_speed, bool* wheel_sx_ccw,
		unsigned char* arduino_state, unsigned char* arduino_state_info, unsigned char* message_number)
{
	*steer_cmd          = message->steer_cmd;
	*speed_cmd          = message->speed_cmd;
	*wheel_dx_speed     = message->wheel_dx_speed;
	*wheel_dx_ccw       = message->wheel_dx_ccw;
	*wheel_sx_speed     = message->wheel_sx_speed;
	*wheel_sx_ccw       = message->wheel_sx_ccw;
	*arduino_state      = message->arduino_state;
	*arduino_state_info = message->arduino_state_info;
	*message_number     = message->message_number;
}


// Internal functions
byte checksum_verify(const unsigned char checksum, const char* message, const size_t bufferSize)
{
	unsigned char result = 0;
	unsigned int sum = 0;

	for (unsigned int i = 0; i < (bufferSize - 1); i++)
		sum += message[i];
	result = sum & 0xFF;

	if (checksum == result)
		return 1;
	else
		return 0;
}

unsigned char checksum_calculate(const char* message, const size_t bufferSize)
{
  unsigned char result = 0;
  unsigned int sum = 0;

  for (unsigned int i = 0; i < (bufferSize - 1); i++)
    sum += message[i];
  result = sum & 0xFF;

  return result;
}
