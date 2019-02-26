#include "telemetryProtocol.h"
#include "type.h"


telemetryProtocol::telemetryProtocol()
{
	cmdBufferIdx = 0;
	cmdBegan = false;
	
	cmdBeginInd = CMD_BEGIN_IND;
	cmdEndInd = CMD_END_IND;

	new_message_available = false;
}

telemetryProtocol::telemetryProtocol(byte cmdBeginIndicator, byte cmdEndIndicator)
{
	cmdBufferIdx = 0;
	cmdBegan = false;
	
	cmdBeginInd = cmdBeginIndicator;
	cmdEndInd = cmdEndIndicator;

	new_message_available = false;
}

void telemetryProtocol::receive()
{
	while (Serial.available()) {
		byte b=Serial.read();

		if (b == cmdBeginInd)
		{
			cmdBegan = true;
			cmdBufferIdx = 0;
		}
		else if (b == cmdEndInd)
		{
			cmdBegan = false;
			new_message_available = true;

			decode_message(buf, cmdBufferIdx);
		}
		else
		{
			if (cmdBegan) {
				if (cmdBufferIdx < MESSAGE_SIZE-2) {
					buf[cmdBufferIdx] = b;
					cmdBufferIdx++;
				}
			}
		}
	}
}

bool telemetryProtocol::send(const telemetry_message* message)
{
	sending_message[0] = cmdBeginInd;
	unsignedInt2buffer(message->steer_cmd, &(sending_message[1]));
	unsignedInt2buffer(message->speed_cmd, &(sending_message[3]));
	unsignedInt2buffer(message->wheel_dx_speed, &(sending_message[5]));
    sending_message[7] = message->wheel_dx_ccw;
	unsignedInt2buffer(message->wheel_sx_speed, &(sending_message[8]));
    sending_message[10] = message->wheel_sx_ccw;
    sending_message[11] = message->arduino_state;
    sending_message[12] = message->arduino_state_info;
    sending_message[13] = message->message_number;
    sending_message[MESSAGE_SIZE-1] = cmdEndInd;

    byte sent_bytes = 0;
	for (byte i=0; i<MESSAGE_SIZE; i++) {
		sent_bytes += Serial.write(sending_message[i]);
	}

	if (sent_bytes != MESSAGE_SIZE)
		return false;

	return true;
}

void telemetryProtocol::read_message(telemetry_message* message)
{
    message->steer_cmd          = received_message.steer_cmd;
    message->speed_cmd          = received_message.speed_cmd;
    message->wheel_dx_speed     = received_message.wheel_dx_speed;
    message->wheel_dx_ccw       = received_message.wheel_dx_ccw;
    message->wheel_sx_speed     = received_message.wheel_sx_speed;
    message->wheel_sx_ccw       = received_message.wheel_sx_ccw;
    message->arduino_state      = received_message.arduino_state;
    message->arduino_state_info = received_message.arduino_state_info;
    message->message_number     = received_message.message_number;

    new_message_available = false;
}

bool telemetryProtocol::message_available()
{
	return new_message_available;
}

void telemetryProtocol::init_message(telemetry_message* message)
{
	message->steer_cmd = message->speed_cmd = 0;
	message->wheel_dx_speed = message->wheel_sx_speed = 0;
	message->wheel_dx_ccw = message->wheel_sx_ccw = false;
	message->arduino_state = SAFE;
	message->arduino_state_info = 0;
	message->message_number = 0;
}

void telemetryProtocol::set_message(telemetry_message* message, unsigned int steer_cmd, unsigned int speed_cmd,
		unsigned int wheel_dx_speed, bool wheel_dx_ccw, unsigned int wheel_sx_speed, bool wheel_sx_ccw,
		byte arduino_state, byte arduino_state_info, byte message_number)
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

void telemetryProtocol::get_message(const telemetry_message* message, unsigned int* steer_cmd, unsigned int* speed_cmd,
		unsigned int* wheel_dx_speed, bool* wheel_dx_ccw, unsigned int* wheel_sx_speed, bool* wheel_sx_ccw,
		byte* arduino_state, byte* arduino_state_info, byte* message_number)
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
void telemetryProtocol::decode_message(byte* message, byte messageLength)
{
	if (messageLength != MESSAGE_SIZE-2) {
		received_message.steer_cmd = 0;
		received_message.speed_cmd = 0;
		received_message.wheel_dx_speed = 0;
		received_message.wheel_dx_ccw = false;
		received_message.wheel_sx_speed = 0;
		received_message.wheel_sx_ccw = false;
		received_message.arduino_state = 0;
		received_message.arduino_state_info = 0;
		received_message.message_number = 0;

		new_message_available = false;
	}
	else {
		buffer2unsignedInt(&(message[0]), &(received_message.steer_cmd));
		buffer2unsignedInt(&(message[2]), &(received_message.speed_cmd));
		buffer2unsignedInt(&(message[4]), &(received_message.wheel_dx_speed));
		received_message.wheel_dx_ccw = (bool) message[6];
		buffer2unsignedInt(&(message[7]), &(received_message.wheel_sx_speed));
		received_message.wheel_sx_ccw = (bool) message[9];
		received_message.arduino_state = (byte) message[10];
		received_message.arduino_state_info = (byte) message[11];
		received_message.message_number = (byte) message[12];
	}
}

void telemetryProtocol::unsignedInt2buffer(unsigned int value, byte* buffer)
{
	buffer[0] = (value >> 8) & 0xFF;
	buffer[1] = value & 0xFF;
}

void telemetryProtocol::buffer2unsignedInt(const byte* buffer, unsigned int* value)
{
	*value = (buffer[0] << 8) | buffer[1];
}
