#ifndef TELEMETRYPROTOCOL_H_
#define TELEMETRYPROTOCOL_H_

#include <Arduino.h>

#include "constant.h"

typedef struct
{
	unsigned int steer_cmd;
	unsigned int speed_cmd;
	unsigned int wheel_dx_speed;
	bool wheel_dx_ccw;
	unsigned int wheel_sx_speed;
	bool wheel_sx_ccw;
	byte arduino_state;
	byte arduino_state_info;
	byte message_number;
} telemetry_message;

class telemetryProtocol {
public:
	static const byte CMD_BEGIN_IND = 0xFE;
	static const byte CMD_END_IND = 0xFF;
	
	byte cmdBufferIdx;
	byte buf[MESSAGE_SIZE-2];
	boolean cmdBegan;

	byte sending_message[MESSAGE_SIZE];
	telemetry_message received_message;
	bool new_message_available;

	byte cmdBeginInd;
	byte cmdEndInd;

	telemetryProtocol();
	telemetryProtocol(byte cmdBeginIndicator, byte cmdEndIndicator);
	
	void receive();
	bool send(const telemetry_message* message);

	void read_message(telemetry_message* message);
	bool message_available();

	void init_message(telemetry_message* message);
	void set_message(telemetry_message* message, unsigned int steer_cmd, unsigned int speed_cmd,
			unsigned int wheel_dx_speed, bool wheel_dx_ccw, unsigned int wheel_sx_speed, bool wheel_sx_ccw,
			byte arduino_state, byte arduino_state_info, byte message_number);
	void get_message(const telemetry_message* message, unsigned int* steer_cmd, unsigned int* speed_cmd,
			unsigned int* wheel_dx_speed, bool* wheel_dx_ccw, unsigned int* wheel_sx_speed, bool* wheel_sx_ccw,
			byte* arduino_state, byte* arduino_state_info, byte* message_number);

private:
	void decode_message(byte* message, byte messageLength);
	void unsignedInt2buffer(unsigned int value, byte* buffer);
	void buffer2unsignedInt(const byte* buffer, unsigned int* value);
};

#endif /* TELEMETRYPROTOCOL_H_ */
