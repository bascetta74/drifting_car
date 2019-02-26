#ifndef TELEMETRYPROTOCOL_H
#define TELEMETRYPROTOCOL_H

#include <cstdint>
#include <serial/serial.h>

typedef struct
{
	unsigned int steer_cmd;
	unsigned int speed_cmd;
	unsigned int wheel_dx_speed;
	bool wheel_dx_ccw;
	unsigned int wheel_sx_speed;
	bool wheel_sx_ccw;
	uint8_t arduino_state;
	uint8_t arduino_state_info;
	uint8_t message_number;
} telemetry_message;


class telemetryProtocol {
public:
	static const uint8_t CMD_BEGIN_IND = 0xFE;
	static const uint8_t CMD_END_IND = 0xFF;
	
	int cmdBufferIdx;
	uint8_t *buf;
	bool cmdBegan;

	uint8_t message_size;
	uint8_t *sending_message;
	telemetry_message received_message;
	bool new_message_available;

	uint8_t cmdBeginInd;
	uint8_t cmdEndInd;

	serial::Serial *serialStream;

	telemetryProtocol(serial::Serial *ss, uint8_t message_size);
	telemetryProtocol(serial::Serial *ss, uint8_t message_size, uint8_t cmdBeginIndicator, uint8_t cmdEndIndicator);
	~telemetryProtocol();

	void receive();
	bool send(const telemetry_message* message);

	void read_message(telemetry_message* message);
	bool message_available();

	void init_message(telemetry_message* message);
	void set_message(telemetry_message* message, unsigned int steer_cmd, unsigned int speed_cmd,
			unsigned int wheel_dx_speed, bool wheel_dx_ccw, unsigned int wheel_sx_speed, bool wheel_sx_ccw,
			uint8_t arduino_state, uint8_t arduino_state_info, uint8_t message_number);
	void get_message(const telemetry_message* message, unsigned int* steer_cmd, unsigned int* speed_cmd,
			unsigned int* wheel_dx_speed, bool* wheel_dx_ccw, unsigned int* wheel_sx_speed, bool* wheel_sx_ccw,
			uint8_t* arduino_state, uint8_t* arduino_state_info, uint8_t* message_number);

private:
	void decode_message(const uint8_t* message, int messageLength);
	void unsignedInt2buffer(unsigned int value, uint8_t* buffer);
	void buffer2unsignedInt(const uint8_t* buffer, unsigned int* value);
};

#endif
