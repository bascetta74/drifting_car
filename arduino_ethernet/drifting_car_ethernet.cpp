// Do not remove the include below
#include "drifting_car_ethernet.h"

#include "constant.h"
#include "type.h"
#include "task_timer.h"
#include "udp_socket.h"
#include "telemetry.h"
#include "receiver.h"
#include "encoder.h"
#include "car_commands.h"

#include <stdio.h>
#include <avr/wdt.h>

#define DEBUG

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup/Loop global variables                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Server MAC, IP address and port
byte			server_mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress		server_ip(192, 168, 1, 177);
unsigned int	server_port = 1511;

// Transmission
char message_in[MESSAGE_SIZE], message_out[MESSAGE_SIZE];
telemetry_message from_odroid, to_odroid;

// Car measures
unsigned int curr_speed, curr_steer;

// Encoder measures
int sx_count, dx_count;

// State machine
state_info arduino_state;

// Errors
unsigned int message_decode_error;

// Time
unsigned int arduino_start_time;
/////////////////////////////////////////////////////////////////////////////////////////////////////


//The setup function is called once at startup of the sketch
void setup()
{
	// Disable watchdog
	wdt_disable();

	// Initialize global variables
	for (int k=0; k<MESSAGE_SIZE; k++)
		message_in[k] = message_out[k] = 0;

	curr_speed = curr_steer = 0;

	// Initialize state machine
	arduino_state.state = SAFE;
	arduino_state.info = 0;

	// Initialize error messages
	message_decode_error = 0;

	// Initialize receiver
	init_receiverIO();
	init_receiverInterrupts();

	// Initialize encoder
	init_encoderIO();
	init_encoderInterrupts();

	// Initialize steer and speed
	init_carcommandsIO();

	// Initialize telemetry messages
	initialize_odroid_message(&from_odroid);
	initialize_odroid_message(&to_odroid);

#ifdef DEBUG
	// Open serial communication for debugging
	Serial.begin(57600);
	while (!Serial) {
		; // wait for serial port to connect
	}
#endif

	// Start UDP connection
	init_udpConnection(server_mac, server_ip, server_port);

#ifdef DEBUG
	Serial.print("UDP connection started, waiting for client...\n");
#endif

	// Wait for connection of a client
	wait_client_connection();

#ifdef DEBUG
	Serial.print("Client connected, starting loop...\n");
#endif

	// Enable watchdog for connection lost
	wdt_enable(WDTO_120MS);

	// Start loop timer
	init_taskTimer(LOOP_FREQUENCY);

	// Initialize arduino start time
	arduino_start_time = micros();
}

// The loop function is called in an endless loop
void loop()
{
	if (canStart() & (arduino_state.state != HALT))
	{
		// Receive e message from Odroid
		receive_udpMessage(message_in);

		// Decode the message
		message_decode_error = decode_odroid_message(message_in, &from_odroid);
		if (message_decode_error > 0)
		{
//			arduino_state.state = HALT;
//			arduino_state.info = message_decode_error;
		}

		// Get current measures from the radio
		curr_steer  = get_steer_value_us();
		curr_speed  = get_speed_value_us();

		// Update the state machine state
		if (is_receiver_button_pressed())
		{
			if (arduino_state.state == SAFE)
			{
				arduino_state.state = MANUAL;
				arduino_state.info  = 0;
			}
			else if (arduino_state.state == MANUAL)
			{
				arduino_state.state = AUTOMATIC;
				arduino_state.info  = 0;
			}
			else if (arduino_state.state == AUTOMATIC)
			{
				arduino_state.state = MANUAL;
				arduino_state.info  = 0;
			}
		}

		// Execute the current state task
		switch (arduino_state.state)
		{
		case SAFE:
			set_steer(STEER_ZERO);
			set_speed(SPEED_ZERO);

			// Prepare the message to be sent
			sx_count = get_wheelcount_sx();
			dx_count = get_wheelcount_dx();
			to_odroid.steer_cmd          = STEER_ZERO;
			to_odroid.speed_cmd          = SPEED_ZERO;
			to_odroid.wheel_dx_speed     = abs(dx_count);
			to_odroid.wheel_sx_speed     = abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			to_odroid.arduino_time       = (unsigned long) micros()-arduino_start_time;
			break;

		case MANUAL:
			// If previous mode was AUTOMATIC set speed and steer to zero
			if (from_odroid.arduino_state == AUTOMATIC)
			{
				set_steer(STEER_ZERO);
				set_speed(SPEED_ZERO);
			}
			else // Otherwise set steer and speed according to radio commands
			{
				set_steer(curr_steer);
				set_speed(curr_speed);
			}

			// Prepare the message to be sent
			sx_count = get_wheelcount_sx();
			dx_count = get_wheelcount_dx();
			to_odroid.steer_cmd          = curr_steer;
			to_odroid.speed_cmd          = curr_speed;
			to_odroid.wheel_dx_speed     = abs(dx_count);
			to_odroid.wheel_sx_speed     = abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			to_odroid.arduino_time       = (unsigned long) micros()-arduino_start_time;
			break;

		case AUTOMATIC:
			// Set steer and speed according to odroid commands
			set_steer(from_odroid.steer_cmd);
			set_speed(from_odroid.speed_cmd);

			// Prepare the message to be sent
			sx_count = get_wheelcount_sx();
			dx_count = get_wheelcount_dx();
			to_odroid.steer_cmd          = from_odroid.steer_cmd;
			to_odroid.speed_cmd          = from_odroid.speed_cmd;
			to_odroid.wheel_dx_speed     = abs(dx_count);
			to_odroid.wheel_sx_speed     = abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			to_odroid.arduino_time       = (unsigned long) micros()-arduino_start_time;
			break;

		case HALT:
			set_steer(STEER_ZERO);
			set_speed(SPEED_ZERO);

			// Prepare the message to be sent
			sx_count = get_wheelcount_sx();
			dx_count = get_wheelcount_dx();
			to_odroid.steer_cmd          = STEER_ZERO;
			to_odroid.speed_cmd          = SPEED_ZERO;
			to_odroid.wheel_dx_speed     = abs(dx_count);
			to_odroid.wheel_sx_speed     = abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			to_odroid.arduino_time       = (unsigned long) micros()-arduino_start_time;
			break;

		}

		// Encode the message
		encode_odroid_message(message_out, &to_odroid);

		// Send the message back
		send_udpMessage(message_out);
	}

	// In case of HALT state set speed and steer to zero and start blinking led "L"
	if (arduino_state.state == HALT)
	{
		set_steer(STEER_ZERO);
		set_speed(SPEED_ZERO);
	}

	// Reset connection watchdog
	wdt_reset();
}
