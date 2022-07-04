// Do not remove the include below
#include "drifting_car_ethernet.h"

#include "constant.h"
#include "type.h"
#include "udp_socket.h"
#include "telemetry.h"
#include "receiver.h"
#include "encoder.h"
#include "car_commands.h"

#include <stdio.h>
#include <avr/wdt.h>
#include <TimerOne.h>

#define DEBUG_1
#define TEST_LOOP_TIMING

void timer_callback();

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
size_t byte_received, byte_sent;

// Test variables
#ifdef TEST_LOOP_TIMING
bool output_on;
#endif
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
	set_encoderConfiguration(ENC_A_RISING, ENC_A_RISING);

	// Initialize steer and speed
	init_carcommandsIO();

	// Initialize telemetry messages
	initialize_odroid_message(&from_odroid);
	initialize_odroid_message(&to_odroid);

#ifdef DEBUG_1
	// Open serial communication for debugging
	Serial.begin(57600);
	while (!Serial) {
		; // wait for serial port to connect
	}
#endif

	// Start UDP connection
	init_udpConnection(server_mac, server_ip, server_port);

#ifdef DEBUG_1
	Serial.println("UDP connection started, waiting for client...");
#endif

	// Wait for connection of a client
	wait_client_connection();

#ifdef DEBUG_1
	Serial.println("Client connected, starting loop...");
#endif

	// Enable watchdog for connection lost
	wdt_enable(WDTO_120MS);

	// Initialize task timer
	Timer1.initialize(LOOP_PERIOD*1000000);
	Timer1.attachInterrupt(timer_callback);

#ifdef DEBUG_1
	Serial.println("Arduino state: SAFE");
#endif

#ifdef TEST_LOOP_TIMING
	pinMode(4, OUTPUT);
	digitalWrite(4, LOW);
	output_on = false;
#endif
}

// The loop function is called in an endless loop
void loop()
{
	// Do nothing
}

// Timed periodic loop
void timer_callback()
{
	if (arduino_state.state != HALT)
	{
		// Receive a message from Odroid
		byte_received = receive_udpMessage((uint8_t*) message_in, MESSAGE_SIZE);
		if (((int)byte_received) != MESSAGE_SIZE)
		{
			arduino_state.state = HALT;
			arduino_state.info = FAULT_COMM_BYTENUM;

#ifdef DEBUG_1
			Serial.print("Wrong number of byte received (");
			Serial.print((int)byte_received);
			Serial.print("), error ");
			Serial.println(arduino_state.info);
#endif
		}

		// Decode the message
		message_decode_error = decode_odroid_message(message_in, &from_odroid);
		if (message_decode_error > 0)
		{
			arduino_state.state = HALT;
			arduino_state.info = message_decode_error;

#ifdef DEBUG_1
			Serial.print("Error decoding incoming message (error ");
			Serial.print(arduino_state.info);
			Serial.println(")");
#endif
		}

#ifdef DEBUG_2
		Serial.print("receiving ("); Serial.print(message_decode_error); Serial.print(") ");
		Serial.print("steer: ");
		Serial.print(from_odroid.steer_cmd);
		Serial.print(" - speed: ");
		Serial.print(from_odroid.speed_cmd);
		Serial.print(" - wdx speed: ");
		Serial.print(from_odroid.wheel_dx_speed);
		Serial.print(" - wdx ccw: ");
		Serial.print(from_odroid.wheel_dx_ccw);
		Serial.print(" - wsx speed: ");
		Serial.print(from_odroid.wheel_sx_speed);
		Serial.print(" - wsx ccw: ");
		Serial.print(from_odroid.wheel_sx_ccw);
		Serial.print(" - state: ");
		Serial.print(from_odroid.arduino_state);
		Serial.print(" - state info: ");
		Serial.println(from_odroid.arduino_state_info);
#endif

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

#ifdef DEBUG_1
				Serial.println("Arduino state: MANUAL");
#endif
			}
			else if (arduino_state.state == MANUAL)
			{
				arduino_state.state = AUTOMATIC;
				arduino_state.info  = 0;

#ifdef DEBUG_1
				Serial.println("Arduino state: AUTOMATIC");
#endif
			}
			else if (arduino_state.state == AUTOMATIC)
			{
				arduino_state.state = MANUAL;
				arduino_state.info  = 0;

#ifdef DEBUG_1
				Serial.println("Arduino state: MANUAL");
#endif
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
			to_odroid.wheel_dx_speed     = (unsigned int) abs(dx_count);
			to_odroid.wheel_sx_speed     = (unsigned int) abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
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
			to_odroid.wheel_dx_speed     = (unsigned int) abs(dx_count);
			to_odroid.wheel_sx_speed     = (unsigned int) abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
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
			to_odroid.wheel_dx_speed     = (unsigned int) abs(dx_count);
			to_odroid.wheel_sx_speed     = (unsigned int) abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			break;

		case HALT:
			set_steer(STEER_ZERO);
			set_speed(SPEED_ZERO);

			// Prepare the message to be sent
			sx_count = get_wheelcount_sx();
			dx_count = get_wheelcount_dx();
			to_odroid.steer_cmd          = STEER_ZERO;
			to_odroid.speed_cmd          = SPEED_ZERO;
			to_odroid.wheel_dx_speed     = (unsigned int) abs(dx_count);
			to_odroid.wheel_sx_speed     = (unsigned int) abs(sx_count);
			to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
			to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
			to_odroid.arduino_state      = (unsigned char) arduino_state.state;
			to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
			break;

		}

		// Encode the message
		encode_odroid_message(message_out, &to_odroid);

#ifdef DEBUG_2
		telemetry_message tmp;
		message_decode_error = decode_odroid_message(message_out, &tmp);
		Serial.print("sending ("); Serial.print(message_decode_error); Serial.print(") ");
		Serial.print("steer: ");
		Serial.print(tmp.steer_cmd);
		Serial.print(" - speed: ");
		Serial.print(tmp.speed_cmd);
		Serial.print(" - wdx speed: ");
		Serial.print(tmp.wheel_dx_speed);
		Serial.print(" - wdx ccw: ");
		Serial.print(tmp.wheel_dx_ccw);
		Serial.print(" - wsx speed: ");
		Serial.print(tmp.wheel_sx_speed);
		Serial.print(" - wsx ccw: ");
		Serial.print(tmp.wheel_sx_ccw);
		Serial.print(" - state: ");
		Serial.print(tmp.arduino_state);
		Serial.print(" - state info: ");
		Serial.println(tmp.arduino_state_info);
#endif

		// Send the message back
		byte_sent = send_udpMessage((const uint8_t*) message_out, MESSAGE_SIZE);
		if (((int)byte_sent) != MESSAGE_SIZE)
		{
			arduino_state.state = HALT;
			arduino_state.info = FAULT_COMM_BYTENUM;

#ifdef DEBUG_1
			Serial.print("Wrong number of byte sent (");
			Serial.print((int)byte_sent);
			Serial.print("), error ");
			Serial.println(arduino_state.info);
#endif
		}
	}

	// In case of HALT state set speed and steer to zero and start blinking led "L"
	if (arduino_state.state == HALT)
	{
		set_steer(STEER_ZERO);
		set_speed(SPEED_ZERO);
	}

	// Reset connection watchdog
	wdt_reset();

	// Generate a square wave to check loop timing
#ifdef TEST_LOOP_TIMING
	if (output_on)
	{
		output_on = false;
		digitalWrite(4, LOW);
	}
	else
	{
		output_on = true;
		digitalWrite(4, HIGH);
	}
#endif
}
