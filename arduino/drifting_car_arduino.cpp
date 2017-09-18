// Do not remove the include below
#include "drifting_car_arduino.h"

#include "constant.h"
#include "type.h"
#include "telemetry.h"
#include "receiver.h"
#include "encoder.h"
#include "car_commands.h"
#include "task_timer.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup/Loop global variables                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial transmission
char message_in[MESSAGE_SIZE], message_out[MESSAGE_SIZE];
telemetry_message from_odroid, to_odroid;

// Car measures
unsigned int curr_speed, curr_steer;

// State machine
state_info arduino_state;

// Errors
unsigned int message_decode_error;
/////////////////////////////////////////////////////////////////////////////////////////////////////


// The setup function is called once at startup of the sketch
void setup()
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Serial communication                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Open serial communications and wait for port to open:
	Serial.begin(SERIAL_BAUDRATE);
	while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Board initialization                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
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

	// Initialize task timer
	init_taskTimer(LOOP_FREQUENCY);

	// Set led "L" to show when the system is in HALT state
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);

	// Activate a watchdog set to 15ms
	//init_encoderWDT();
	/////////////////////////////////////////////////////////////////////////////////////////////////////
}

// The loop function is called in an endless loop
void loop()
{
	if (canStart() & (arduino_state.state != HALT))
	{
		// If one complete message is available read it and write one message back
		if (Serial.available() >= MESSAGE_SIZE)
		{
			// Read a complete message
			for (int k=0; k<MESSAGE_SIZE; k++)
				message_in[k] = Serial.read();

			// Decode the message
			message_decode_error = decode_odroid_message(message_in, &from_odroid);
			if (message_decode_error > 0)
			{
//				arduino_state.state = HALT;
//				arduino_state.info = message_decode_error;
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
				to_odroid.steer_cmd          = STEER_ZERO;
				to_odroid.speed_cmd          = SPEED_ZERO;
				to_odroid.wheel_dx_speed     = get_wheelspeed_dx_us();
				to_odroid.wheel_sx_speed     = get_wheelspeed_sx_us();
				to_odroid.wheel_dx_ccw       = (is_wheel_dx_ccw() == 0x01) ? true : false;
				to_odroid.wheel_sx_ccw       = (is_wheel_sx_ccw() == 0x01) ? true : false;
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
				to_odroid.steer_cmd          = curr_steer;
				to_odroid.speed_cmd          = curr_speed;
				to_odroid.wheel_dx_speed     = get_wheelspeed_dx_us();
				to_odroid.wheel_sx_speed     = get_wheelspeed_sx_us();
				to_odroid.wheel_dx_ccw       = (is_wheel_dx_ccw() == 0x01) ? true : false;
				to_odroid.wheel_sx_ccw       = (is_wheel_sx_ccw() == 0x01) ? true : false;
				to_odroid.arduino_state      = (unsigned char) arduino_state.state;
				to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
				break;

			case AUTOMATIC:
				// Set steer and speed according to odroid commands
				set_steer(from_odroid.steer_cmd);
				set_speed(from_odroid.speed_cmd);

				// Prepare the message to be sent
				to_odroid.steer_cmd          = from_odroid.steer_cmd;
				to_odroid.speed_cmd          = from_odroid.speed_cmd;
				to_odroid.wheel_dx_speed     = get_wheelspeed_dx_us();
				to_odroid.wheel_sx_speed     = get_wheelspeed_sx_us();
				to_odroid.wheel_dx_ccw       = (is_wheel_dx_ccw() == 0x01) ? true : false;
				to_odroid.wheel_sx_ccw       = (is_wheel_sx_ccw() == 0x01) ? true : false;
				to_odroid.arduino_state      = (unsigned char) arduino_state.state;
				to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
				break;

			case HALT:
				set_steer(STEER_ZERO);
				set_speed(SPEED_ZERO);

				// Prepare the message to be sent
				to_odroid.steer_cmd          = STEER_ZERO;
				to_odroid.speed_cmd          = SPEED_ZERO;
				to_odroid.wheel_dx_speed     = get_wheelspeed_dx_us();
				to_odroid.wheel_sx_speed     = get_wheelspeed_sx_us();
				to_odroid.wheel_dx_ccw       = (is_wheel_dx_ccw() == 0x01) ? true : false;
				to_odroid.wheel_sx_ccw       = (is_wheel_sx_ccw() == 0x01) ? true : false;
				to_odroid.arduino_state      = (unsigned char) arduino_state.state;
				to_odroid.arduino_state_info = (unsigned char) arduino_state.info;
				break;
			}

			// Encode the message
			encode_odroid_message(message_out, &to_odroid);

			// Write the message back
			if (Serial.write(message_out, MESSAGE_SIZE) != MESSAGE_SIZE)
			{
				arduino_state.state = HALT;
				arduino_state.info = FAULT_COMM_BYTENUM;
			}
		}
	}

	// In case of HALT state set speed and steer to zero and start blinking led "L"
	if (arduino_state.state == HALT)
	{
		set_steer(STEER_ZERO);
		set_speed(SPEED_ZERO);

		// Switch off "L" led to show the system is in HALT state
		digitalWrite(13, LOW);
	}
}
