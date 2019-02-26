// Do not remove the include below
#include "drifting_car_serial.h"

#include "constant.h"
#include "type.h"
#include "telemetryProtocol.h"
#include "receiver.h"
#include "encoder.h"
#include "car_commands.h"

#include <TimerOne.h>

//#define DEBUG
//#define TEST_LOOP_TIMING

void timer_callback();

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup/Loop global variables                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial transmission
telemetryProtocol telemetry;
telemetry_message from_odroid, to_odroid;

// Car measures
byte message_counter;
unsigned int curr_speed, curr_steer;

// Encoder measures
int sx_count, dx_count;

// State machine
state_info arduino_state;

// Test variables
#ifdef TEST_LOOP_TIMING
bool output_on;
#endif
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
	Serial.flush();
	Serial.setTimeout(1);
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Board initialization                                                                            //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialize global variables
	curr_speed = curr_steer = 0;
	message_counter = 0;

	// Initialize state machine
	arduino_state.state = SAFE;
	arduino_state.info = 0;

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
	telemetry.init_message(&from_odroid);
	telemetry.init_message(&to_odroid);

	// Initialize task timer
	Timer1.initialize(LOOP_PERIOD*1000000);
	Timer1.attachInterrupt(timer_callback);

	// Set led "L" to show when the system is in HALT state
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Starting communication                                                                          //
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Sending an empty message to start the communication
	if (!telemetry.send(&to_odroid))
	{
		arduino_state.state = HALT;
		arduino_state.info = FAULT_COMM_BYTENUM;
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG
	Serial.println("Board initialized, loop starting...");
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
		// Try to receive a message
		telemetry.receive();

		// If one complete message is available read it and write one message back
		if (telemetry.message_available())
		{
			// Read a complete message
			telemetry.read_message(&from_odroid);

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
				to_odroid.arduino_state      = (byte) arduino_state.state;
				to_odroid.arduino_state_info = (byte) arduino_state.info;
				to_odroid.message_number     = (byte) message_counter;
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
				to_odroid.arduino_state      = (byte)arduino_state.state;
				to_odroid.arduino_state_info = (byte) arduino_state.info;
				to_odroid.message_number     = (byte) message_counter;
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
				to_odroid.arduino_state      = (byte) arduino_state.state;
				to_odroid.arduino_state_info = (byte) arduino_state.info;
				to_odroid.message_number     = (byte) message_counter;
				break;

			case HALT:
				set_steer(STEER_ZERO);
				set_speed(SPEED_ZERO);

				// Prepare the message to be sent
				sx_count = get_wheelcount_sx();
				dx_count = get_wheelcount_dx();
				to_odroid.steer_cmd          = STEER_ZERO;
				to_odroid.speed_cmd          = SPEED_ZERO;
				to_odroid.wheel_dx_speed     = abs(dx_count);;
				to_odroid.wheel_sx_speed     = abs(sx_count);;
				to_odroid.wheel_dx_ccw       = (dx_count < 0) ? true : false;
				to_odroid.wheel_sx_ccw       = (sx_count < 0) ? true : false;
				to_odroid.arduino_state      = (byte) arduino_state.state;
				to_odroid.arduino_state_info = (byte) arduino_state.info;
				to_odroid.message_number     = (byte) message_counter;
				break;
			}

			// Update message counter
			if (message_counter<250)
				message_counter++;
			else
				message_counter = 0;

			if (message_counter == 125) {
				if (arduino_state.state == SAFE)
					arduino_state.state = MANUAL;
				else {
					if (arduino_state.state == MANUAL)
						arduino_state.state = AUTOMATIC;
					else {
						if (arduino_state.state == AUTOMATIC)
							arduino_state.state = SAFE;
					}
				}
			}
#ifdef TEST_LOOP_TIMING
			if (output_on)
			{
				output_on = false;
				digitalWrite(4,LOW);
			}
			else
			{
				output_on = true;
				digitalWrite(4, HIGH);
			}
#endif
		}

		// Write the message back
		if (!telemetry.send(&to_odroid))
		{
			arduino_state.state = HALT;
			arduino_state.info = FAULT_COMM_BYTENUM;
		}

#ifdef DEBUG
		if (message_counter<254)
			message_counter++;
		else
			message_counter = 0;

		Serial.print("message number: ");
		Serial.println(message_counter);
#endif
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
