/*
 * constant.h
 *
 *  Created on: Jul 5, 2017
 *      Author: Luca Bascetta
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_

// FIR filter
#define MAX_FIR_ORDER 			8		// Maximum order of FIR filters

// Realtime loop
#define LOOP_FREQUENCY 			100		// Main loop frequency Hz

// Serial communication
#define SERIAL_BAUDRATE 		115200  // Serial port baud rate
#define MESSAGE_SIZE			14		// Size in byte of the serial message

// Fault codes
#define FAULT_COMM_BYTENUM		100		// Wrong number of bytes wrote
#define FAULT_COMM_CHECKSUM		101		// Checksum error in serial message
#define FAULT_COMM_HEADER		102		// Wrong header in serial message

// Car commands
#define STEER_MIN				1240	// Minimum value for steer command (pulse width ms)
#define STEER_ZERO				1500	// Neutral value for steer command (pulse width ms)
#define STEER_MAX				1740	// Maximum value for steer command (pulse width ms)
#define SPEED_MIN				1500	// Minimum value for steer command (pulse width ms)
#define SPEED_ZERO				1500	// Neutral value for steer command (pulse width ms)
#define SPEED_MAX				1600	// Maximum value for steer command (pulse width ms)
#define BUTTON_ZERO				1500	// Neutral value for button (pulse width ms)

// Arduino PINS
#define STEER_SERVO_PIN			2		// Steer servo pin
#define SPEED_SERVO_PIN			3		// Speed servo pin
#define ENC_FRWHEEL_A_PIN		8		// Encoder front right wheel channel A pin
#define ENC_FRWHEEL_B_PIN		9		// Encoder front right wheel channel B pin
#define ENC_FLWHEEL_A_PIN		10		// Encoder front left wheel channel A pin
#define ENC_FLWHEEL_B_PIN		11		// Encoder front left wheel channel B pin
#define CH1_RECEIVER_PIN		14		// Channel 1 (steer) receiver pin
#define CH2_RECEIVER_PIN		15		// Channel 2 (speed) receiver pin
#define CH3_RECEIVER_PIN		16		// Channel 3 (button) receiver pin

#endif /* CONSTANT_H_ */
