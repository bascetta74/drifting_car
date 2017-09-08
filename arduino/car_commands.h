/*
 * car_commands.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Luca Bascetta
 */

#ifndef CAR_COMMANDS_H_
#define CAR_COMMANDS_H_

void init_carcommandsIO();

void set_steer(unsigned int steer_cmd);
void set_speed(unsigned int speed_cmd);

#endif /* CAR_COMMANDS_H_ */
