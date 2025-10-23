/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_
#include <stdbool.h>

//------------ESC_PROTOCOLS----------
#define BIT_BANGING_V1
#define DSHOT_MODE 300	// 150 300 600 1200

#define DSHOT_BUFFER_LENGTH 18	// 16 bits of Dshot and 2 for clearing
#define DSHOT_PWM_FRAME_LENGTH 35
#define DSHOT_1_LENGTH 26
#define DSHOT_0_LENGTH 13

#if defined(BIT_BANGING_V1)
#define DSHOT_BB_BUFFER_LENGTH \
	18	// 16 bits of Dshot and 2 for clearing - used when bit-banging dshot
		// used
#define DSHOT_BB_FRAME_LENGTH \
	140	 // how many counts of timer gives one bit frame
#define DSHOT_BB_FRAME_SECTIONS \
	14	// in how many sections is bit frame divided (must be factor of
		// DSHOT_BB_FRAME_LENGTH)
#define DSHOT_BB_1_LENGTH 10
#define DSHOT_BB_0_LENGTH 4

#elif defined(BIT_BANGING_V2)
#define DSHOT_BB_BUFFER_LENGTH \
	18	// 16 bits of Dshot and 2 for clearing - used when bit-banging dshot
		// used
#define DSHOT_BB_FRAME_LENGTH 35
#define DSHOT_BB_1_LENGTH 26
#define DSHOT_BB_0_LENGTH 13
#define DSHOT_BB_FRAME_SECTIONS 3
#endif

#define BDSHOT_RESPONSE_LENGTH 21
#define BDSHOT_RESPONSE_BITRATE \
	(DSHOT_MODE * 4 / 3)  // in my tests this value was not 5/4 * DSHOT_MODE as
						  // documentation suggests
#define BDSHOT_RESPONSE_OVERSAMPLING \
	3  // it has to be a factor of (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE /
	   // BDSHOT_RESPONSE_BITRATE)

//-------------------MOTORS--------------------
#define MOTORS_COUNT 1	// how many motors are used
#define MOTOR_1 9		// PC9
#define MOTOR_POLES_NUMBER \
	14	// how many poles have your motors (usually 14 or 12)

#endif /*GLOBAL_CONSTANTS_H_*/