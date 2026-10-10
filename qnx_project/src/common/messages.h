/*
 * messages.h
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */

#ifndef COMMON_MESSAGES_H_
#define COMMON_MESSAGES_H_

#include <stdint.h>
#include<sys/neutrino.h>
#include"../common/config.h"

typedef enum{
	CMD_STOP=0,
	CMD_FORWARD,
	CMD_TURN_RIGHT
}nav_cmd_t;

typedef enum{
	STATE_SAFE=0,
	STATE_EMERGENCY,
	STATE_WARNING,
	STATE_FAULT
}safety_state_t;

typedef enum{
	SENSOR_WAITING=0,
	SENSOR_OK,
	SENSOR_FAULT,
	SENSOR_NONE
}sensor_status_t;

typedef enum{
	TASK_SENSOR=0,
	TASK_NAVIGATION,
	TASK_MOTOR,
	TASK_SAFETY,
	NUM_TASKS
}task_id_t;
//pulse codes for triggering task and heartbeat

#define PULSE_TICK (_PULSE_CODE_MINAVAIL +0)//for triggering tassks
#define PULSE_HEARTBEAT (_PULSE_CODE_MINAVAIL +1)
#define PULSE_WD_CHECK (_PULSE_CODE_MINAVAIL +2)
#define PULSE_DISPLAY (_PULSE_CODE_MINAVAIL +3)
//msg types
#define MSG_NAV_REQUEST 1  //uint16_t
#define MSG_SENSOR_DATA 2
#define MSG_MOTOR_CMD 3

//structres for task-to-task communication

//navigation task to safety task
typedef struct{
	uint16_t type;
	nav_cmd_t cmd;
	int speed_percent;
}nav_msg_t;
//sensor_to_safety
typedef struct{
	uint16_t type;
	float distance_cm[NUM_SENSORS];
	sensor_status_t status[NUM_SENSORS];
}sensor_msg_t;

//after safety decision safety needs to send a msg to motor for movement

typedef struct{
	uint16_t type;
	nav_cmd_t cmd;
	int speed_percent;
}motor_msg_t;

//after communicating ev ery task should  acknowledge that cmd excucted successfully
//repy struct's

typedef struct{
	safety_state_t state;
	int approved_speed;
}nav_reply_t;

typedef struct{
	int ok;
}ack_reply_t;

///safety should receive all types of msg like normal ipc msg and pulse so we need to take a union structure for handle all types of data
typedef union{
	uint16_t type;
	nav_msg_t nav;
	sensor_msg_t sensor;
	struct _pulse pulse;
}safety_in_t;


#endif /* COMMON_MESSAGES_H_ */
