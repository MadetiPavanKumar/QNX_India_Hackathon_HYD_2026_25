/*
 * telemetry.h
 *
 *  Created on: Oct 9, 2026
 *      Author: pavan
 */


#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "../common/config.h"
#include "../common/messages.h"


typedef struct {
   
    float distance_cm[NUM_SENSORS];
    sensor_status_t sensor_status[NUM_SENSORS];
    int has_sensor_data;

  
    nav_cmd_t requested_cmd;
    int requested_speed;

   
    nav_cmd_t motor_cmd;
    int approved_speed;

    
    safety_state_t safety_state;

    uint64_t last_processing_us;
    uint64_t decision_count;
    uint64_t average_us;
    uint64_t worst_us;
    uint64_t deadline_violations;
    int deadline_ms;

    int has_safety_data;

} telemetry_snapshot_t;


int telemetry_update_sensors(const sensor_msg_t *msg);


int telemetry_update_safety(
    safety_state_t state,
    nav_cmd_t requested_cmd,
    int requested_speed,
    nav_cmd_t motor_cmd,
    int approved_speed,
    uint64_t processing_us,
    uint64_t decision_count,
    uint64_t average_us,
    uint64_t worst_us,
    uint64_t violations,
    int deadline_ms
);


int telemetry_start(void);

#endif
