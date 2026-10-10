
#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include "../common/messages.h"
#include "../drivers/median_filter.h"

typedef struct {
    median_filter_t filter;       
    sensor_status_t status;      
    int             bad_count;    
    int             ever_ok;      
    float           distance_cm; 
} sensor_health_t;


typedef enum {
    SENSOR_EVENT_NONE = 0,
    SENSOR_EVENT_READY,       
    SENSOR_EVENT_FAULT,      
    SENSOR_EVENT_RECOVERED   
} sensor_event_t;


void sensor_health_init(sensor_health_t *sh);


sensor_event_t sensor_health_update(sensor_health_t *sh, float raw_cm);

#endif
