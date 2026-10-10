
#include "../sensor/sensor_health.h"
#include "../common/config.h"

void sensor_health_init(sensor_health_t *sh)
{
    mf_init(&sh->filter);
    sh->status      = SENSOR_WAITING;
    sh->bad_count   = 0;
    sh->ever_ok     = 0;
    sh->distance_cm = MAX_VALID_CM;
}

sensor_event_t sensor_health_update(sensor_health_t *sh, float raw_cm)
{
    sensor_event_t event = SENSOR_EVENT_NONE;

    if (raw_cm >= MIN_VALID_CM && raw_cm <= MAX_VALID_CM) {
        sh->bad_count = 0;
        mf_add(&sh->filter, raw_cm);

        if (sh->status != SENSOR_OK && mf_ready(&sh->filter)) {
            event = sh->ever_ok ? SENSOR_EVENT_RECOVERED : SENSOR_EVENT_READY;
            sh->status  = SENSOR_OK;
            sh->ever_ok = 1;
        }
        if (sh->status == SENSOR_OK)
            sh->distance_cm = mf_median(&sh->filter);
    } else {
        if (sh->bad_count < SENSOR_FAULT_AFTER)
            sh->bad_count++;

        if (sh->status != SENSOR_FAULT && sh->bad_count >= SENSOR_FAULT_AFTER) {
            sh->status = SENSOR_FAULT;
            mf_init(&sh->filter);
            event = SENSOR_EVENT_FAULT;
        } else if (sh->status != SENSOR_OK) {
            mf_init(&sh->filter);      
        }
    }
    return event;
}
