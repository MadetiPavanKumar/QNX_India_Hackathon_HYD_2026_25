
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../sensor/sensor_monitor.h"
#include "../sensor/sensor_health.h"
#include "../safety/safety_supervisor.h"
#include "../system/telemetry.h"
#include "../system/watchdog.h"
#include "../system/ticker.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"
#include "../drivers/hc_sr04.h"

#define HANG_AFTER_MS     15000   
#define HANG_DURATION_S   3      

static int g_test_hang;
static int sensor_handle[NUM_SENSORS] = { -1, -1 };

static void log_event(int sensor, sensor_event_t ev)
{
    switch (ev) {
    case SENSOR_EVENT_READY:     log_msg("[SENSOR] sensor %d ready", sensor); break;
    case SENSOR_EVENT_RECOVERED: log_msg("[SENSOR] sensor %d recovered", sensor); break;
    case SENSOR_EVENT_FAULT:
        log_msg("[SENSOR] sensor %d FAULT (%d bad readings in a row)", sensor, SENSOR_FAULT_AFTER);
        break;
    default: break;
    }
}

static void *sensor_thread(void *arg)
{
    struct _pulse pulse;
    sensor_msg_t msg;
    ack_reply_t ack;
    sensor_health_t health[NUM_SENSORS];
    int tick_chid, safety_coid, wd, i;
    int hang_done = 0;
    uint64_t start = now_ms();
    (void)arg;

    for (i = 0; i < NUM_SENSORS; i++)
        sensor_health_init(&health[i]);

    
    tick_chid = ChannelCreate(0);
    if (tick_chid == -1 || timer_pulse_start(tick_chid, PULSE_TICK, SENSOR_PERIOD_MS) != 0)
        return NULL;

    safety_coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_get_chid(), _NTO_SIDE_CHANNEL, 0);
    wd = watchdog_connect();
    if (safety_coid == -1) {
        perror("sensor: ConnectAttach(safety)");
        return NULL;
    }

    for (;;) {
        MsgReceive(tick_chid, &pulse, sizeof(pulse), NULL);   /* wait for tick */

        
        if (g_test_hang && !hang_done && (now_ms() - start) > HANG_AFTER_MS) {
            hang_done = 1;
            log_msg("[SENSOR] (test) simulating a hang for %d s", HANG_DURATION_S);
            sleep(HANG_DURATION_S);
        }

        msg.type = MSG_SENSOR_DATA;
        for (i = 0; i < NUM_SENSORS; i++) {
            float raw = read_distance(sensor_handle[i]);
            log_event(i, sensor_health_update(&health[i], raw));
            msg.distance_cm[i] = health[i].distance_cm;
            msg.status[i]      = health[i].status;
        }
        
        msg_send_timeout(safety_coid, &msg, sizeof(msg), &ack, sizeof(ack), TICK_MS * 2);
        (void)telemetry_update_sensors(&msg);

        watchdog_beat(wd, TASK_SENSOR);
    }
    return NULL;
}


int sensor_monitor_start(int test_hang)
{
    g_test_hang = test_hang;

    sensor_handle[0] = hc_sr04_begin(PIN_US0_TRIG, PIN_US0_ECHO);
    sensor_handle[1] = hc_sr04_begin(PIN_US1_TRIG, PIN_US1_ECHO);
    if (sensor_handle[0] < 0 || sensor_handle[1] < 0) {
        fprintf(stderr, "sensor_monitor: hc_sr04_begin failed (h0=%d h1=%d)\n",
                sensor_handle[0], sensor_handle[1]);
        return -1;
    }
    return create_rt_thread(sensor_thread, NULL, PRIO_SENSOR, "sensor_mon");
}


void sensor_monitor_close(void)
{
    int i;
    for (i = 0; i < NUM_SENSORS; i++)
        if (sensor_handle[i] >= 0)
            hc_sr04_end(sensor_handle[i]);
}
