/*
 * safety_supervisor.c
 *
 *  Created on: Oct 6, 2026
 *      Author: pavan
 */

#include <stdio.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "../safety/safety_supervisor.h"
#include "../motor/motor.h"
#include "../display/display.h"
#include "../system/telemetry.h"//new
#include "../system/watchdog.h"
#include "../system/rt_util.h"
#include "../common/config.h"
#include "../common/log.h"

#define STATS_EVERY   40     /* print timing stats every N decisions   */
#define STATUS_EVERY  20     /* print a status line every N decisions  */

static int safety_chid = -1;

/* ----- state owned by the Safety thread only ----- */
static float           dist_cm[NUM_SENSORS];  
static sensor_status_t status[NUM_SENSORS];   
static uint64_t        last_sensor_ms;        


static unsigned long n_decisions, n_violations;
static uint64_t sum_us, max_us;


static int      display_coid = -1;
static int      last_disp_state = -1;
static uint64_t last_disp_ms;

int safety_get_chid(void)
{
    return safety_chid;
}


static const char *state_name(safety_state_t s)
{
    switch (s) {
    case STATE_SAFE:      return "SAFE";
    case STATE_WARNING:   return "WARNING";
    case STATE_EMERGENCY: return "EMERGENCY";
    default:              return "FAULT";
    }
}

static void sensors_init_state(void)
{
    int i;
    for (i = 0; i < NUM_SENSORS; i++) {
        dist_cm[i] = MAX_VALID_CM;
        status[i]  = SENSOR_WAITING;
    }
}

static void handle_sensor_data(const sensor_msg_t *s)
{
    int i;
    for (i = 0; i < NUM_SENSORS; i++) {
        dist_cm[i] = s->distance_cm[i];
        status[i]  = s->status[i];
    }
    last_sensor_ms = now_ms();
}


static float closest_obstacle(void)
{
    float m = MAX_VALID_CM;
    int i;
    for (i = 0; i < NUM_SENSORS; i++)
        if (dist_cm[i] < m)
            m = dist_cm[i];
    return m;
}


static int sensors_trustworthy(const char **why)
{
    int i;

    for (i = 0; i < NUM_SENSORS; i++) {
        if (status[i] != SENSOR_OK) {
            *why = (status[i] == SENSOR_WAITING) ? "waiting for sensor data"
                                                 : "sensor fault";
            return 0;
        }
    }
    if (now_ms() - last_sensor_ms > SENSOR_STALE_MS) {
        *why = "sensor data stale";
        return 0;
    }
    return 1;
}


static safety_state_t evaluate(const nav_msg_t *req, int *approved,
                               float *closest, const char **why)
{
    float d;

    *why = "";
    *approved = 0;

    if (!sensors_trustworthy(why)) {
        *closest = -1.0f;
        return STATE_FAULT;
    }

    d = closest_obstacle();
    *closest = d;

    if (req->cmd == CMD_STOP)
        return (d <= EMERGENCY_CM) ? STATE_EMERGENCY :
               (d <= WARNING_CM)   ? STATE_WARNING   : STATE_SAFE;

    if (req->cmd == CMD_TURN_RIGHT) {
        *approved = (req->speed_percent > NAV_TURN_SPEED) ? NAV_TURN_SPEED
                                                         : req->speed_percent;
        return (d <= EMERGENCY_CM) ? STATE_EMERGENCY :
               (d <= WARNING_CM)   ? STATE_WARNING   : STATE_SAFE;
    }

    /* CMD_FORWARD */
    if (d <= EMERGENCY_CM)
        return STATE_EMERGENCY;

    if (d <= WARNING_CM) {
        float factor = (d - EMERGENCY_CM) / (WARNING_CM - EMERGENCY_CM);  /* 0..1 */
        int speed = (int)(req->speed_percent * factor);
        if (speed < MIN_MOVE_SPEED)
            speed = MIN_MOVE_SPEED;
        *approved = speed;
        return STATE_WARNING;
    }

    *approved = req->speed_percent;
    return STATE_SAFE;
}


static int send_to_motor(int motor_coid, nav_cmd_t cmd, int speed)
{
    motor_msg_t m;
    ack_reply_t r;

    m.type          = MSG_MOTOR_CMD;
    m.cmd           = (speed == 0) ? CMD_STOP : cmd;   /* speed 0 == stop */
    m.speed_percent = speed;
    return (msg_send_timeout(motor_coid, &m, sizeof(m), &r, sizeof(r),
                             SAFETY_DEADLINE_MS) == -1) ? -1 : 0;
}


static void update_display(safety_state_t state)
{
    uint64_t now = now_ms();
    float d[NUM_SENSORS];
    int i;

    if (display_coid == -1)
        return;
    if ((int)state == last_disp_state && now - last_disp_ms < DISPLAY_PERIOD_MS)
        return;

    for (i = 0; i < NUM_SENSORS; i++)
        d[i] = (status[i] == SENSOR_OK) ? dist_cm[i] : -1.0f;

    MsgSendPulse(display_coid, -1, PULSE_DISPLAY, display_pack(state, d[0], d[1]));
    last_disp_state = (int)state;
    last_disp_ms    = now;
}


static void record_timing(uint64_t elapsed_us)
{
    n_decisions++;
    sum_us += elapsed_us;
    if (elapsed_us > max_us)
        max_us = elapsed_us;

    if (n_decisions % STATS_EVERY == 0) {
        log_msg("[SAFETY] timing: %lu decisions, avg %llu us, worst %llu us, "
                "deadline %d ms, violations %lu",
                n_decisions, (unsigned long long)(sum_us / n_decisions),
                (unsigned long long)max_us, SAFETY_DEADLINE_MS, n_violations);
    }
}


static void handle_nav_request(int rcvid, const nav_msg_t *req, int motor_coid)
{
    static safety_state_t last_state = STATE_SAFE;
    static unsigned long  req_count;
    nav_reply_t reply;
    nav_cmd_t motor_cmd;
    const char *why;
    float closest;
    int approved;
    safety_state_t state;
    uint64_t t0 = now_us(), elapsed;

    state = evaluate(req, &approved, &closest, &why);

    if (send_to_motor(motor_coid, req->cmd, approved) != 0) {
        /* Motor Controller didn't answer in time - we can't confirm the
         * robot is doing what we think, so report a fault. */
        state    = STATE_FAULT;
        approved = 0;
        why      = "motor controller not responding";
    }

    elapsed = now_us() - t0;
    if (elapsed > (uint64_t)SAFETY_DEADLINE_MS * 1000ULL) {
        n_violations++;
        log_msg("[SAFETY] DEADLINE MISSED (%llu us) -> fail-safe stop", (unsigned long long)elapsed);
        send_to_motor(motor_coid, CMD_STOP, 0);
        state    = STATE_EMERGENCY;
        approved = 0;
    }
    record_timing(elapsed);

    
    req_count++;
    if (state != last_state || req_count % STATUS_EVERY == 0) {
        log_msg("[SAFETY] state=%s closest=%.1fcm requested=%d%% approved=%d%%%s%s",
                state_name(state), closest, req->speed_percent, approved,
                (state == STATE_FAULT) ? " reason: " : "",
                (state == STATE_FAULT) ? why : "");
        last_state = state;
    }

    update_display(state);

    reply.state          = state;
    reply.approved_speed = approved;

    motor_cmd = (approved == 0) ? CMD_STOP : req->cmd;

    MsgReply(rcvid, EOK, &reply, sizeof(reply));

    (void)telemetry_update_safety(
        state,
        req->cmd,
        req->speed_percent,
        motor_cmd,
        approved,
        elapsed,
        n_decisions,
        n_decisions ? sum_us / n_decisions : 0,
        max_us,
        n_violations,
        SAFETY_DEADLINE_MS
    );
}


static void *safety_thread(void *arg)
{
    safety_in_t in;
    ack_reply_t ack = { 1 };
    struct sigevent ev;
    uint64_t timeout_ns = (uint64_t)TICK_MS * 2 * 1000000ULL;
    int motor_coid, wd;
    (void)arg;

    sensors_init_state();

    motor_coid = ConnectAttach(ND_LOCAL_NODE, 0,motor_controller_get_chid(),_NTO_SIDE_CHANNEL, 0);
    wd = watchdog_connect();
    if (display_get_chid() != -1)
        display_coid = ConnectAttach(ND_LOCAL_NODE, 0, display_get_chid(),_NTO_SIDE_CHANNEL, 0);
    if (motor_coid == -1) {
        perror("safety: ConnectAttach(motor)");
        return NULL;
    }

    for (;;) {
        SIGEV_UNBLOCK_INIT(&ev);
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, &ev, &timeout_ns, NULL);

        int rcvid = MsgReceive(safety_chid, &in, sizeof(in), NULL);
        watchdog_beat(wd, TASK_SAFETY);

        if (rcvid == -1)        
            continue;
        if (rcvid == 0)      
            continue;

        switch (in.type) {
        case MSG_SENSOR_DATA:
            handle_sensor_data(&in.sensor);
            MsgReply(rcvid, EOK, &ack, sizeof(ack));
            break;
        case MSG_NAV_REQUEST:
            handle_nav_request(rcvid, &in.nav, motor_coid);
            break;
        default:
            MsgError(rcvid, ENOSYS);  
            break;
        }
    }
    return NULL;
}


int safety_supervisor_start(void)
{
    safety_chid = ChannelCreate(0);
    if (safety_chid == -1) {
        perror("safety: ChannelCreate");
        return -1;
    }
    return create_rt_thread(safety_thread, NULL, PRIO_SAFETY, "safety");
}
