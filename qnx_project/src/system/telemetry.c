
#include "telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <stdint.h>
#include <curl/curl.h>

#include "../common/config.h"
#include "../system/rt_util.h"


#define FIREBASE_DATABASE_URL \
    "https://qnx-safety-supervisor-default-rtdb.firebaseio.com/telemetry/latest.json"

#define FIREBASE_AUTH_TOKEN "BZtqM7XLaUct84tQxgHo0uwPBVH3IXc6I65RGV5v"

/* Upload interval and network timeouts */
#define TELEMETRY_INTERVAL_MS       1000
#define TELEMETRY_CONNECT_TIMEOUT_MS 1500
#define TELEMETRY_TOTAL_TIMEOUT_MS   2500
#define TELEMETRY_JSON_SIZE          2048

/* ============================================================
 * SHARED SNAPSHOT
 * ============================================================ */

static pthread_mutex_t telemetry_lock = PTHREAD_MUTEX_INITIALIZER;
static telemetry_snapshot_t snapshot;


static const char *state_name(safety_state_t state)
{
    switch (state) {
    case STATE_SAFE:      return "SAFE";
    case STATE_WARNING:   return "WARNING";
    case STATE_EMERGENCY: return "EMERGENCY";
    case STATE_FAULT:     return "FAULT";
    default:              return "UNKNOWN";
    }
}

static const char *command_name(nav_cmd_t cmd)
{
    switch (cmd) {
    case CMD_STOP:       return "CMD_STOP";
    case CMD_FORWARD:    return "CMD_FORWARD";
    case CMD_TURN_RIGHT: return "CMD_TURN_RIGHT";
    default:             return "UNKNOWN";
    }
}

static const char *sensor_status_name(sensor_status_t status)
{
    if (status == SENSOR_OK)
        return "SENSOR_OK";

    if (status == SENSOR_WAITING)
        return "SENSOR_WAITING";

    return "SENSOR_FAULT";
}

int telemetry_update_sensors(const sensor_msg_t *msg)
{
    int i;

    if (msg == NULL)
        return -1;

    
    if (pthread_mutex_trylock(&telemetry_lock) != 0)
        return -1;

    for (i = 0; i < NUM_SENSORS; i++) {
        snapshot.distance_cm[i] = msg->distance_cm[i];
        snapshot.sensor_status[i] = msg->status[i];
    }

    snapshot.has_sensor_data = 1;

    pthread_mutex_unlock(&telemetry_lock);
    return 0;
}

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
    int deadline_ms)
{
    if (pthread_mutex_trylock(&telemetry_lock) != 0)
        return -1;

    snapshot.safety_state = state;
    snapshot.requested_cmd = requested_cmd;
    snapshot.requested_speed = requested_speed;
    snapshot.motor_cmd = motor_cmd;
    snapshot.approved_speed = approved_speed;

    snapshot.last_processing_us = processing_us;
    snapshot.decision_count = decision_count;
    snapshot.average_us = average_us;
    snapshot.worst_us = worst_us;
    snapshot.deadline_violations = violations;
    snapshot.deadline_ms = deadline_ms;

    snapshot.has_safety_data = 1;

    pthread_mutex_unlock(&telemetry_lock);
    return 0;
}

static int make_json(
    const telemetry_snapshot_t *s,
    char *json,
    size_t capacity)
{
    int used;
    int i;

    used = snprintf(
        json, capacity,
        "{"
        "\"has_sensor_data\":%s,"
        "\"has_safety_data\":%s,"
        "\"sensors\":[",
        s->has_sensor_data ? "true" : "false",
        s->has_safety_data ? "true" : "false"
    );

    if (used < 0 || (size_t)used >= capacity)
        return -1;

    for (i = 0; i < NUM_SENSORS; i++) {
        int n = snprintf(
            json + used, capacity - (size_t)used,
            "%s{"
            "\"distance_cm\":%.2f,"
            "\"status\":\"%s\""
            "}",
            i == 0 ? "" : ",",
            (double)s->distance_cm[i],
            sensor_status_name(s->sensor_status[i])
        );

        if (n < 0 || (size_t)n >= capacity - (size_t)used)
            return -1;

        used += n;
    }

    {
        int n = snprintf(
            json + used, capacity - (size_t)used,
            "],"
            "\"safety\":{"
                "\"state\":\"%s\","
                "\"navigation\":{"
                    "\"cmd\":\"%s\","
                    "\"requested_speed\":%d"
                "},"
                "\"motor\":{"
                    "\"cmd\":\"%s\","
                    "\"approved_speed\":%d"
                "},"
                "\"timing\":{"
                    "\"last_processing_us\":%llu,"
                    "\"decision_count\":%llu,"
                    "\"average_us\":%llu,"
                    "\"worst_us\":%llu,"
                    "\"deadline_violations\":%llu,"
                    "\"deadline_ms\":%d"
                "}"
            "}"
            "}",
            state_name(s->safety_state),
            command_name(s->requested_cmd),
            s->requested_speed,
            command_name(s->motor_cmd),
            s->approved_speed,
            (unsigned long long)s->last_processing_us,
            (unsigned long long)s->decision_count,
            (unsigned long long)s->average_us,
            (unsigned long long)s->worst_us,
            (unsigned long long)s->deadline_violations,
            s->deadline_ms
        );

        if (n < 0 || (size_t)n >= capacity - (size_t)used)
            return -1;

        used += n;
    }

    return used;
}

static int firebase_upload(CURL *curl, const char *json)
{
    CURLcode result;
    long http_status = 0;

    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);
    curl_easy_setopt(
        curl, CURLOPT_POSTFIELDSIZE, (long)strlen(json)
    );

    result = curl_easy_perform(curl);

    if (result != CURLE_OK) {
        fprintf(stderr, "[TELEMETRY] Upload failed: %s\n",
                curl_easy_strerror(result));
        return -1;
    }

    curl_easy_getinfo(
        curl, CURLINFO_RESPONSE_CODE, &http_status
    );

    if (http_status < 200 || http_status >= 300) {
        fprintf(stderr, "[TELEMETRY] Firebase HTTP status: %ld\n",
                http_status);
        return -1;
    }

    return 0;
}


static void *telemetry_thread(void *arg)
{
    telemetry_snapshot_t local;
    char json[TELEMETRY_JSON_SIZE];
    char firebase_url[2048];
    CURL *curl = NULL;
    struct curl_slist *headers = NULL;
    struct timespec delay;
    int n;

    (void)arg;

    if (strstr(FIREBASE_DATABASE_URL, "YOUR_DATABASE_HOST") != NULL ||
        strcmp(FIREBASE_AUTH_TOKEN,
               "REPLACE_WITH_YOUR_NEW_DATABASE_SECRET") == 0) {
        fprintf(stderr,
                "[TELEMETRY] Configure Firebase URL and secret\n");
        return NULL;
    }

    n = snprintf(
        firebase_url, sizeof(firebase_url),
        "%s?auth=%s",
        FIREBASE_DATABASE_URL,
        FIREBASE_AUTH_TOKEN
    );

    if (n < 0 || (size_t)n >= sizeof(firebase_url)) {
        fprintf(stderr, "[TELEMETRY] Firebase URL too long\n");
        return NULL;
    }

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        fprintf(stderr, "[TELEMETRY] libcurl initialization failed\n");
        return NULL;
    }

    curl = curl_easy_init();

    if (curl == NULL) {
        fprintf(stderr, "[TELEMETRY] curl initialization failed\n");
        curl_global_cleanup();
        return NULL;
    }

    headers = curl_slist_append(
        NULL, "Content-Type: application/json"
    );

    if (headers == NULL)
        goto cleanup;

    curl_easy_setopt(curl, CURLOPT_URL, firebase_url);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");

    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);

    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(
        curl, CURLOPT_CONNECTTIMEOUT_MS,
        (long)TELEMETRY_CONNECT_TIMEOUT_MS
    );
    curl_easy_setopt(
        curl, CURLOPT_TIMEOUT_MS,
        (long)TELEMETRY_TOTAL_TIMEOUT_MS
    );

    delay.tv_sec = TELEMETRY_INTERVAL_MS / 1000;
    delay.tv_nsec =
        (TELEMETRY_INTERVAL_MS % 1000) * 1000000L;

    for (;;) {
        if (pthread_mutex_lock(&telemetry_lock) == 0) {
            local = snapshot;
            pthread_mutex_unlock(&telemetry_lock);

            if (local.has_sensor_data || local.has_safety_data) {
                n = make_json(&local, json, sizeof(json));

                if (n < 0) {
                    fprintf(stderr,
                            "[TELEMETRY] JSON buffer too small\n");
                } else if (firebase_upload(curl, json) == 0) {
                    fprintf(stderr,
                            "[TELEMETRY] Firebase upload successful\n");
                }
            }
        }

        nanosleep(&delay, NULL);
    }

cleanup:
    curl_slist_free_all(headers);

    if (curl != NULL)
        curl_easy_cleanup(curl);

    curl_global_cleanup();
    return NULL;
}

int telemetry_start(void)
{
    return create_rt_thread(
        telemetry_thread,
        NULL,
        PRIO_SENSOR - 1,
        "telemetry"
    );
}
