
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include "common/config.h"
#include "common/log.h"
#include "system/watchdog.h"
#include "display/display.h"
#include "motor/motor.h"
#include "system/telemetry.h"
#include "safety/safety_supervisor.h"
#include "sensor/sensor_monitor.h"
#include "navigation/navigation.h"
#include "system/telemetry.h"//new


int main(int argc, char *argv[])
{
    int test_hang = (argc > 1 && strcmp(argv[1], "--test-hang") == 0);
    sigset_t set;
    int sig;

    log_init();
    log_msg("=== QNX Safety Supervisor (mini) starting ===");

    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    if (watchdog_start()          != 0 ||
        display_start()           != 0 ||
        motor_controller_start()  != 0 ||
        safety_supervisor_start() != 0 ||
        sensor_monitor_start(test_hang) != 0 ||
        navigation_start()        != 0 ||
		telemetry_start() != 0) {
        fprintf(stderr, "failed to start a task / init hardware - exiting\n");
        motor_controller_close();
        sensor_monitor_close();
        return 1;
    }


    log_msg("all tasks running - press Ctrl+C to stop%s",
            test_hang ? " (fault test: sensor will hang at 15 s)" : "");

    sigwait(&set, &sig);
    log_msg("shutting down - stopping motors");
    motor_controller_close();
    sensor_monitor_close();
    return 0;
}
