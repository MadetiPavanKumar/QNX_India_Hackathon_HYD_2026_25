
#ifndef CONFIG_H
#define CONFIG_H


#define TICK_MS                 75
#define SENSOR_PERIOD_MS        120
//distances
#define WARNING_CM              32.0f
#define EMERGENCY_CM            18.0f

//dist ranges
#define MIN_VALID_CM            2.0f
#define MAX_VALID_CM            400.0f

//minmum speed required to move robot                          */
#define MIN_MOVE_SPEED          50

//filter configuration
#define MEDIAN_WINDOW           5
#define MEDIAN_MIN_SAMPLES      3
#define SENSOR_FAULT_AFTER      10

#define SENSOR_STALE_MS         400

#define SAFETY_DEADLINE_MS      TICK_MS

#define MOTOR_TIMEOUT_MS        (TICK_MS * 3)

#define WATCHDOG_CHECK_MS           100
#define HEARTBEAT_TIMEOUT_MS        400  
#define SENSOR_HEARTBEAT_TIMEOUT_MS 500   

#define NAV_CRUISE_SPEED        70      
#define NAV_TURN_SPEED          60      
#define NAV_BLOCKED_TICKS       50      
#define NAV_TURN_TICKS          25       


#define PRIO_SAFETY             30
#define PRIO_MOTOR              25
#define PRIO_NAVIGATION         20
#define PRIO_WATCHDOG           15
#define PRIO_SENSOR             10
#define PRIO_DISPLAY            5      


#define OLED_I2C_DEV         "/dev/i2c1"
#define DISPLAY_PERIOD_MS       200

#define NUM_SENSORS             2

#define PIN_US0_TRIG            5
#define PIN_US0_ECHO            6
#define PIN_US1_TRIG            16
#define PIN_US1_ECHO            19

#define PIN_MOTOR_EN            18      
#define PIN_MOTOR_RIGHT_FWD     10
#define PIN_MOTOR_RIGHT_REV     9
#define PIN_MOTOR_LEFT_FWD      8
#define PIN_MOTOR_LEFT_REV      11
#define MOTOR_PWM_HZ            2000

#endif /* CONFIG_H */
