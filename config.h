/*
 * config.h
 *
 *  Created on: Dec 1, 2014
 *      Author: fwmav
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Some defines for the compilation
#define USE_WIFI 1
#define PRINT_DEBUG 1
#define PRINT_DATA 0
#define PRINT_STRING 1

// Voltage defines
#define MAX_CELL_VOLT (4.2/RLEC_CAN_VOLTAGE_MULT)
#define MIN_CELL_VOLT (2.5/RLEC_CAN_VOLTAGE_MULT)

#define MAX_CELL_TEMP 60

#define BAT_LOW (3/RLEC_CAN_VOLTAGE_MULT)
#define BAT_VERY_LOW (2.7/RLEC_CAN_VOLTAGE_MULT)
#define BAT_CRITICAL MIN_CELL_VOLT

#define BAT_BALANCE_CUTOFF (4.13/RLEC_CAN_VOLTAGE_MULT)
#define BAT_CHARGE_CUTOFF (4.15/RLEC_CAN_VOLTAGE_MULT)
#define BAT_DELTA_BALANCE 8

// FAULT DEFINES
#define RLEC_CELL_1_AD_FAULT 0x1 // critical
#define RLEC_CELL_TEMP_AD_FAULT 0x2 // warning
#define RLEC_RLEC_TEMP_AD_FAULT 0x4 // warning
#define RLEC_CELL_VOLTAGE_CONNECTION_FAULT 0x8 //critical
#define RLEC_CELL_VOLTAGE_AD_FAULT 0x10 // critical
#define RLEC_MODULE_VOLTAGE_AD_FAULT 0x20 // critical
#define RLEC_ZERO_CAPACITOR_VOLTAGE_FAULT 0x40 // nothing
#define RLEC_CELL_1_VOLTAGE_FAULT 0x80 // critical

// Startup time for failsafe
#define T_MIN 1000 // s*10

/*
 * Digital outputs
 */
// #define THROTTLE_OUT 4// DOUT0 - we are not using throttle output at the moment
#define RTDS_PIN 5 // DOUT1 
#define AMS_LED 6 // DOUT2
#define FW_ENABLE 7 // DOUT3
#define SHUTDOWN 2 // DOUT4
// #define DC_ENABLE 8 // DOUT6, PC22 - we are not using DC enable (always on)
#define IMD_LED 9 // DOUT7, PC21

#define THROTTLE_IN_1 57 // PA22, Analog 3, AIN0
#define THROTTLE_IN_2 56 // PA23, Analog 2, AIN1
#define BATT_SENSE 55 // PA24, Analog 1, AIN2

#define BRAKE_EN 49 // PC14, Digital Pin 49, DIN1
#define IMD_STATUS 50 // PC13, DIN2

// Legacy defines
#define HLIM_PIN SHUTDOWN

/*
 * Frequency defines [Hz]
 */
#ifndef HEARTBEAT_FREQUENCY
#define HEARTBEAT_FREQUENCY 1.0
#endif

#ifndef CONSOLE_FREQUENCY
#define CONSOLE_FREQUENCY 10.0
#endif

#ifndef TELEMETRY_FREQUENCY
#define TELEMETRY_FREQUENCY 1.0
#endif

#ifndef FAILSAFE_FREQUENCY
#define FAILSAFE_FREQUENCY 10.0
#endif

#ifndef BMS_CAN_FREQUENCY
#define BMS_CAN_FREQUENCY 10.0
#endif

#ifndef RMS_CAN_FREQUENCY
#define RMS_CAN_FREQUENCY 10.0
#endif

#ifndef DOWNLINK_FREQUENCY
#define DOWNLINK_FREQUENCY 1.0
#endif

#ifndef SENSOR_INPUT_FREQUENCY
#define SENSOR_INPUT_FREQUENCY 5.0
#endif

#ifndef SYSTIME_FREQUENCY
#define SYSTIME_FREQUENCY 100.0
#endif

#ifndef DATALOG_FREQUENCY
#define DATALOG_FREQUENCY 10.0
#endif

#endif /* CONFIG_H_ */

