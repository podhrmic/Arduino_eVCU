/*
 * Copyright (C) 2015 Michal Podhradsky
 * michal.podhradsky@pdx.edu
 *
 * This file is part of Viking Motorsports Arduino_eVCU.
 *
 * Viking Motorsports Arduino_eVCU is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Viking Motorsports Arduino_eVCU is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Viking Motorsports Arduino_eVCU; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
/**
 * @file config.h
 *
 * ECU Configuration file
 */
#ifndef CONFIG_H_
#define CONFIG_H_

// Some defines for the compilation
#define PRINT_DEBUG 1//0
#define PRINT_DATA 0//1
#define PRINT_STRING 0//1

// Voltage defines
#define MAX_CELL_VOLT (4.3/RLEC_CAN_VOLTAGE_MULT)
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
// >> RTDS- ->  DOUT0
#define RTDS_PIN 4 // DOUT0

// >> FW_ENABLE ->  DOUT1
#define FW_ENABLE 5 // DOUT1

// >> REGEN_DISABLE  ->  DOUT2
#define REGEN_DISABLE 6 // DOUT2

// >> NC -> DOUT3
#define PIN_DOUT3 7// DOUT3

// >> RMS_THROTTLE  ->  DOUT4
#define RMS_THROTTLE 2 // DOUT4

// >>  AMS_STATUS ->  DOUT5
#define AMS_STATUS 3 // DOUT5

// >> PWR_READY -> DOUT6
#define PWR_READY 8 // DOUT6

// >> PWR_STDBY  -> DOUT7
#define PWR_STDBY 9 // DOUT7

// Throttle input
#define THROTTLE_IN_1 57 // PA22, Analog 3, AIN0
#define THROTTLE_IN_2 56 // PA23, Analog 2, AIN1

// >> CAN_BMS -> CAN0
// >> CAN_RMS -> CAN1

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
