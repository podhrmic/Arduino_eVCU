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
 * @file DeviceGEVCU.h
 *
 * Vehicle Control Unit
 */
#ifndef DEVICEGEVCU_H_
#define DEVICEGEVCU_H_

#include <Arduino.h>
#include "due_can.h"
#include "config.h"

enum BmsStatus {
	BMS_Uninit,
	BMS_Running,
	BMS_Fault
};

enum VSMstate {
	VSM_start,
	VSM_precharge_init,
	VSM_precharge_active,
	VSM_precharge_complete,
	VSM_wait,
	VSM_ready,
	VSM_running,
	VSM_fault,
	VSM_shutdown,
	VSM_recycle
};

enum InverterState {
	Inv_power_on,
	Inv_stop,
	Inv_open_loop,
	Inv_closed_loop,
	Inv_wait,
	Inv_internal_1,
	Inv_internal_2,
	Inv_internal_3,
	Inv_idle_run,
	Inv_idle_stop,
	Inv_internal_4,
	Inv_internal_5,
	Inv_internal_6
};

enum InvRunMode {
	InvRun_Torque_Mode,
	InvRun_Speed_Mode
};

enum InvCmdMode {
	InvCmd_CAN,
	InvCmd_VSM,
};

#define RMS_CAN_FREQUENCY 1000000

class DeviceGEVCU {
public:
	DeviceGEVCU();
	void process();
	void handleCanFrame(CAN_FRAME *frame);
	void console_periodic();
	uint16_t calculate_checksum(uint8_t* data, uint16_t data_len);


	void setMaxCellTemp(int8_t maxCellTemp) {
		max_cell_temp = maxCellTemp;
	}

	void setMaxCellVolt(uint16_t maxCellVolt) {
		max_cell_volt = maxCellVolt;
	}

	void setMinCellTemp(int8_t minCellTemp) {
		min_cell_temp = minCellTemp;
	}

	void setMinCellVolt(uint16_t minCellVolt) {
		min_cell_volt = minCellVolt;
	}

	enum VSMstate getVSMState(void) {
		return vsm_state;
	}
	// the can bus instance which this CanHandler instance is assigned to
	CANRaw *bus;

	// Time
	uint32_t timer; // u

	// Sys time (GEVCU)
	uint16_t sys_time; // u, overflows in 10hours

	// Temps
	int16_t phase_temp[3];//3x i
	int16_t gate_temp;// i
	int16_t board_temp;// i
	int16_t rtd_temp[5];// 5x i
	int16_t motor_temp;// i

	// Torque
	int16_t torque_shud;// i
	int16_t torque_cmd;// i
	int16_t torque_fb;// i

	// Analog inputs
	int16_t analog_in[4];//4x i

	// Digital inputs
	uint8_t digital_in[6];//6x u

	// Motor info
	int16_t motor_angle; //i
	int16_t motor_speed;// i
	int16_t inv_freq;// i
	int16_t resolver_angle;// i

	// Current
	int16_t phase_current[3]; // Phase A, B, C 3xi
	int16_t dc_current;// i

	// Voltage
	int16_t dc_voltage;// i
	int16_t output_volt;// i
	int16_t p_ab_volt;// i
	int16_t p_bc_volt;// i

	// Flux
	int16_t flux_cmd;//i
	int16_t flux_fb;// i
	int16_t id_fb;// i
	int16_t iq_fb;// i
	int16_t id_cmd;// i
	int16_t iq_cmd;// i

	// Internal Voltages
	int16_t ref_1_5;// i
	int16_t ref_2_5;// i
	int16_t ref_5_0;// i
	int16_t sys_12v;// i

	// Internal State
	enum VSMstate vsm_state;// u
	enum InverterState inv_state;// u
	uint8_t relay_state;// u
	enum InvRunMode inv_mode;// u
	enum InvCmdMode inv_cmd;// u
	uint8_t inv_enable;// u
	uint8_t motor_direction;// u
	uint8_t faults[8];// u

	// Various
	int16_t modulation_index;// i
	int16_t flux_reg_out;// i

	// Firmware data
	uint16_t eeprom_version;
	uint16_t sw_version;

	// Cell data
	int8_t max_cell_temp;
	int8_t min_cell_temp;
	uint16_t max_cell_volt;
	uint16_t min_cell_volt;

	enum VSMstate last_state;
private:
	CAN_FRAME tx_frame; // the request frame sent to the car
	CAN_FRAME rx_frame; // received frame
};

#endif /* DEVICEGEVCU_H_ */
