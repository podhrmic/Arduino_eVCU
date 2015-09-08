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
 * @file DeviceBMS.h
 *
 * Battery Management System
 */
#ifndef DEVICEBMS_H_
#define DEVICEBMS_H_

#include <Arduino.h>
#include "due_can.h"
#include "RLECModule.h"
#include "config.h"
#include "DeviceGEVCU.h"

#define RLEC_CAN_VOLTAGE_MULT 0.00244
#define RLEC_CAN_MODULE_MULT 0.0122
#define RLEC_CAN_FREQUENCY 500000
#define NUM_RLECS 16
#define STANDARD_CHARGE_MODE 0

enum MLECStatus {
	MLECUninit,
	MLECInit,
	MLECRunning
};


class DeviceBMS {
public:
	DeviceBMS();
	void process();
	void handleCanFrame(CAN_FRAME *frame);
	void can_init_rlecs();//
	void can_periodic();

	void mlec_charger_periodic(RLECModule* module);
	void mlec_init_broadcast();//
	void mlec_init_msgs();//
	void mlec_update_msg_ids(uint16_t offset);
	uint8_t mlec_broadcast();
	uint8_t mlec_send_request();

	void rlec_parse_msg(void);
	void rlec_parse_scan(int);

	void send_rlec_info(RLECModule* rlec);

	void update_battery_data();

	int8_t getMaxCellTemp() const {
		return max_cell_temp;
	}

	uint16_t getMaxCellVolt() const {
		return max_cell_volt;
	}

	int8_t getMinCellTemp() const {
		return min_cell_temp;
	}

	uint16_t getMinCellVolt() const {
		return min_cell_volt;
	}

	CANRaw *bus;	// the can bus instance which this CanHandler instance is assigned to

	// Rx message
	CAN_FRAME rx_msg;

	// RLEC request msg
	CAN_FRAME tMsg0;
	CAN_FRAME tMsg1;
	CAN_FRAME tMsg2;
	CAN_FRAME tMsg3;

	// Broadcast msgs
	CAN_FRAME bdc0;
	CAN_FRAME bdc1;
	CAN_FRAME bdc2;
	CAN_FRAME bdc3;
	CAN_FRAME bdc4;
	CAN_FRAME bdc5;

	// array with active RLECS
	uint8_t rlecs[NUM_RLECS];
	RLECModule rlecsX[NUM_RLECS];

	// Index of current RLECs probe
	uint8_t rlec_idx;

	// Tx Message offset
	uint8_t rlec_txoffset;

	// Status
	enum MLECStatus status;

	// Variables for RLEC scan
	uint8_t request_sent;

	int8_t max_cell_temp;
	int8_t min_cell_temp;
	uint16_t max_cell_volt;
	uint16_t min_cell_volt;

	// for charging diagnostics
	enum VSMstate vsm_state;// u
};

#endif /* DEVICEBMS_H_ */
