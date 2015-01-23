/*
 * DeviceBMS.h
 *
 *  Created on: Dec 1, 2014
 *      Author: fwmav
 */

#ifndef DEVICEBMS_H_
#define DEVICEBMS_H_

#include <Arduino.h>
#include "due_can.h"
#include "RLECModule.h"
#include "config.h"

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
	//void can_event_rlecs(); - is now process()
	void mlec_charger_periodic(RLECModule* module, CAN_FRAME* msg6);
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
};

#endif /* DEVICEBMS_H_ */
