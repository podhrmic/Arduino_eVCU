/*
 * RLECModule.h
 *
 *  Created on: Dec 1, 2014
 *      Author: fwmav
 */

#ifndef RLECMODULE_H_
#define RLECMODULE_H_

#include <Arduino.h>

#define RLEC_CELLS 12

enum RLECStatus {
  Disabled,
  Active
};

class RLECModule {
public:
	RLECModule();
    uint16_t cell_voltage[RLEC_CELLS];
    uint16_t max_cell_volt;
    uint16_t min_cell_volt;
    int8_t rlec_temp;
    uint16_t balance_resistors; // balance regs byte for now
    uint8_t faults; // faults byte
    uint16_t rlec_volt;
    int8_t cell_temp[RLEC_CELLS];
    int8_t min_cell_temp;
    int8_t max_cell_temp;
    uint8_t sw_id;
    uint8_t charge_status;
    enum RLECStatus status;
};

#endif /* RLECMODULE_H_ */
