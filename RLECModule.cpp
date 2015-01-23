/*
 * RLECModule.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: fwmav
 */

#include "RLECModule.h"

RLECModule::RLECModule() {
	for (int i=0; i<RLEC_CELLS; i++){
		cell_voltage[i] = 0;
		cell_temp[i] = 0;
	}
    max_cell_volt = 0;
    min_cell_volt = 0;
    rlec_temp = 0;
    balance_resistors = 0;
    faults = 0;
    rlec_volt = 0;
    min_cell_temp = 0;
    max_cell_temp = 0;
    sw_id = 0;
    charge_status = 0;
    status = Disabled;
}

