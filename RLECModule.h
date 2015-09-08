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
 * @file RLECModule.h
 *
 * RLEC module
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
