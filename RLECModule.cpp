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
 * @file RLECModule.cpp
 *
 * RLEC module
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
