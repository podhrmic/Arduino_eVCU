/*
* Copyright (C) 2014 Viking Motorsport Team (VMS)
*
* This file is part of VMS Battery Management System.
*
* VMS Battery Management System is free software; you can redistribute 
* it and/or modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2,
* or (at your option) any later version.
*
* VMS Battery Management System is distributed in the hope that
* it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*/

/**
* @file main.hpp
*
* BMS main loop.
*/
#ifndef MAIN_H
#define MAIN_H

#include "DeviceGEVCU.h"
#include "DeviceBMS.h"
#include "ichip_2128.h"
#include "config.h"



inline void handle_periodic_tasks( void );

inline void failsafe_periodic( void );
inline void failsafe_shutdown( void );
inline void failsafe_warning( void );
//inline void datalog_periodic(void); - not using it now

inline void charger_shutdown( void );
inline void batlow_warning( void );
inline void batverylow_warning( void );
inline void batcritical_warning( void );

/*
 * Timers are:
 * 0. datalog
 * 1. heartbeat
 * 2. console (serial over usb)
 * 3. telemetry (wifi)
 * 4. failsafe
 * 5. BMS CAN
 * 6. RMS CAN
 * 7. sensor inputs
 * 8. system time
 */
void heartbeat_timer(void);
void console_timer(void);
void telemetry_timer(void);
void failsafe_timer(void);
void bms_can_timer(void);
void rms_can_timer(void);
void sensors_input_timer(void);
void systime_timer(void);
void datalog_timer(void);

/*
 * Device GEVCU
 *
 */
extern DeviceGEVCU* device;
extern DeviceBMS* bms;


#endif /* MAIN_H */

