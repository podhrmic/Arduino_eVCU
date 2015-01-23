/*
 * DeviceGEVCU.cpp
 *
 *  Created on: Nov 30, 2014
 *      Author: fwmav
 */

#include "DeviceGEVCU.h"
#include <stdio.h>

DeviceGEVCU::DeviceGEVCU() {
	sys_time = 0;
	bus = &CAN;//because we want CAN0 now
    timer = 0;
}

void DeviceGEVCU::console_periodic(){
	
	char buffer[1024];
        float f_timer = (float)timer*0.003;
        float f_sys_time = (float)sys_time/100;
	sprintf(buffer, "%f, %f,"
  			 // phase temp
  			 "%i, %i, %i,"
  			 // temps   rtd temp				motor temp
  			 "%i, %i,  	%i, %i, %i, %i, %i,		%i,"
  			 // torque
  			 "%i, %i, %i,"
  			 // analog in
  			 "%i, %i, %i, %i,"
  			// digitalin
  			 "%u, %u, %u, %u, %u, %u,"
  			 //motor info
  			 "%i, %i, %i, %i,"
  			 //current
  			 "%i, %i, %i, 	%i,"
  			 // Voltage
  			 "%i, %i, %i, %i,"
  			 // Flux
  			 "%i, %i, %i, %i, %i, %i,"
  			 // Internal Voltages
  			 "%i, %i, %i, %i,"
  			 //States
  			 "%u, %u, %u, %u, %u, %u, %u,"
  			 // Faults (8bytes)
  			 "%u, %u, %u, %u, %u, %u, %u, %u,"
  			 //Various
  			 "%i, %i,"
			// min cell temp, max cell temp, min cell volt
			 "%i, %i, %u, %u\r\n",
  			 f_timer, f_sys_time,
  			 phase_temp[0],phase_temp[1],phase_temp[2],
  			 gate_temp, board_temp, rtd_temp[0],rtd_temp[1],rtd_temp[2],rtd_temp[3],rtd_temp[4],motor_temp,
  			 torque_shud, torque_cmd,torque_fb,
  			 analog_in[0], analog_in[1], analog_in[2], analog_in[3],
  			 digital_in[0], digital_in[1], digital_in[2], digital_in[3], digital_in[4], digital_in[5],
  			 motor_angle, motor_speed, inv_freq, resolver_angle,
  			 phase_current[0], phase_current[1], phase_current[2], dc_current,
  			 dc_voltage, output_volt, p_ab_volt, p_bc_volt,
  			 flux_cmd, flux_fb, id_fb, iq_fb, id_cmd, iq_cmd,
  			 ref_1_5, ref_2_5, ref_5_0, sys_12v,
  			 vsm_state, inv_state, relay_state, inv_mode, inv_cmd, inv_enable, motor_direction,
  			 faults[0],faults[1],faults[2],faults[3],faults[4],faults[5],faults[6],faults[7],
  			 modulation_index, flux_reg_out,
			 min_cell_temp, max_cell_temp, min_cell_volt, max_cell_volt);
	SerialUSB.print(buffer);

	/*
	SerialUSB.print("Up time: " + String(((float)sys_time)/100) + "\r\n");
    SerialUSB.print("Timer: " + String(timer*0.003) + "\r\n");
    SerialUSB.print("max_cell_temp: " + String(max_cell_temp) + "\r\n");
    */

}

/*
 * Handle the response of the ECU and calculate the throttle value
 */
void DeviceGEVCU::handleCanFrame(CAN_FRAME *frame) {
        
	switch(frame->id){
	 case 0xA0: // Temperatures #1
		 phase_temp[0] = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 phase_temp[1] = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 phase_temp[2] = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 gate_temp = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA1: // Temperatures #2
		 board_temp = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 rtd_temp[0] = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 rtd_temp[1] = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 rtd_temp[2] = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA2: // Temperatures #3
		 rtd_temp[3] = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 rtd_temp[4] = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 motor_temp = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 torque_shud = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA3: // Analog Input Voltages
		 analog_in[0] = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 analog_in[1] = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 analog_in[2] = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 analog_in[3] = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA4: // Digital Input Status
		 digital_in[0] = frame->data.bytes[0];
		 digital_in[1] = frame->data.bytes[1];
		 digital_in[2] = frame->data.bytes[2];
		 digital_in[3] = frame->data.bytes[3];
		 digital_in[4] = frame->data.bytes[4];
		 digital_in[5] = frame->data.bytes[5];
		 break;
	 case 0xA5: // Motor Position Information
		 motor_angle = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 motor_speed = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 inv_freq = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 resolver_angle = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA6: // Current Information
		 phase_current[0] = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 phase_current[1] = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 phase_current[2] = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 dc_current = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA7: // Voltage Information
		 dc_voltage = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 output_volt = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 p_ab_volt = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 p_bc_volt = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA8: // Flux Information
		 flux_cmd = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 flux_fb = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 id_fb = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 iq_fb = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xA9: // Internal Voltages
		 ref_1_5 = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 ref_2_5 = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 ref_5_0 = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 sys_12v = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xAA: // Internal States
		 vsm_state = (VSMstate)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 inv_state = (InverterState)frame->data.bytes[2];
		 relay_state = frame->data.bytes[3];
		 inv_mode = (InvRunMode)frame->data.bytes[4];
		 inv_cmd = (InvCmdMode)frame->data.bytes[5];
		 inv_enable = frame->data.bytes[6];
		 motor_direction = frame->data.bytes[7];
		 break;
	 case 0xAB: // Fault Codes
		 faults[0] = frame->data.bytes[0];
		 faults[1] = frame->data.bytes[1];
		 faults[2] = frame->data.bytes[2];
		 faults[3] = frame->data.bytes[3];
		 faults[4] = frame->data.bytes[4];
		 faults[5] = frame->data.bytes[5];
		 faults[6] = frame->data.bytes[6];
		 faults[7] = frame->data.bytes[7];
		 break;
	 case 0xAC: // Torque & Timer Info
		 torque_cmd = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 torque_fb = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 timer = (uint32_t)(frame->data.bytes[7] << 24 | frame->data.bytes[6] << 16 | frame->data.bytes[5] << 8  | frame->data.bytes[4] );
		 break;
	 case 0xAD: // Modulation Index & Flux Weakening Output Information
		 modulation_index = (int16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 flux_reg_out = (int16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 id_cmd = (int16_t)(frame->data.bytes[5] << 8 | frame->data.bytes[4]);
		 iq_cmd = (int16_t)(frame->data.bytes[7] << 8 | frame->data.bytes[6]);
		 break;
	 case 0xAE: // Firmware Information
		 eeprom_version = (uint16_t)(frame->data.bytes[1] << 8 | frame->data.bytes[0]);
		 sw_version = (uint16_t)(frame->data.bytes[3] << 8 | frame->data.bytes[2]);
		 break;
	 case 0xAF: // Diagnostic Data
		 // Ignore
		 break;
	 default:
		 break;
	 }
}

/*
 * If a message is available, read it and forward it to registered observers.
 */
void DeviceGEVCU::process() {
	static CAN_FRAME frame;

	if (bus->rx_avail()) {
		bus->get_rx_buff(frame);
		handleCanFrame(&frame);
	}
}



