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
	phase_temp[0] = 0;//3x i
	phase_temp[1] = 0;
	phase_temp[2] = 0;
	gate_temp = 0;// i
	board_temp = 0;// i
	rtd_temp[0] = 0;// 5x i
	rtd_temp[1] = 0;// 5x i
	rtd_temp[2] = 0;// 5x i
	rtd_temp[3] = 0;// 5x i
	rtd_temp[4] = 0;// 5x i
	motor_temp = 0;// i

	// Torque
	torque_shud = 0;// i
	torque_cmd = 0;// i
	torque_fb = 0;// i

	// Analog inputs
	analog_in[0] = 0;//4x i
	analog_in[1] = 0;//4x i
	analog_in[2] = 0;//4x i
	analog_in[3] = 0;//4x i

	// Digital inputs
	digital_in[0] = 0;//6x u
	digital_in[1] = 0;//6x u
	digital_in[2] = 0;//6x u
	digital_in[3] = 0;//6x u
	digital_in[4] = 0;//6x u
	digital_in[5] = 0;//6x u

	// Motor info
	motor_angle = 0; //i
	motor_speed = 0;// i
	inv_freq = 0;// i
	resolver_angle = 0;// i

	// Current
	phase_current[0] = 0; // Phase A, B, C 3xi
	phase_current[1] = 0; // Phase A, B, C 3xi
	phase_current[2] = 0; // Phase A, B, C 3xi
	dc_current = 0;// i

	// Voltage
	dc_voltage = 0;// i
	output_volt = 0;// i
	p_ab_volt = 0;// i
	p_bc_volt = 0;// i

	// Flux
	flux_cmd = 0;//i
	flux_fb = 0;// i
	id_fb = 0;// i
	iq_fb = 0;// i
	id_cmd = 0;// i
	iq_cmd = 0;// i

	// Internal Voltages
	ref_1_5 = 0;// i
	ref_2_5 = 0;// i
	ref_5_0 = 0;// i
	sys_12v = 0;// i

	// Internal State
	enum VSMstate vsm_state = VSM_start;// u
	enum VSMstate last_state = vsm_state;
	enum InverterState inv_state = Inv_power_on;// u
	relay_state = 0;// u
	enum InvRunMode inv_mode = InvRun_Torque_Mode;// u
	enum InvCmdMode inv_cmd = InvCmd_CAN;// u
	inv_enable = 0;// u
	motor_direction = 0;// u
	faults[0] = 0;// u
	faults[1] = 0;// u
	faults[2] = 0;// u
	faults[3] = 0;// u
	faults[4] = 0;// u
	faults[5] = 0;// u
	faults[6] = 0;// u
	faults[7] = 0;// u

	// Various
	modulation_index = 0;// i
	flux_reg_out = 0;// i

	// Firmware data
	eeprom_version = 0;
	sw_version = 0;

	// Cell data
	max_cell_temp = 0;
	min_cell_temp = 0;
	max_cell_volt = 0;
	min_cell_volt = 0;
}

/**
 * Calculate checksum per Xgear documentation
 * @param data
 * @param data_len
 * @return
 */
uint16_t DeviceGEVCU::calculate_checksum(uint8_t* data, uint16_t data_len)
{
  uint8_t byte1 = 0;
  uint8_t byte2 = 0;
  for(int x=0; x<data_len; ++x)
  {
    byte1 += data[x];
    byte2 += byte1;
  }
  return (byte1<<8)+byte2;
}


void DeviceGEVCU::console_periodic(){
    float f_timer = (float)timer*0.003;
    float f_sys_time = (float)sys_time/100;

#ifdef PRINT_STRING
	char buffer[1024];
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
#else /* PRINT_BINARY */
	uint8_t buf[1024];

	// header
	buf[0] = 0xBE;
	buf[1] = 0xEF;

	uint16_t idx = 2;

	uint16_t payload_lenght = 119;
	uint16_t packet_length = 125;

	// datalenght
	memcpy(&buf[idx], &payload_lenght, sizeof(uint16_t));
	idx += sizeof(uint16_t);

	// payload

	//f_timer	4	float	4	sec	RMS time
	memcpy(&buf[idx], &f_timer, sizeof(float));
	idx += sizeof(float);

	//f_sys_time	4	float	8	sec	system time
	memcpy(&buf[idx], &f_sys_time, sizeof(float));
	idx += sizeof(float);

	//phase_temp	6	int16 x3	14
	memcpy(&buf[idx], &phase_temp, sizeof(int16_t)*3);
	idx += sizeof(int16_t)*3;

	//gate_temp	2	int16	16
	memcpy(&buf[idx], &gate_temp, sizeof(int16_t));
	idx += sizeof(int16_t);

	//board_temp	2	int16	18
	memcpy(&buf[idx], &board_temp, sizeof(int16_t));
	idx += sizeof(int16_t);

	//rtd_temp	10	int16 x5	28
	memcpy(&buf[idx], &rtd_temp, sizeof(int16_t)*5);
	idx += sizeof(int16_t)*5;

	//motor_temp	2	int16	30
	memcpy(&buf[idx], &motor_temp, sizeof(int16_t));
	idx += sizeof(int16_t);

	//torque_shud	2	int16	32
	memcpy(&buf[idx], &torque_shud, sizeof(int16_t));
	idx += sizeof(int16_t);

	//torque_cmd	2	int16	34
	memcpy(&buf[idx], &torque_cmd, sizeof(int16_t));
	idx += sizeof(int16_t);

	//torque_fb	2	int16	36
	memcpy(&buf[idx], &torque_fb, sizeof(int16_t));
	idx += sizeof(int16_t);

	//analog_in	8	int16 x 4	44
	memcpy(&buf[idx], &analog_in, sizeof(int16_t)*4);
	idx += sizeof(int16_t)*4;

	//digital_in	6	uint8 x 6	50
	memcpy(&buf[idx], &digital_in, sizeof(int8_t)*6);
	idx += sizeof(int8_t)*6;

	//motor_angle	2	int16	52
	memcpy(&buf[idx], &motor_angle, sizeof(int16_t));
	idx += sizeof(int16_t);

	//motor_speed	2	int16	54
	memcpy(&buf[idx], &motor_speed, sizeof(int16_t));
	idx += sizeof(int16_t);

	//inv_freq	2	int16	56
	memcpy(&buf[idx], &inv_freq, sizeof(int16_t));
	idx += sizeof(int16_t);

	//resolver_angle	2	int16	58
	memcpy(&buf[idx], &resolver_angle, sizeof(int16_t));
	idx += sizeof(int16_t);

	//phase_current	6	int16 x 3	64
	memcpy(&buf[idx], &phase_current, sizeof(int16_t)*3);
	idx += sizeof(int16_t)*3;

	//dc_current	2	int16	66
	memcpy(&buf[idx], &dc_current, sizeof(int16_t));
	idx += sizeof(int16_t);

	//dc_voltage	2	int16	68
	memcpy(&buf[idx], &dc_voltage, sizeof(int16_t));
	idx += sizeof(int16_t);

	//output_volt	2	int16	70
	memcpy(&buf[idx], &output_volt, sizeof(int16_t));
	idx += sizeof(int16_t);

	//p_ab_volt	2	int16	72
	memcpy(&buf[idx], &p_ab_volt, sizeof(int16_t));
	idx += sizeof(int16_t);

	//p_bc_volt	2	int16	74
	memcpy(&buf[idx], &p_bc_volt, sizeof(int16_t));
	idx += sizeof(int16_t);

	//flux_cmd	2	int16	76
	memcpy(&buf[idx], &flux_cmd, sizeof(int16_t));
	idx += sizeof(int16_t);

	//flux_fb	2	int16	78
	memcpy(&buf[idx], &flux_fb, sizeof(int16_t));
	idx += sizeof(int16_t);

	//id_fb	2	int16	80
	memcpy(&buf[idx], &id_fb, sizeof(int16_t));
	idx += sizeof(int16_t);

	//iq_fb	2	int16	82
	memcpy(&buf[idx], &iq_fb, sizeof(int16_t));
	idx += sizeof(int16_t);

	//id_cmd	2	int16	84
	memcpy(&buf[idx], &id_cmd, sizeof(int16_t));
	idx += sizeof(int16_t);

	//iq_cmd	2	int16	86
	memcpy(&buf[idx], &iq_cmd, sizeof(int16_t));
	idx += sizeof(int16_t);

	//ref_1_5	2	int16	88
	memcpy(&buf[idx], &ref_1_5, sizeof(int16_t));
	idx += sizeof(int16_t);

	//ref_2_5	2	int16	90
	memcpy(&buf[idx], &ref_2_5, sizeof(int16_t));
	idx += sizeof(int16_t);

	//ref_5_0	2	int16	92
	memcpy(&buf[idx], &ref_5_0, sizeof(int16_t));
	idx += sizeof(int16_t);

	//sys_12v	2	int16	94
	memcpy(&buf[idx], &sys_12v, sizeof(int16_t));
	idx += sizeof(int16_t);

	//vsm_state	1	uint8	95
	memcpy(&buf[idx], &vsm_state, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//inv_state	1	uint8	96
	memcpy(&buf[idx], &inv_state, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//relay_state	1	uint8	97
	memcpy(&buf[idx], &relay_state, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//inv_mode	1	uint8	98
	memcpy(&buf[idx], &inv_mode, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//inv_cmd	1	uint8	99
	memcpy(&buf[idx], &inv_cmd, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//inv_enable	1	uint8	100
	memcpy(&buf[idx], &inv_enable, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//motor_direction	1	uint8	101
	memcpy(&buf[idx], &motor_direction, sizeof(uint8_t));
	idx += sizeof(uint8_t);

	//faults	8	uint8 x 8	109
	memcpy(&buf[idx], &faults, sizeof(uint8_t)*8);
	idx += sizeof(uint8_t)*8;

	//modulation_index	2	int16	111
	memcpy(&buf[idx], &modulation_index, sizeof(int16_t));
	idx += sizeof(int16_t);

	//flux_reg_out	2	int16	113
	memcpy(&buf[idx], &flux_reg_out, sizeof(int16_t));
	idx += sizeof(int16_t);

	//min_cell_temp	1	int8	114
	memcpy(&buf[idx], &min_cell_temp, sizeof(int8_t));
	idx += sizeof(int8_t);

	//max_cell_temp	1	int8	115
	memcpy(&buf[idx], &max_cell_temp, sizeof(int8_t));
	idx += sizeof(int8_t);

	//min_cell_volt	2	uint16	117
	memcpy(&buf[idx], &min_cell_volt, sizeof(uint16_t));
	idx += sizeof(uint16_t);

	//max_cell_volt	2	uint16	119
	memcpy(&buf[idx], &max_cell_volt, sizeof(uint16_t));
	idx += sizeof(uint16_t);

	// Payload Checksum
	static uint16_t  dta_chksum;
	dta_chksum = calculate_checksum(&buf[4], payload_lenght);
	memcpy(&buf[idx], &dta_chksum, sizeof(uint16_t));
	idx += sizeof(uint16_t);
#endif /* PRINT_STRING */

#if PRINT_DATA
#ifdef PRINT_STRING
	SerialUSB.print(buffer);
#else /* print binary */
	Serial.write(buf, packet_length);
#endif /* PRINT_STRING */
#endif

#if PRINT_DEBUG
	SerialUSB.print("Up time: " + String(((float)sys_time)/100) + "\r\n");
    SerialUSB.print("Timer: " + String(timer*0.003) + "\r\n");
    SerialUSB.print("max_cell_temp: " + String(max_cell_temp) + "\r\n");
    SerialUSB.print("vsm_state: " + String(vsm_state) + "\r\n");
    SerialUSB.print("sys_12v: " + String(sys_12v) + "\r\n");
#endif

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



