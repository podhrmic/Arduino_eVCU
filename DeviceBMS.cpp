/*
 * DeviceBMS.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: fwmav
 */

#include "DeviceBMS.h"

DeviceBMS::DeviceBMS() {
	bus = &CAN2;//because we want CAN0 now
	can_init_rlecs();

	max_cell_temp = 0;
	min_cell_temp = 100;
	max_cell_volt = 0;
	min_cell_volt = -1;
}

void DeviceBMS::can_init_rlecs() {
    // MLEC setup
    status = MLECUninit;
	//status = MLECRunning;
    memset(rlecs, 0, sizeof(rlecs));
    rlec_idx = 0;
    rlec_txoffset = 0x40;
    mlec_init_broadcast();
    mlec_init_msgs();
    mlec_update_msg_ids(rlec_txoffset);
    request_sent = 0;
}

/**
 * Periodic CAN function
 *
 * Scan and ask for data
 */
 void DeviceBMS::can_periodic(){
    switch(status){
        case MLECUninit:
#if PRINT_DEBUG
        SerialUSB.print("MLEC Uninit \r\n");
#endif
            if (mlec_broadcast() && mlec_send_request()) {
#if PRINT_DEBUG
              SerialUSB.print("Request sent");
#endif
                request_sent = 1;
                status = MLECInit;
            }
            break;
        case MLECInit:
#if PRINT_DEBUG
        	SerialUSB.print("Scanned RLEC: " + String(rlec_idx) + "\r\n");
        	SerialUSB.print("Found: " + String(rlecsX[rlec_idx].status) + "\r\n");
#endif

            if (rlec_idx < (NUM_RLECS-1)) {
                // Scanning
                rlec_idx++;
                rlec_txoffset = rlec_txoffset+0x2;
                mlec_update_msg_ids(rlec_txoffset);
                mlec_send_request();
                request_sent = 1;
            }
            else {
                rlec_idx = 0;
                rlec_txoffset = 0x40;
                status = MLECRunning; // Scan complete
            }
            break;
        case MLECRunning:
            // Good for now, although it looses time if waiting for rlecs that arent
            // present, maybe some resolution to move up idx to actual values of RLECS
            // (only 1 in the array)
            if (rlecsX[rlec_idx].status == Active) {
                // RLEC idx present
                mlec_update_msg_ids(rlec_txoffset+0x2*rlec_idx);

                // Modify messages for charger (TODO)
                //mlec_charger_periodic(&rlecsX[rlec_idx],&tMsg0);

                mlec_send_request();

                //Telemetry (OPTIONAL)
#if PRINT_DEBUG
    		send_rlec_info(&(rlecsX[rlec_idx]));
#endif
            }
            //set messages to default
            mlec_init_msgs();
            rlec_idx++;
            rlec_idx = rlec_idx % NUM_RLECS;
            break;
        default:
            break;
    }
}

 /*
  * Charger function
  *
  */
 void mlec_charger_periodic(RLECModule* module, CAN_FRAME* msg6 ) {
  uint32_t balance_resistors = 0;
  if (STANDARD_CHARGE_MODE){
     //debuglink.printf("STANDARD CHARGE MODE\r\n");
     // standard charge mode
     if ((module->max_cell_volt >= BAT_CHARGE_CUTOFF) || (module->charge_status == 1)) {
         // switch off charger
         if (digitalRead(HLIM_PIN) == HIGH) {
             digitalWrite(HLIM_PIN, LOW);
             module->charge_status = 1;
         }


         // update status
         //TBD

         for (uint8_t j = 0; j<RLEC_CELLS;j++) {
         // if any cell is more than delta from the lowest cell
             if ((module->cell_voltage[j] - module->min_cell_volt) >= BAT_DELTA_BALANCE) {
                 balance_resistors = balance_resistors + (0x1 << j);
             }
             //debuglink.printf("delta: %d\r\n", (module->cell_voltage[j] - module->min_cell_volt));
         }

         if (balance_resistors == 0) {
             module->charge_status = 0;
         }
         //debuglink.printf("delta: %d\r\n", BAT_DELTA_BALANCE);

         //debuglink.printf("BALANCING\r\n");
     }
  }
  else {
 //debuglink.printf("BALANCE CHARGE MODE\r\n");
  // COOL BALANCE MODE
     // we dont want charging and balancing at the same time
     //hlim = 1;
     for (uint8_t j = 0; j<RLEC_CELLS;j++) {
         if (module->cell_voltage[j] > BAT_BALANCE_CUTOFF) {
             balance_resistors = balance_resistors + (0x1 << j);
         }
     }
  }
 //debuglink.printf("balance resistors: %X\r\n", balance_resistors);
 // Configure message to given RLEC
 msg6->data.bytes[3] = (uint8_t)(balance_resistors >> 8);
 msg6->data.bytes[4] = (uint8_t)(balance_resistors & 0xFF);
 }


void DeviceBMS::send_rlec_info(RLECModule* rlec) {
	SerialUSB.print(">>> RLEC_" + String(rlec_idx) + "\r\n");
	SerialUSB.print("max_cell_volt: " + String((float)(rlec->max_cell_volt*RLEC_CAN_VOLTAGE_MULT)) + "[V]\r\n");
	SerialUSB.print("min_cell_volt: " + String((float)(rlec->min_cell_volt*RLEC_CAN_VOLTAGE_MULT)) + "[V]\r\n");

	SerialUSB.print("rlec_temp: " + String(rlec->rlec_temp) + "[C]\r\n");
	SerialUSB.print("balance_resistors: " + String(rlec->balance_resistors) + "\r\n");
	SerialUSB.print("faults: " + String(rlec->faults) + "\r\n");

	SerialUSB.print("rlec_volt: " + String((float)(rlec->rlec_volt*RLEC_CAN_MODULE_MULT)) + "[V]\r\n");
	SerialUSB.print("max_cell_temp: " + String(rlec->max_cell_temp) + "[C]\r\n");
	SerialUSB.print("min_cell_temp: " + String(rlec->min_cell_temp) + "[C]\r\n");
	SerialUSB.print("\r\n");
}

void DeviceBMS::update_battery_data(){
  	max_cell_temp = 0;
	min_cell_temp = 100;
	max_cell_volt = 0;
	min_cell_volt = -1;
	for (int i=0;i<NUM_RLECS;i++){
		if (rlecsX[i].status == Active) {
			// check max cell temp
			if (rlecsX[i].max_cell_temp > max_cell_temp) {
				max_cell_temp = rlecsX[i].max_cell_temp;
			}

			// check min cell temp
			if (rlecsX[i].min_cell_temp < min_cell_temp) {
				min_cell_temp = rlecsX[i].min_cell_temp;
			}

			// check max cell voltage
			if (rlecsX[i].max_cell_volt > max_cell_volt) {
				max_cell_volt = rlecsX[i].max_cell_volt;
			}

			// check min cell voltage
			if (rlecsX[i].min_cell_volt < min_cell_volt) {
				min_cell_volt = rlecsX[i].min_cell_volt;
			}
		}
	}
}


/**
 * Check if we received messages
 */
void DeviceBMS::process() {
	if (bus->rx_avail()) {
		bus->get_rx_buff(rx_msg);
                //SerialUSB.print("Msg id is " + String(rx_msg.id) + "\r\n");
        if (status == MLECRunning) {
           // Normal operation
           rlec_parse_msg();
           // handleCanFrame(rx_msg);
        }
        else {
           // Scan operation
           rlec_parse_scan(rx_msg.id);
        }
	}
}


/**
 * Parse message during normal operation
 */
void DeviceBMS::rlec_parse_msg(void) {
    static uint8_t msg_offset, msg_id, idx;
    msg_offset = (uint8_t)(rx_msg.id >> 4);
    msg_id = (uint8_t)(rx_msg.id & 0xF);
    idx = msg_offset/2;

    if (idx >= NUM_RLECS) idx = 0;
    switch(msg_id) {
        case 0x1: // Cell Voltage 1-4
            rlecsX[idx].cell_voltage[0] = (uint16_t)(rx_msg.data.bytes[0] << 8 | rx_msg.data.bytes[1]);
            rlecsX[idx].cell_voltage[1] = (uint16_t)(rx_msg.data.bytes[2] << 8 | rx_msg.data.bytes[3]);
            rlecsX[idx].cell_voltage[2] = (uint16_t)(rx_msg.data.bytes[4] << 8 | rx_msg.data.bytes[5]);
            rlecsX[idx].cell_voltage[3] = (uint16_t)(rx_msg.data.bytes[6] << 8 | rx_msg.data.bytes[7]);
            break;
        case 0x2: // Cell Voltage 5-8
            rlecsX[idx].cell_voltage[4] = (uint16_t)(rx_msg.data.bytes[0] << 8 | rx_msg.data.bytes[1]);
            rlecsX[idx].cell_voltage[5] = (uint16_t)(rx_msg.data.bytes[2] << 8 | rx_msg.data.bytes[3]);
            rlecsX[idx].cell_voltage[6] = (uint16_t)(rx_msg.data.bytes[4] << 8 | rx_msg.data.bytes[5]);
            rlecsX[idx].cell_voltage[7] = (uint16_t)(rx_msg.data.bytes[6] << 8 | rx_msg.data.bytes[7]);
            break;
        case 0x3: // Cell Voltage 9-12
            rlecsX[idx].cell_voltage[8] = (uint16_t)(rx_msg.data.bytes[0] << 8 | rx_msg.data.bytes[1]);
            rlecsX[idx].cell_voltage[9] = (uint16_t)(rx_msg.data.bytes[2] << 8 | rx_msg.data.bytes[3]);
            rlecsX[idx].cell_voltage[10] = (uint16_t)(rx_msg.data.bytes[4] << 8 | rx_msg.data.bytes[5]);
            rlecsX[idx].cell_voltage[11] = (uint16_t)(rx_msg.data.bytes[6] << 8 | rx_msg.data.bytes[7]);
            break;
        case 0x4: // Max/Min cell voltage, RLEC temp, balance, faults
            rlecsX[idx].max_cell_volt = (uint16_t)(rx_msg.data.bytes[0] << 8 | rx_msg.data.bytes[1]);
            rlecsX[idx].min_cell_volt = (uint16_t)(rx_msg.data.bytes[2] << 8 | rx_msg.data.bytes[3]);
            rlecsX[idx].rlec_temp = rx_msg.data.bytes[4];
            rlecsX[idx].balance_resistors = (uint16_t)( (rx_msg.data.bytes[5] << 8 | rx_msg.data.bytes[6]) & 0xFFF);
            rlecsX[idx].faults = rx_msg.data.bytes[7];
            break;
        case 0x5: // Unfiltered Cell Voltage 1-4
            // Not used
            break;
        case 0x6: // Unfiltered Cell Voltage 5-8
            // Not used
            break;
        case 0x7: // Unfiltered Cell Voltage 9-12
            // Not used
            break;
        case 0x8: // Raw Cell Voltage 1-4
            // Not used
            break;
        case 0x9: // Raw Cell Voltage 5-8
            // Not used
            break;
        case 0xA: // Raw Cell Voltage 9-12
            // Not used
            break;
        case 0xB: // RLEC module voltage,
            rlecsX[idx].rlec_volt = (uint16_t)(rx_msg.data.bytes[6] << 8 | rx_msg.data.bytes[7]);
            break;
        case 0xC: // Cell temp 1-8
            rlecsX[idx].cell_temp[0] = rx_msg.data.bytes[0];
            rlecsX[idx].cell_temp[1] = rx_msg.data.bytes[1];
            rlecsX[idx].cell_temp[2] = rx_msg.data.bytes[2];
            rlecsX[idx].cell_temp[3] = rx_msg.data.bytes[3];
            rlecsX[idx].cell_temp[4] = rx_msg.data.bytes[4];
            rlecsX[idx].cell_temp[5] = rx_msg.data.bytes[5];
            rlecsX[idx].cell_temp[6] = rx_msg.data.bytes[6];
            rlecsX[idx].cell_temp[7] = rx_msg.data.bytes[7];
            break;
        case 0xD: // Cell temp 9-12, min/max cell temp, SW id
            rlecsX[idx].cell_temp[8] = rx_msg.data.bytes[0];
            rlecsX[idx].cell_temp[9] = rx_msg.data.bytes[1];
            rlecsX[idx].cell_temp[10] = rx_msg.data.bytes[2];
            rlecsX[idx].cell_temp[11] = rx_msg.data.bytes[3];
            rlecsX[idx].max_cell_temp = rx_msg.data.bytes[4];
            rlecsX[idx].min_cell_temp = rx_msg.data.bytes[5];
            rlecsX[idx].sw_id = rx_msg.data.bytes[7];
            break;
        default:
            break;
        }
}


/**
 * Parse message during scan
 */
void DeviceBMS::rlec_parse_scan(int msg_id) {
  
    if ((msg_id >> 4) == (rlec_txoffset-0x40)) {//offset between Tx and Rx msgs for a particular RLEC
        // expected value arrived, notify periodic
        request_sent = 0;

        // write in buffer
        rlecsX[rlec_idx].status = Active;
    }
}


/**
 * mlec_broadcast
 *
 * Send broadcast messages
 */
uint8_t DeviceBMS::mlec_broadcast(void){
	/*
    static uint8_t counter = 0;
    counter = counter + canbus_mlec.write(mlec.bdc0);
    wait(0.007); // busy wait (NOP)
    counter = counter + canbus_mlec.write(mlec.bdc1);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc2);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc3);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc4);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.bdc5);
    wait(0.007);
    if (counter != 6){ // not all messages were OK
        return 0;
    }
    else return 1; // broadcast sucessful
    */
	bus->sendFrame(bdc0);
        delay(7);
	bus->sendFrame(bdc1);
        delay(7);
	bus->sendFrame(bdc2);
        delay(7);
	bus->sendFrame(bdc3);
        delay(7);
	bus->sendFrame(bdc4);
        delay(7);
	bus->sendFrame(bdc5);
        delay(7);
	return 1;
}


/**
 * mlec_send_request
 *
 * Send data request for a specific RLEC.
 * Call this function after setting the right msg ids
 */
uint8_t DeviceBMS::mlec_send_request(void){
	/*
    static uint8_t counter = 0;
    counter = counter + canbus_mlec.write(mlec.tMsg0);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg1);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg2);
    wait(0.007);
    counter = counter + canbus_mlec.write(mlec.tMsg3);
    if (counter != 4){ // not all messages were OK
        return 0;
    }
    else return 1; // send sucessful
    */
	bus->sendFrame(tMsg0);
        delay(7);
	bus->sendFrame(tMsg1);
        delay(7);
	bus->sendFrame(tMsg2);
        delay(7);
	bus->sendFrame(tMsg3);
        delay(7);
	return 1;
}


/**
 * mlec_update_msg_id
 *
 * Updates msg ids
 */
void DeviceBMS::mlec_update_msg_ids(uint16_t offset){
    offset = offset << 4;
    tMsg0.id = offset | 0x6;
    tMsg1.id = offset | 0xA;
    tMsg2.id = offset | 0xB;
    tMsg3.id = offset | 0xC;
}

/**
 * mlec_init_msgs
 *
 * Fills in data request messages
 */
void DeviceBMS::mlec_init_msgs(void){
    // from the TH!NK datasheet
    // MSG 0, id = 0xXX6
	tMsg0.length = 8;
    tMsg0.data.bytes[0] = 0x0F;
    tMsg0.data.bytes[1] = 0x0F;
    tMsg0.data.bytes[2] = 0xFF;
    tMsg0.data.bytes[3] = 0x00;
    tMsg0.data.bytes[4] = 0x00;
    tMsg0.data.bytes[5] = 0x00;
    tMsg0.data.bytes[6] = 0x00;
    tMsg0.data.bytes[7] = 0x00;
    tMsg0.extended  = false;

    // MSG 1, id = 0xXXA
    tMsg1.length = 8;
    tMsg1.data.bytes[0] = 0x00;
    tMsg1.data.bytes[1] = 0x00;
    tMsg1.data.bytes[2] = 0x00;
    tMsg1.data.bytes[3] = 0x00;
    tMsg1.data.bytes[4] = 0x00;
    tMsg1.data.bytes[5] = 0x00;
    tMsg1.data.bytes[6] = 0x00;
    tMsg1.data.bytes[7] = 0x00;
    tMsg1.extended = false;

    // MSG 2, id = 0xXXB
    tMsg2.length = 8;
    tMsg2.data.bytes[0] = 0x00;
    tMsg2.data.bytes[1] = 0x00;
    tMsg2.data.bytes[2] = 0x00;
    tMsg2.data.bytes[3] = 0x00;
    tMsg2.data.bytes[4] = 0x00;
    tMsg2.data.bytes[5] = 0x00;
    tMsg2.data.bytes[6] = 0x00;
    tMsg2.data.bytes[7] = 0x00;
    tMsg2.extended  = false;

    // MSG 3, id = 0xXXC
    tMsg3.length = 8;
    tMsg3.data.bytes[0] = 0x00;
    tMsg3.data.bytes[1] = 0x00;
    tMsg3.data.bytes[2] = 0x00;
    tMsg3.data.bytes[3] = 0x00;
    tMsg3.data.bytes[4] = 0x00;
    tMsg3.data.bytes[5] = 0x00;
    tMsg3.data.bytes[6] = 0x0C;
    tMsg3.data.bytes[7] = 0x0C;
    tMsg3.extended  = false;

}


/**
 * mlec_init_broadcast
 *
 * Fills in universal broadcast messages
 */
void DeviceBMS::mlec_init_broadcast(void){
    // from the TH!NK datasheet
    bdc0.id = 0x7e1;
    bdc0.length = 8;
    bdc0.data.bytes[0] = 0x01;
    bdc0.data.bytes[1] = 0x0C;
    bdc0.data.bytes[2] = 0x0C;
    bdc0.data.bytes[3] = 0x01;
    bdc0.data.bytes[4] = 0x01;
    bdc0.data.bytes[5] = 0x00;
    bdc0.data.bytes[6] = 0x05;
    bdc0.data.bytes[7] = 0xBD;
    bdc0.extended = false;;

    bdc1.id = 0x7e2;
    bdc1.length = 8;
    bdc1.data.bytes[0] = 0x00;
    bdc1.data.bytes[1] = 0x0A;
    bdc1.data.bytes[2] = 0x00;
    bdc1.data.bytes[3] = 0x0A;
    bdc1.data.bytes[4] = 0x00;
    bdc1.data.bytes[5] = 0x00;
    bdc1.data.bytes[6] = 0x01;
    bdc1.data.bytes[7] = 0x00;
    bdc1.extended = false;;

    bdc2.id = 0x7e3;
    bdc2.length = 8;
    bdc2.data.bytes[0] = 0x06;
    bdc2.data.bytes[1] = 0xB9;
    bdc2.data.bytes[2] = 0x06;
    bdc2.data.bytes[3] = 0xB9;
    bdc2.data.bytes[4] = 0x00;
    bdc2.data.bytes[5] = 0x09;
    bdc2.data.bytes[6] = 0x00;
    bdc2.data.bytes[7] = 0x03;
    bdc2.extended = false;;

    bdc3.id = 0x7e4;
    bdc3.length = 8;
    bdc3.data.bytes[0] = 0x00;
    bdc3.data.bytes[1] = 0x09;
    bdc3.data.bytes[2] = 0x46;
    bdc3.data.bytes[3] = 0x2D;
    bdc3.data.bytes[4] = 0x0A;
    bdc3.data.bytes[5] = 0x02;
    bdc3.data.bytes[6] = 0x00;
    bdc3.data.bytes[7] = 0x4B;
    bdc3.extended = false;;

    bdc4.id = 0x7e5;
    bdc4.length = 8;
    bdc4.data.bytes[0] = 0x05;
    bdc4.data.bytes[1] = 0x02;
    bdc4.data.bytes[2] = 0x03;
    bdc4.data.bytes[3] = 0x51;
    bdc4.data.bytes[4] = 0x03;
    bdc4.data.bytes[5] = 0xAE;
    bdc4.data.bytes[6] = 0x05;
    bdc4.data.bytes[7] = 0xCA;
    bdc4.extended = false;;

    bdc5.id = 0x7e6;
    bdc5.length = 2;
    bdc5.data.bytes[0] = 0x23;
    bdc5.data.bytes[1] = 0x1E;
    bdc5.extended = false;;
}
