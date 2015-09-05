/*
 * ichip_2128.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: fwmav
 */

#include "ichip_2128.h"

/*
 * Constructor. Assign serial interface to use for ichip communication
 */
ICHIPWIFI::ICHIPWIFI() {
	serialInterface = &Serial2;
	setup();

	paramCache.timeRunning = 0;//
	paramCache.torqueRequested = 0;//
	paramCache.torqueActual = 0;//
	paramCache.throttle = 0;//
	paramCache.brake = 0;//
	paramCache.speedActual = 0;//
	paramCache.dcVoltage = 0;//
	paramCache.dcCurrent = 0;//
	paramCache.bitfield1 = 0;
	paramCache.bitfield2 = 0;
	paramCache.bitfield3 = 0;
	paramCache.bitfield4 = 0;
	paramCache.running = false;//
	paramCache.faulted = false;//
	paramCache.warning = false;//
	paramCache.tempMotor = 0;//
	paramCache.tempInverter = 0;//
}




/*
 * Initialization of hardware and parameters
 */
void ICHIPWIFI::setup() {
	//MSEL pin
	pinMode(18, OUTPUT);
	digitalWrite(18, HIGH);

	//RESET pin
	pinMode(42, OUTPUT);
	digitalWrite(42, HIGH);

	tickCounter = 0;

	serialInterface->begin(115200);
}

/*
 * Convert into ichio friendly format
 */
void ICHIPWIFI::updateParamCache() {
	uint32_t ms = millis();
	paramCache.timeRunning = ms;

	paramCache.torqueRequested = device->torque_cmd/10;
	paramCache.torqueActual = device->torque_fb/10;//
//	paramCache.throttle = (device->analog_in[0]);// TODO
//	paramCache.brake = 0;//
	paramCache.speedActual = device->motor_speed;
	paramCache.dcVoltage = device->dc_voltage/10;//
	paramCache.dcCurrent = device->dc_current/10;//
//	paramCache.bitfield1 = 0;
//	paramCache.bitfield2 = 0;
//	paramCache.bitfield3 = 0;
//	paramCache.bitfield4 = 0;
	if (device->vsm_state == VSM_running) {
		paramCache.running = true;//
	}
	else {
		paramCache.running = false;//
	}

	if (device->faults[0] + device->faults[1] +
			device->faults[2] + device->faults[3] +
			device->faults[4] + device->faults[5] +
			device->faults[6] + device->faults[7]) {
		paramCache.faulted = true;//
	}
	else {
		paramCache.faulted = false;//
	}

	//paramCache.warning = false;//
	paramCache.tempMotor = device->motor_temp/10;
	paramCache.tempInverter = device->gate_temp/10;//
}

/*
 * Periodic updates of parameters to ichip RAM.
 * Also query for changed parameters of the config page.
 */
void ICHIPWIFI::handleTick() {
	tickCounter++;

	updateParamCache();


	// cmd torque
	setParam(Constants::torqueRequested, paramCache.torqueRequested);

	// torqueActual
	setParam(Constants::torqueActual, paramCache.torqueActual);

	// dcVoltage
	setParam(Constants::dcVoltage, paramCache.dcVoltage);


	// dcCurrent
	setParam(Constants::dcCurrent, paramCache.dcCurrent);

	// throttle
	setParam(Constants::throttle, paramCache.throttle);

	// brake
	setParam(Constants::brake, paramCache.brake);

	// RPM
	setParam(Constants::speedActual, paramCache.speedActual);

	// up time (1Hz)
	setParam(Constants::timeRunning, getTimeRunning());

	// running (?) 1Hz
	setParam(Constants::running, (paramCache.running ? Constants::trueStr : Constants::falseStr));

	// fault (?)
	setParam(Constants::faulted, (paramCache.faulted ? Constants::trueStr : Constants::falseStr));

	// warning?
	setParam(Constants::warning, (paramCache.warning ? Constants::trueStr : Constants::falseStr));

	//tempMotor
	setParam(Constants::tempMotor, paramCache.tempMotor);

	// tempInverter
	setParam(Constants::tempInverter, paramCache.tempInverter);

//	setParam(Constants::bitfield1, paramCache.bitfield1);
//	setParam(Constants::bitfield1, paramCache.bitfield2);
//	setParam(Constants::bitfield1, paramCache.bitfield3);
//	setParam(Constants::bitfield1, paramCache.bitfield4);

}


/*
 * Calculate the runtime in hh:mm:ss
   This runtime calculation is good for about 50 days of uptime.
   Of course, the sprintf is only good to 99 hours so that's a bit less time.
 */
char *ICHIPWIFI::getTimeRunning() {
	uint32_t ms = millis();
	int seconds = (int) (ms / 1000) % 60;
	int minutes = (int) ((ms / (1000 * 60)) % 60);
	int hours = (int) ((ms / (1000 * 3600)) % 24);
	sprintf(buffer, "%02d:%02d:%02d", hours, minutes, seconds);
	return buffer;
}



/*
 * Determine if a parameter has changed
 * The result will be processed in loop() -> processParameterChange()
 */
void ICHIPWIFI::getNextParam() {
	sendCmd("WNXT", GET_PARAM); //send command to get next changed parameter
}

/*
 * Try to retrieve the value of the given parameter.
 */
void ICHIPWIFI::getParamById(String paramName) {
	sendCmd(paramName + "?", GET_PARAM);
}

/*
 * Set a parameter to the given string value
 */
void ICHIPWIFI::setParam(String paramName, String value) {
	sendCmd(paramName + "=\"" + value + "\"", SET_PARAM);
}

/*
 * Set a parameter to the given int32 value
 */
void ICHIPWIFI::setParam(String paramName, int32_t value) {
	sprintf(buffer, "%l", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint32 value
 */
void ICHIPWIFI::setParam(String paramName, uint32_t value) {
	sprintf(buffer, "%lu", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given sint16 value
 */
void ICHIPWIFI::setParam(String paramName, int16_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint16 value
 */
void ICHIPWIFI::setParam(String paramName, uint16_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}

/*
 * Set a parameter to the given uint8 value
 */
void ICHIPWIFI::setParam(String paramName, uint8_t value) {
	sprintf(buffer, "%d", value);
	setParam(paramName, buffer);
}
/*
 * Set a parameter to the given float value
 */
void ICHIPWIFI::setParam(String paramName, float value, int precision) {
	char format[10];
	sprintf(format, "%%.%df", precision);
	sprintf(buffer, format, value);
	setParam(paramName, buffer);
}


//A version of sendCmd that defaults to SET_PARAM which is what most of the code used to assume.
void ICHIPWIFI::sendCmd(String cmd) {
	sendCmd(cmd, SET_PARAM);
}

/*
 * Send a command to ichip. The "AT+i" part will be added.
 * If the comm channel is busy it buffers the command
 */
void ICHIPWIFI::sendCmd(String cmd, ICHIP_COMM_STATE cmdstate) {
	serialInterface->write(Constants::ichipCommandPrefix);
	serialInterface->print(cmd);
	serialInterface->write(13);
}


void ICHIPWIFI::sendToSocket(int socket, String data) {
	char buff[6];
	sprintf(buff, "%03i", socket);
	String temp = "SSND%%:" + String(buff);
	sprintf(buff, ",%i:", data.length());
	temp = temp + String(buff) + data;
	sendCmd(temp, SEND_SOCKET);
}

