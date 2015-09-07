
#include <DueTimer.h>
#include "variant.h"
#include <due_can.h>
#include <genieArduino.h>
Genie genie;


#include "main.h"



/**
 * PIN macros
 */
#define PIN_ON(_x) digitalWrite(_x, HIGH)
#define PIN_OFF(_x) digitalWrite(_x, LOW)
#define PIN_TOGGLE(_x) pin_toggle(_x)

inline void pin_toggle(int pin) {
	int state = digitalRead(pin);
	if (state==HIGH) {
		digitalWrite(pin, LOW);
	}
	else {
		digitalWrite(pin,HIGH);
	}
}

/**
 * LED macros
 */
#define LED_ON(_x) PIN_ON(_x)
#define LED_OFF(_x) PIN_OFF(_x)
#define LED_TOGGLE(_x) PIN_TOGGLE(_x)

/**
 * Devices
 */
DeviceGEVCU* device;
DeviceBMS* bms;
#if USE_WIFI
ICHIPWIFI * ichip;
#endif



/**
 * Timer flags
 */
uint8_t flag_console;
uint8_t flag_telemetry;
uint8_t flag_failsafe;
uint8_t flag_bms_can;
uint8_t flag_rms_can;
uint8_t flag_sensors_input;
uint8_t flag_datalog;


/**
 * Timer callbacks
 */
void console_timer(void){
	flag_console = 1;
}

void telemetry_timer(void){
	flag_telemetry = 1;
}

void failsafe_timer(void){
	flag_failsafe = 1;
}

void bms_can_timer(void){
	flag_bms_can= 1;
}

void rms_can_timer(void){
	flag_rms_can= 1;
}

void sensors_input_timer(void){
	flag_sensors_input = 1;
}


/**
 * Systime
 * We are hoping for 10ms resolution
 */
void systime_timer(void){
	device->sys_time++;
}

void datalog_timer(void){
	flag_datalog = 1;
}


/**
 * Wifi stuff
 */
void sendWiReach(char* message)
{
	Serial2.println(message);
	delay(700);
	while (Serial2.available()) {
#if PRINT_DEBUG
		SerialUSB.write(Serial2.read());
#endif
	}
}

#define CURRENT_METER_SCREEN_ID 0x01
void initWiReach()
{
#if PRINT_DEBUG
	SerialUSB.begin(115200); // use SerialUSB only as the programming port doesn't work
#endif
}


/**
 * Setup Arduino
 */
void setup() {
	//wifi init
#if USE_WIFI
	initWiReach();
#endif

	// initialize pins
	// DOUT0 - no RTDS
	//pinMode(THROTTLE_OUT, OUTPUT);
	//PIN_OFF(THROTTLE_OUT);

	// DOUT1 - no throttle
	pinMode(RTDS_PIN, OUTPUT);
	PIN_OFF(RTDS_PIN);

	// DOUT2 - AMS ERR ON 
	pinMode(AMS_LED, OUTPUT);
	PIN_OFF(AMS_LED);

	// DOUT3 - FW_ENABLE OFF 
	pinMode(FW_ENABLE, OUTPUT);
	PIN_ON(FW_ENABLE);

	// DOUT4 - SHUTDOWN OPEN (OFF)
	pinMode(SHUTDOWN, OUTPUT);
	PIN_ON(SHUTDOWN);

	// DOUT6 - DC_ENABLE OFF
	//p		inMode(DC_ENABLE, OUTPUT);
	//PIN_OFF(DC_ENABLE);

	// DOUT7 - IMD_LED
	pinMode(IMD_LED, OUTPUT);
	PIN_OFF(IMD_LED);

	// BRAKE_EN
	pinMode(BRAKE_EN, INPUT);

	//IMD_STATUS
	pinMode(IMD_STATUS, INPUT);

	//setup ports
	Can0.begin(CAN_BPS_1000K); // RMS can == CAN

	// right now we are using Can1 (aka CAN2 for RMS)
	Can1.begin(CAN_BPS_500K); // BMS can == CAN2

	int filter;
	// just one frame
	Can0.setRXFilter(0, 0, 0, false);
	Can1.setRXFilter(0, 0, 0, false);

#if PRINT_DATA
	SerialUSB.begin(921600); // use SerialUSB only as the programming port doesn't work
#endif

	// Instantiate device
	device = new DeviceGEVCU();
	bms = new DeviceBMS();
#if USE_WIFI
	ichip = new ICHIPWIFI();
#endif

	/**
	 * Attach timers.
	 * It is calling for threads...
	 */
	// 0. datalog - not using datalog timer at this point
	//Timer0.attachInterrupt(datalog_timer).setFrequency(DATALOG_FREQUENCY).start();
	//delay(50); // to offset the timer firing a little bit

	// 1. heartbeat
	//Timer1.attachInterrupt(heartbeat_timer).setFrequency(HEARTBEAT_FREQUENCY).start();
	//delay(50); // to offset the timer firing a little bit

	// 2. console (serial over usb)
	Timer2.attachInterrupt(console_timer).setFrequency(CONSOLE_FREQUENCY).start();
	delay(50); // to offset the timer firing a little bit

	// 3. telemetry (wifi)
	Timer3.attachInterrupt(telemetry_timer).setFrequency(TELEMETRY_FREQUENCY).start();
	delay(50); // to offset the timer firing a little bit

	// 4. failsafe
	Timer4.attachInterrupt(failsafe_timer).setFrequency(FAILSAFE_FREQUENCY).start();
	delay(50); // to offset the timer firing a little bit

	// 5. BMS CAN
	Timer5.attachInterrupt(bms_can_timer).setFrequency(BMS_CAN_FREQUENCY).start();
	delay(50); // to offset the timer firing a little bit

	// 6. RMS CAN - we are not sending anything on RMS CAN
	//Timer6.attachInterrupt(rms_can_timer).setFrequency(RMS_CAN_FREQUENCY).start();
	//delay(50); // to offset the timer firing a little bit

	// 7. sensor inputs - we are not doing this AFAIK
	//Timer7.attachInterrupt(sensors_input_timer).setFrequency(SENSOR_INPUT_FREQUENCY).start();
	//delay(50); // to offset the timer firing a little bit

	// 8. system time
	Timer8.attachInterrupt(systime_timer).setFrequency(SYSTIME_FREQUENCY).start();
	delay(50); // to offset the timer firing a little bit
}

/**
 * Main loop, equivalent of while(1) {...}
 */
void loop() {
	// Process RMS CAN
	device->process();

	// Process BMS CAN
	bms->process();

	// handle periodic tasks based on trigger from timers
	handle_periodic_tasks();
}


/**
 * Handle periodic tasks
 *
 * Timer calls the function within an ISR, so it
 * should not be used for calling bulky or library functions
 * (such as printf). Instead it triggers the flag, which is
 * then handled here.
 */
inline void handle_periodic_tasks(void){
	if (flag_console) {
		//console_periodic();
		bms->update_battery_data();
		device->setMaxCellTemp(bms->getMaxCellTemp());
		device->setMinCellTemp(bms->getMinCellTemp());
		device->setMaxCellVolt(bms->getMaxCellVolt());
		device->setMinCellVolt(bms->getMinCellVolt());
		device->console_periodic();
		bms->vsm_state = device->getVSMState();
#if USE_WIFI
		ichip->handleTick();
#endif
		flag_console = 0;

	}
	if (flag_telemetry) {
		//telemetry_periodic();
		///device->console_periodic();
		flag_telemetry = 0;
	}
	// start failsafe checks only after T_MIN secs
	// so BMS have time to get CAN responses
	if (flag_failsafe ) {
		if (flag_failsafe && (device->sys_time > T_MIN)) {
			failsafe_periodic();
		}
		flag_failsafe = 0;
	}
	if (flag_bms_can) {
		bms->can_periodic();
		flag_bms_can = 0;
	}
	if (flag_rms_can) {
		//rms_can_periodic();
		flag_rms_can = 0;
	}
	if (flag_sensors_input) {
		//sensors_input_periodic();
		flag_sensors_input = 0;
	}
	if (flag_datalog) {
		//datalog_periodic();
		flag_datalog = 0;
	}
}

/**
 * Failsafe check
 *
 * This function will save your life is anything goes wrong.
 * At least in theory - split to different functions at different frequencies
 *
 */
inline void failsafe_periodic(void) {
	// check for RTDS sound
	static int rtds_cnt;
	if ((device->vsm_state == VSM_ready) && (rtds_cnt < 10)) {
		rtds_cnt++;
		PIN_ON(RTDS_PIN);
		//if (rtds_cnt > 0) {
		//	device->last_state = device->vsm_state;
		//}
	}
	else{
		rtds_cnt = 0;
		//device->last_state = device->vsm_state;
		PIN_OFF(RTDS_PIN);
	}
	
	// check rlecs for faults
	for (int i=0;i<NUM_RLECS;i++){
		if (bms->rlecsX[i].status == Active) {
			// critical faults
			if ((bms->rlecsX[i].faults & RLEC_CELL_1_AD_FAULT) != 0) {
				SerialUSB.print("!RLEC_CELL_1_AD_FAULT - shutting down...\r\n");  
				failsafe_shutdown();
			}
			else if ((bms->rlecsX[i].faults & RLEC_CELL_VOLTAGE_CONNECTION_FAULT) != 0) {
				SerialUSB.print("!RLEC_CELL_VOLTAGE_CONNECTION_FAULT - shutting down...\r\n");  
				failsafe_shutdown();
			}
			else if ((bms->rlecsX[i].faults & RLEC_CELL_VOLTAGE_AD_FAULT) != 0) {
				SerialUSB.print("!RLEC_CELL_VOLTAGE_AD_FAULT - shutting down...\r\n");  
				failsafe_shutdown();
			}
			else if ((bms->rlecsX[i].faults & RLEC_MODULE_VOLTAGE_AD_FAULT) != 0) {
				SerialUSB.print("!RLEC_MODULE_VOLTAGE_AD_FAULT - shutting down...\r\n");  
				failsafe_shutdown();
			}
			else if ((bms->rlecsX[i].faults & RLEC_CELL_1_VOLTAGE_FAULT) != 0) {
				SerialUSB.print("!RLEC_CELL_1_VOLTAGE_FAULT - shutting down...\r\n");  
				failsafe_shutdown();
			}

			// warnings
			else if ((bms->rlecsX[i].faults & RLEC_CELL_TEMP_AD_FAULT) != 0) {
				//debuglink.printf("!RLEC_CELL_TEMP_AD_FAULT - warning light on.\r\n");  
				failsafe_warning();
			}
			else if ((bms->rlecsX[i].faults & RLEC_RLEC_TEMP_AD_FAULT) != 0) {
				//debuglink.printf("!RLEC_RLEC_TEMP_AD_FAULT - warning light on.\r\n");  
				failsafe_warning();
			}

			// Charging - overcharge protection
			if (bms->rlecsX[i].max_cell_volt > MAX_CELL_VOLT) {
				charger_shutdown();
				//debuglink.printf("Charging Stopped.\r\n");  
			}

			// Voltage limits
			// warning if minimal allowed voltage reached
			if (bms->rlecsX[i].min_cell_volt < MIN_CELL_VOLT) {
				//batcritical_warning();
				failsafe_shutdown();
				SerialUSB.print("Minimal voltage reached - shutting down.\r\n");
				SerialUSB.print("RLEC " + String(i) + "\r\n");
				SerialUSB.print("Min voltage: " + String((float)bms->rlecsX[i].min_cell_volt*0.00244) + "\r\n");
			}
			// warning if cell below low threshold
			else if (bms->rlecsX[i].min_cell_volt < BAT_LOW) {
				//batlow_warning();
				//debuglink.printf("Warning - low voltage.\r\n"); 
			}
			// warning if cell below very low threshold
			else if (bms->rlecsX[i].min_cell_volt < BAT_VERY_LOW) {
				//batverylow_warning();
				//debuglink.printf("Warning - very low voltage.\r\n"); 
			}

			//Temperature limits
			if (bms->rlecsX[i].max_cell_temp > MAX_CELL_TEMP) {
				failsafe_shutdown();
				SerialUSB.print("Max cell temperature reached- shutting down.\r\n");
			}
		}
	}
}

/*
 * Charger stop (HLIM low)
 * HLIM: 1=ON, 0=OFF
 */
inline void charger_shutdown( void ) {
	//PIN_OFF(SHUTDOWN); // disable AIR
	failsafe_shutdown();
}

/*
 * Open main contactor (LLIM low)
 * LLIM: 1=ON, 0 = FF
 */
inline void failsafe_shutdown( void ) {
	PIN_OFF(FW_ENABLE); // disable FW_EN

	delay(500); // give some time to remove current from AIRs

	PIN_OFF(SHUTDOWN); // disable AIR

	failsafe_warning(); // light up LED
}

/*
 * Light up warning light
 */
inline void failsafe_warning( void ) {
	LED_ON(AMS_LED);
}

/*
 * Light up BatLow
 */
inline void batlow_warning( void ) {
	//LED_ON(BATLOW_PIN);
}

/*
 * Light up BatLow & reduce throttle
 */
inline void batverylow_warning( void ) {
	//LED_ON(BATVERYLOW_PIN);
}

/*
 * Battery ciritical (i.e. lowest allowed limit)
 */
inline void batcritical_warning( void ) {
	PIN_OFF(FW_ENABLE); // disable FW_EN

	delay(500); // give some time to remove current from AIRs

	failsafe_shutdown(); // open shutdown circuit
}

