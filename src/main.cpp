/**
 * @file main.cpp
 * @brief Main file for the 317 lab project.
 * 
 * When complete, will set sweep parameters and run the state machine.
 */
/*TODO - IN ORDER OF PRIORITY:
  - PDC tasks:
      - Should set massive buffer as member variable of PDC class and just run all data transfer through that. That also solves the global declaration issue.
        Data parser would have to change, because everything would get cast into 4 bytes. Would also have to check if we have enough memory to do this.s
      - Alternative is to interrupt when buffer is empty and reload. interrupting sweep is probably bad. ask jeff.
  - State machine questions:
      - figure out what to do with extra zeroes in data
      - Timing parameters will all need to change. Talk to Jeff about how to figure these out.
      - SWEEP_DELAY is currently set up with clock cycles. Should change to microseconds. Setting to 200 us for now.
      - Should move interrupt code to separate file at some point.
  - Convert AT25M02 documentation to doxygen
  - test IMU with proper initialization (I do not remember what I meant by this)
      - At some point, IMU should turn into a class with LIS3MDL and LSM6 as members
  - Maybe figure out a good way to calculate what the maximum serialized data buffer size needs to be and initialize in the pdc constructor.
    That only matters if we end up running out of memory on the board (which we might).-
  - Look into if the non-blocking I2C library would be helpful for making timing work out.
  
*/
/* NOTES:
Our DAC range is 0.55V to 2.76V. We can assume that a DAC value of 0 will give 0 at the sweep, and a value of 4095 will give 5V at the sweep. 
The DAC needs to be updated. It's dramed. In our application, it probably doesn't because we're constantly changing it. 
Wondering if the driver for this does it automatically.

State machine must be an integer number of sweeps per second. And need a bit of a buffer - a little more or less than 45 sweeps per second
In the state machine, there's a buffer period. idea is that the kicker should fall in that buffer period.
Timing will be completely different since this processor is so much faster. 
Can still use a buffer to end up at 45 sweeps per second. 
*/

//========== Libraries ==========//
#include <Arduino.h>
#include <Wire.h>
#include <Pip.hpp> //note, Max1148 isn't included because it's included in Pip
#include <PDC.hpp>
#include <AT25M02.hpp>
#include <PipController.hpp>
//========== For IMU ==========//
#include <IMU.hpp> // Library to initialize and sample IMU
#include <Wire.h>  // I2C communications
#include <LSM6.h>  // Library for the gyro/accelerometer
#include <LIS3MDL.h> // Library for the LIS3MDL 3-axis magnetometer

//========== Timing ==========//
// SAMPLE_PERIOD defined in sweep_values_v5.1h
#define BUFFER  1000  // buffer to wait period of missed measurement
#define SWEEP_OFFSET 500 // is this used?
#define RAM_BUFFER_DELAY 10  // Delay in seconds until the chip starts sending saved data.

//========== Debugging ==========//
void blink();
bool debug = true;

//========== Sweep Parameters ==========//
#define SWEEP_STEPS        28               // Number of steps in sweep
// Sweep range set with SHIELD_NUMBER in sweep_values_v5_1.h
#define SHIELD_NUMBER 15
#include "sweep_values_v5_1.h"

// These two set the step duration, which sets the sweep duration (11.23 ms).
// This works out to being 2.0 degrees rotation of the main payload while sweeping.
// Natural delay of ~76 us after setting DAC. 
// Adding 124 us delay to make it 200 us between DAC and ADC.
//^all those comments are old, leaving for now for reference

#define SWEEP_DELAY            200          // Delay between DAC and ADC in us -Sean
#define SWEEP_AVERAGES         8           // each sample ~20-21 us

//========== Class Declarations ==========//
PDC pdc;

Max1148 adc0(Channel::CHAN0);
Max1148 adc1(Channel::CHAN1);

Pip pip0(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, PIP_0_SWEEP_MIN, PIP_0_SWEEP_MAX, DAC0, adc0);
Pip pip1(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, PIP_1_SWEEP_MIN, PIP_1_SWEEP_MAX, DAC1, adc1);
PipController pipController(pip0, pip1);

LIS3MDL compass;
LSM6 gyro;
AT25M02 ram;

//========== Buffers and Messaging ==========//
int16_t IMUData[10];
uint32_t IMUTimeStamp;
// Use J & T for buffered messages (sentinel + 1)
uint8_t imuSentinel = 'I';
uint8_t sweepSentinel = 'S';
uint8_t interruptSentinel = 'B';
uint8_t messageSentinel = '#';
uint8_t* p_imuSentinel = &imuSentinel;
uint8_t* p_sweepSentinel = &sweepSentinel;
uint8_t* p_interruptSentinel = &interruptSentinel;
uint8_t* p_messageSentinel = &messageSentinel;

bool savedSweep = false;		// If we have a saved sweep to send
bool sendFromRam = false;		// When to send from ram
bool storeToRam = true;			// Save data to the ram chip

//========== Sweep Variable ==========//
uint8_t shieldID = 0;
uint8_t* p_shieldID = &shieldID;
uint16_t sweep0[SWEEP_STEPS];
uint16_t sweep1[SWEEP_STEPS];

//removed deprecated communication booleans

//========== Main Loop Timing ==========//
uint32_t startTime = 0;
uint32_t sweepStartTime = 0;
uint32_t sweepTimeStamp = 0;

//========== Buffer for Ram ==========//
#define IMU_TIMESTAMP_OFFSET 0
#define IMU_DATA_OFFSET     (sizeof(IMUTimeStamp) + IMU_TIMESTAMP_OFFSET)
#define SWEEP_TIMESTAMP_OFFSET (sizeof(IMUData) + IMU_DATA_OFFSET)
#define SWEEP_DATA_OFFSET (sizeof(sweepTimeStamp) + SWEEP_TIMESTAMP_OFFSET)
#define RAM_BUF_LEN  (sizeof(IMUTimeStamp) + sizeof(IMUData) + sizeof(sweepTimeStamp) + sizeof(sweep0) + sizeof(sweep1))
// Stores the IMU data then the Sweep data
uint8_t ramBuf[RAM_BUF_LEN];

//========== Interrupt Timing ==========//
#define SYNC_PIN 2
volatile uint32_t timer;
volatile bool syncPulse;
volatile bool newCycle;
void configureTimerInterrupt();
void syncHandler();

//========== Finite State Machine States ==========//
enum BobState {
	idle,
	startSweep,
	sendSweep,
	takeIMU,
	sendIMU,
	sendStored,
	sendTimeStamps,
	waitForSweep,
	getSweep,
	waitForNewCycle,
	interrupted
};
BobState currentState = idle;

//FSM function prototypes
void FSMUpdate();
void FSMAction();
void startSweepOnShield();
void sendIMUData();
void sendSweepData();
void takeIMUData();
void sendStoredData();
void sendInterruptTimeStamps();
void getSweepData();

void setup() {
	if(debug){
        Serial.begin(230400);
        pdc.init();
        /* SPI.begin();
        startTime=micros();
        startSweepOnShield();
        sendSweepData();  */
/*         SPI.begin();
        initIMU(&compass, &gyro);
        pdc.init();
        sampleIMU(&compass, &gyro, IMUData);
        pdc.send(IMUData, sizeof(IMUData)/sizeof(IMUData[0]));
 */	}
	else{
		// Configure serial, 230.4 kb/s baud rate
		Serial.begin(230400); 
		Serial.print("PIP_0_SWEEP_MIN = ");
		Serial.println(PIP_0_SWEEP_MIN);
		Serial.print("PIP_0_SWEEP_MAX = ");
		Serial.println(PIP_0_SWEEP_MAX);
		Serial.print("PIP_1_SWEEP_MIN = ");
		Serial.println(PIP_1_SWEEP_MIN);
		Serial.print("PIP_1_SWEEP_MAX = ");
		Serial.println(PIP_1_SWEEP_MAX);  

		// Setup IMU
		initIMU(&compass, &gyro);

		// Setup RAM
		ram.init();

		// Setup PDC
		pdc.init();

        //setup SPI
        SPI.begin();

		// Initialize time
		startTime = micros(); 

		// Configure the timer interrupt
		configureTimerInterrupt();
		//configure the external interrupt
		attachInterrupt(digitalPinToInterrupt(SYNC_PIN), syncHandler, RISING);
	}
}

void loop() {
	if(debug){
        /* sampleIMU(&compass, &gyro, IMUData);
        for(int i = 0; i < 10; i++){
            Serial.print(IMUData[i]);
            Serial.print(" ");
        }
        Serial.println();
        delay(2000); */
	}
	else{
		FSMUpdate();
  		FSMAction();
	}
}

/**
 * @brief Finite State Machine update function.
 * 
 * This function updates the state machine based on the current state.

 */
void FSMUpdate(){
  switch(currentState){
    case idle: {
        if (syncPulse) {
            syncPulse = false;
            return;
        }
        volatile uint32_t irq_state = __get_PRIMASK();  
        __disable_irq();                       
        if (micros() - timer > SWEEP_OFFSET) {
            currentState = startSweep;
        }
        __set_PRIMASK(irq_state);  
        break;
    }
    case startSweep: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = sendSweep;
        }
        break;
    }
    case sendSweep: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = takeIMU;
        }
        break;
    }
    case takeIMU: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = sendIMU;
        }
        break;
    }
    case sendIMU: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = sendStored;
        }
        break;
    }
    case sendStored: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = sendTimeStamps;
        }
        break;
    }
    case sendTimeStamps: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = waitForSweep;
        }
        break;
    }
    case waitForSweep: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = getSweep;
        }
        break;
    }
    case getSweep: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = waitForNewCycle;
        }
        break;
    }
    case waitForNewCycle: {
        if (newCycle || syncPulse) {
            currentState = idle;
            newCycle = false;
            syncPulse = false;
        }
        break;
    }
    case interrupted: {
        currentState = waitForNewCycle;
        break;
    }
    default: {
        currentState = idle;
        break;
    }
}


}

/**
 * @brief Finite State Machine action function.
 * 
 * This function performs the action associated with the current state.
 */
void FSMAction(){
	switch(currentState){
		case idle:
			break;
		case startSweep:
			startSweepOnShield();
			break;
		case sendSweep:
			sendSweepData();
			break;
		case takeIMU:
			takeIMUData();
			break;
		case sendIMU:
			sendIMUData();
			break;
		case sendStored:
			sendStoredData();
			break;
		case sendTimeStamps:
			sendInterruptTimeStamps();
			break;
		case waitForSweep:
			break;
		case getSweep:
			getSweepData();
			break;
		case waitForNewCycle:
			break;
		case interrupted:
			savedSweep = false;
			break;
		default:
			break;
	}
}

/**
 * @brief Blinks the onboard LED. Used for debugging.
 */
void blink(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}

/**
 * @brief Interrupt handler for the TC0 timer.
 */
void TC0_Handler(){
  // Clear the status register. This is necessary to prevent the interrupt from being called repeatedly.
  TC_GetStatus(TC0, 0);
  if (micros() - timer >= SAMPLE_PERIOD) {
		newCycle = true;
		timer = micros();
	}
}

void syncHandler(){
  timer = micros();
	syncPulse = true;
}

/**
 * @brief Configures timer counter for interrupt
 * Documentation for internal functions - see tc.c (.platformio/packages/framework-arduino-sam/system/libsam/source/tc.c)
 * The processor has 3 clocks, each have 3 channels and 3 registers (RA, RB, RC).
 * This is set up to use TC0 and channel 0. RC is used to store the compare value.
 * The interrupt is triggered when the counter reaches the compare value. Calculate interrupt frequency with:
 * interrupt frequency = clock frequency / (RC+1)
 * Note that the clock frequency for TC0 is configured to be 656.25 kHz.
 */
void configureTimerInterrupt(){
  // Enable the clock to the TC0 peripheral
  pmc_enable_periph_clk(ID_TC0);

  /*Configures the timer:
    - First two parameters set it to RC compare waveform mode. This means the timer resets when it reaches the value in RC.
    - The third parameter sets the clock source to MCK/128. MCK is at 84 MHz, so this sets the clock to 656.25 kHz.
  */
  TC_Configure(TC0, 0, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  //RC sets the value that the counter reaches before triggering the interrupt
  //This sets it to 10kHz
  TC_SetRC(TC0, 0, 64.625); // 

  // Enable the interrupt RC compare interrupt
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Disable all other TC0 interrupts
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  TC_Start(TC0, 0);
}

/**
 * @brief Starts the sweep.
 * Note that we no longer have to get the sweep data separately
 */
void startSweepOnShield(){
	sweepStartTime = micros();
	sweepTimeStamp = sweepStartTime - startTime;
	pipController.sweep();
}


void takeIMUData(){
    IMUTimeStamp = micros() - startTime;
    sampleIMU(&compass, &gyro, IMUData);
}
void sendSweepData(){
    if(!savedSweep){
        return;
    }
    pdc.send(p_messageSentinel, sizeof(messageSentinel));
    pdc.send(p_messageSentinel, sizeof(messageSentinel));
    pdc.send(p_sweepSentinel, sizeof(sweepSentinel));
/*     uint32_t* p_sweepTimeStamp = &sweepTimeStamp;
    pdc.send(p_sweepTimeStamp, sizeof(sweepTimeStamp));
    pdc.send(p_shieldID, sizeof(shieldID));
 */    
    uint16_t* p_sweep0 = pip0.get_data();
    uint16_t* p_sweep1 = pip1.get_data();
    int size0 = pip0.get_data_size();
    int size1 = pip1.get_data_size();
    pdc.send(p_sweep0, size0);
    pdc.send(p_sweep1, size1);
    savedSweep = false;

}

void sendIMUData(){

}

void sendStoredData(){

}

void sendInterruptTimeStamps(){

}

void getSweepData(){

}
