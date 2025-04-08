/*!
\mainpage Shield Code 6.0 Documentation

# Shield Code 6.0
This project manages the shield firmware for the Lynch Rocket Lab at Dartmouth. The code is designed for a custom PCB modeled off of the Arduino Due, and the code uses the Arduino libraries and framework.
The project is built in Platformio, which is the current industry standard for embedded development. This also means the code is in C++, rather than Arduino .ino

The project uses a finite state machine to manage screen voltage sweeps, ADC sampling, IMU sampling, RAM read/writes, and UART communication. This is described in Ruth Nordhoff's thesis, which members of the lab can access [here](https://drive.google.com/file/d/1XH5SRRh2m84pu7f2B1RSFuYGV1u3EBdw/view?usp=sharing).

One major difference in this version of the firmware is that it is entirely object oriented. This is mean to increase data security and make it easier for physicists to modify readable code in main.cpp while abstracting more complicated code to libraries. I highly recommend that updates to this firmware maintain the object oriented structure.

Members of the 317 lab can access my notes on designing the board and known problems [here](https://docs.google.com/document/d/1zxoe2ke9uMofQnCEMGJmY7DL-JV_K1uOB_eNFqKSiaQ/edit?usp=sharing)


## Programming the Board
GSE-D is configured to program the board. Simply connect to the board with USB, cd into the project repository and run 

   pio run

Make sure you fetch and pull changes from git before programming the board.

## Notes and Quirks
The board uses an external crystal oscillator, rather than the included ceramic oscillator on the Due. This required some changes that are not included in this documentation. First, a modded_system_sam3xa.c file is included in src/ to change the startup clock settings. Then, replace_libsam.py replaces the gcc_rel.a file that contains the precompiled startup code with our modded version. This required manually including the CMSIS libraries in /src/.
Due to some interrupt handling business explained in the PDC section of the documentation, we have to use a modified version of the Arduino framework, which is stored on my personal repository and included in the platformio configuration file. Whoever replaces me when I graduate should fork this repository to ensure it isn't lost when I lose access to my Dartmouth email. 

## Documentation
The documentation is maintained with Doxygen. A workflow in the main branch automatically generates and pushes the documentation to this website. Ensure neither Doxyfile nor layout.xml are removed from the main branch.
*/
/**
 * @file main.cpp
 * @brief Main file for the 317 lab project.
 * 
 * Contains shield initialization, state machine implementation and functions, and interrupt handling.
 * 
 */

//========== Libraries ==========//
#include <Arduino.h>
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
#define SWEEP_OFFSET           500
#define RAM_BUFFER_DELAY 10  // Delay in seconds until the chip starts sending saved data.

//========== Debugging ==========//
void blink();
bool debug = false;
int cycle_counter = 0;

//========== Sweep Parameters ==========//
#define SWEEP_STEPS        28               // Number of steps in sweep
// Sweep range set with SHIELD_NUMBER in sweep_values_v5_1.h
#define SHIELD_NUMBER 5
#include "sweep_values_v5_1.h"

// These two set the step duration, which sets the sweep duration (11.23 ms).
// This works out to being 2.0 degrees rotation of the main payload while sweeping.
// Natural delay of ~76 us after setting DAC. 
// Adding 124 us delay to make it 200 us between DAC and ADC.

#define SWEEP_DELAY            46.875          // Old version was 1500 clock cycles on a 32 MHz processor. comes out to this in us
#define SWEEP_AVERAGES         8           // each sample ~20-21 us

//========== Class Declarations ==========//
PDC pdc;

Max1148 adc0(Channel::CHAN2);
Max1148 adc1(Channel::CHAN1);

Pip pip0(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, 339, 3752, DAC0, adc0);
Pip pip1(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, 339, 3752, DAC1, adc1);
PipController pipController(pip0, pip1);

LIS3MDL compass;
LSM6 gyro;
AT25M02 ram;

//========== Sweep Variable ==========//
uint8_t shieldID = 60;

//========== Buffers and Messaging ==========//
int16_t IMUData[10];
uint32_t IMUTimeStamp;
uint32_t *p_IMUTimeStamp = &IMUTimeStamp;
// Use J & T for buffered messages (sentinel + 1)
uint8_t sweepSentinel[3] = {'#', '#', 'S'};       // 3 bytes: "##S"
uint8_t sweepSentinelBuf[3] = {'#', '#', 'T'};    // 3 bytes: "##T"
uint8_t imuSentinel[3] = {'#', '#', 'I'};
uint8_t imuSentinelBuf[3] = {'#', '#', 'J'};

bool savedSweep = false;		// If we have a saved sweep to send
bool sendFromRam = false;		// When to send from ram
bool storeToRam = true;			// Save data to the ram chip

//buffer for combined sweep data
uint16_t sweep_buffer[2*SWEEP_STEPS]; //112 bytes



//========== Main Loop Timing ==========//
uint32_t startTime = 0;
uint32_t sweepStartTime = 0;
uint32_t sweepTimeStamp = 0;
uint32_t *p_sweepTimeStamp = &sweepTimeStamp;

//========== Buffer for Ram ==========//
#define IMU_TIMESTAMP_OFFSET 0
#define IMU_DATA_OFFSET     (sizeof(IMUTimeStamp) + IMU_TIMESTAMP_OFFSET)
#define SWEEP_TIMESTAMP_OFFSET (sizeof(IMUData) + IMU_DATA_OFFSET)
#define SWEEP_DATA_OFFSET (sizeof(sweepTimeStamp) + SWEEP_TIMESTAMP_OFFSET)
#define RAM_BUF_LEN  (sizeof(IMUTimeStamp) + sizeof(IMUData) + sizeof(sweepTimeStamp) + sizeof(sweep_buffer)) //140 bytes, plus 7 bytes for sentinels/id
// Stores the IMU data then the Sweep data
uint8_t ramBuf[RAM_BUF_LEN];

const size_t totalSize = 294;
uint8_t memory_block[totalSize];
uint8_t* p_memory_block = memory_block;
//========== Interrupt Timing ==========//
#define SYNC_PIN 53
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
	waitForNewCycle,
	interrupted,
    store,
    read
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
void storeData();
void readData();
void sendData();

bool isFirst = true;

void setup() {
	if(debug){
      	// Configure serial, 230.4 kb/s baud rate
        //12.4 ms per message
		Serial.begin(230400); 
		// Setup IMU
		initIMU(&compass, &gyro);


		// Setup PDC
		pdc.init();
        SPI.begin();

		// Initialize time
		//startTime = micros();  
    }    
	else{
		// Configure serial, 230.4 kb/s baud rate
        delay(200);
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);

		Serial.begin(230400); 
		// Setup IMU
		initIMU(&compass, &gyro);

        SPI.begin();

		// Setup RAM
		ram.init();

		// Setup PDC - must be called after Serial.begin()
		pdc.init();

		// Initialize time
		startTime = micros(); 

		// Configure the timer interrupt
		configureTimerInterrupt();
		//configure the external interrupt
        pinMode(SYNC_PIN, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(SYNC_PIN), syncHandler, FALLING);
        pinMode(7, OUTPUT);
        
	}
}

void loop() {
	if(debug){
        takeIMUData();
        //sendIMUData();
        delay(500);
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
            if (isFirst){
                isFirst = false;
                timer = micros();
                newCycle = false;
            }  
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
            currentState = takeIMU;
        }
        break;
    }

    case takeIMU: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = read;
        }
        break;
    }
        case read:
            if (syncPulse) {
                currentState = interrupted;
                syncPulse = false;
            } else {
                currentState = store;
            }
            break;
        case store: {
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
    
/*
    case sendSweep: {
        if (syncPulse) {
            currentState = interrupted;
            syncPulse = false;
        } else {
            currentState = startSweep;
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
            currentState = waitForNewCycle;
        }
        break;
    }
    
    */
    
    
    
    


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
			//sendSweepData();
			break;
		case takeIMU:
			takeIMUData();
			break;
		case sendIMU:
			//sendIMUData();
			break;
		case sendStored:
			//sendStoredData();
			break;
		case waitForNewCycle:
			break;
        case store:
            storeData();
            break;
        case read:
            readData();
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
bool goLow = true;
void TC0_Handler(){
     if (goLow){
        goLow=false;
    } 
  // Clear the status register. This is necessary to prevent the interrupt from being called repeatedly.
  TC_GetStatus(TC0, 0);
  
  if (micros() - timer >= SAMPLE_PERIOD) {
        goLow=true; 
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
 * Documentation for internal functions - see tc.c (system/libsam/source/tc.c at https://github.com/swallace23/framework-arduino-sam)
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
  NVIC_SetPriority(TC0_IRQn, 0);

  TC_Start(TC0, 0);
}


 
/**
 * @brief Attaches last data buffer to UART PDC, runs DAC sweep, and saves pip data into a combined buffer.
 */
void startSweepOnShield(){
    //take timestamp
    int lastTime = sweepTimeStamp;
	sweepStartTime = micros();
	sweepTimeStamp = sweepStartTime - startTime;
    //this was here for debugging state machine timing discontinuities
     if(sweepTimeStamp-lastTime<22000){
        pinMode(6, OUTPUT);
        digitalWrite(6, HIGH);
    }
    else{
        pinMode(6, OUTPUT);
        digitalWrite(6, LOW);
    } 
    sendData();
	pipController.sweep();
    memcpy(sweep_buffer, pip0.data, SWEEP_STEPS*sizeof(uint16_t));
    //copy pip data into combined buffer
    memcpy(sweep_buffer + SWEEP_STEPS, pip1.data, SWEEP_STEPS*sizeof(uint16_t));
    savedSweep=true;
}


void takeIMUData(){
    IMUTimeStamp = micros() - startTime;
    sampleIMU(&compass, &gyro, IMUData);
}

void storeData(){
    if (storeToRam){
        ram.writeData((uint8_t *)&IMUTimeStamp, sizeof(IMUTimeStamp));
        ram.writeData((uint8_t *)IMUData, sizeof(IMUData));
        ram.writeData((uint8_t *)&sweepTimeStamp, sizeof(sweepTimeStamp));
        ram.writeData((uint8_t *)sweep_buffer, sizeof(sweep_buffer));
    }
}

void readData(){
    if(sendFromRam){
        ram.readData(ramBuf, RAM_BUF_LEN);
    }
}

int shortSize = sizeof(sweepSentinel)+sizeof(sweepTimeStamp)+sizeof(sweep_buffer)+sizeof(imuSentinel)+sizeof(IMUTimeStamp)+sizeof(IMUData);

void sendData(){
    if(!savedSweep){
        return;
    } 
    if (!sendFromRam && micros() - startTime > RAM_BUFFER_DELAY * 1000000) {
		sendFromRam = true;
	}
    p_memory_block = memory_block;
    if(sendFromRam && ram.usedBytes()>=RAM_BUF_LEN){
        // 1. Copy sweepSentinel (3 bytes: e.g., { '#', '#', 'S' }).
        memcpy(p_memory_block, sweepSentinel, sizeof(sweepSentinel));
        p_memory_block += sizeof(sweepSentinel);

        // 3. Copy sweep timestamp from ramBuf (4 bytes).
        memcpy(p_memory_block, p_sweepTimeStamp, sizeof(sweepTimeStamp));
        p_memory_block += sizeof(sweepTimeStamp);

        // 2. Copy payload ID (1 byte) using shieldID.
        memcpy(p_memory_block, &shieldID, sizeof(shieldID));
        p_memory_block += sizeof(shieldID);

        
        // 4. Copy sweep ADC data (sweep_buffer).
        memcpy(p_memory_block, sweep_buffer, sizeof(sweep_buffer));
        p_memory_block += sizeof(sweep_buffer);

        memcpy(p_memory_block, imuSentinel, sizeof(imuSentinel));
        p_memory_block += sizeof(imuSentinel);
        memcpy(p_memory_block, p_IMUTimeStamp, sizeof(IMUTimeStamp));
        p_memory_block += sizeof(IMUTimeStamp);
        memcpy(p_memory_block, IMUData, sizeof(IMUData));
        p_memory_block += sizeof(IMUData);
        memcpy(p_memory_block, imuSentinelBuf, sizeof(imuSentinelBuf));
        p_memory_block += sizeof(imuSentinelBuf);
        memcpy(p_memory_block, ramBuf + IMU_TIMESTAMP_OFFSET, sizeof(IMUTimeStamp));
        p_memory_block += sizeof(IMUTimeStamp);
        memcpy(p_memory_block, ramBuf + IMU_DATA_OFFSET, sizeof(IMUData));
        p_memory_block += sizeof(IMUData);
        memcpy(p_memory_block, sweepSentinelBuf, sizeof(sweepSentinelBuf));
        p_memory_block += sizeof(sweepSentinelBuf);
        memcpy(p_memory_block, ramBuf + SWEEP_TIMESTAMP_OFFSET, sizeof(sweepTimeStamp));
        p_memory_block += sizeof(sweepTimeStamp);
        memcpy(p_memory_block, &shieldID, sizeof(shieldID));
        p_memory_block += sizeof(shieldID);
        memcpy(p_memory_block, ramBuf + SWEEP_DATA_OFFSET, sizeof(sweep_buffer));

        p_memory_block = memory_block;
        pdc.send(memory_block, totalSize);
    } else {
        // Non-RAM branch:
        // 1. Copy sweepSentinel (3 bytes).
        memcpy(p_memory_block, sweepSentinel, sizeof(sweepSentinel));
        p_memory_block += sizeof(sweepSentinel);
        // 3. Copy sweep timestamp from p_sweepTimeStamp (4 bytes).
        memcpy(p_memory_block, p_sweepTimeStamp, sizeof(sweepTimeStamp));
        p_memory_block += sizeof(sweepTimeStamp);
        // 2. Copy payload ID (1 byte) as shieldID.
        memcpy(p_memory_block, &shieldID, sizeof(shieldID));
        p_memory_block += sizeof(shieldID);

     


        // 4. Copy sweep ADC data.
        memcpy(p_memory_block, sweep_buffer, sizeof(sweep_buffer));
        p_memory_block += sizeof(sweep_buffer);

        memcpy(p_memory_block, imuSentinel, sizeof(imuSentinel));
        p_memory_block += sizeof(imuSentinel);
        memcpy(p_memory_block, p_IMUTimeStamp, sizeof(IMUTimeStamp));
        p_memory_block += sizeof(IMUTimeStamp);
        memcpy(p_memory_block, IMUData, sizeof(IMUData));
        
        // Structure: [3-byte sentinel]["4-byte timestamp"]["1-byte payload ID"][sweep data]...
        p_memory_block += sizeof(IMUData);
        pdc.send(memory_block, shortSize);
    } 
    
}

/* void sendSweepData(){
    if(!savedSweep){
        return;
    }
    
    
    pdc.send(sweepSentinel, sizeof(sweepSentinel));
    pdc.send_next(p_sweepTimeStamp, sizeof(sweepTimeStamp));
    pdc.send_next(sweep_buffer, sizeof(sweep_buffer));
    savedSweep = false; 

}

void sendIMUData(){
    pdc.send(imuSentinel, sizeof(imuSentinel));
    pdc.send_next(p_IMUTimeStamp, sizeof(IMUTimeStamp));
    pdc.send(IMUData, sizeof(IMUData));
} */
/* void sendStoredData(){
     if (!sendFromRam && micros() - startTime > RAM_BUFFER_DELAY * 1000000) {
		sendFromRam = true;
	}
    // Send from ram when ready & have data
    if (sendFromRam && ram.usedBytes()>=RAM_BUF_LEN) {
        ram.readData(ramBuf, RAM_BUF_LEN);
        pdc.send(imuSentinelBuf, sizeof(imuSentinelBuf));
        pdc.send(ramBuf, sizeof(IMUTimeStamp));
        pdc.send(ramBuf + IMU_DATA_OFFSET, sizeof(IMUData));
        pdc.send(sweepSentinelBuf, sizeof(sweepSentinelBuf));
        pdc.send(ramBuf + SWEEP_TIMESTAMP_OFFSET, sizeof(sweepTimeStamp));
        pdc.send(ramBuf + SWEEP_DATA_OFFSET, sizeof(sweep_buffer));
    }
    // Write IMU then sweep data to the ram at once.
    if (storeToRam){
        ram.writeData((uint8_t *)&IMUTimeStamp, sizeof(IMUTimeStamp));
		ram.writeData((uint8_t *)IMUData, sizeof(IMUData));
		ram.writeData((uint8_t *)&sweepTimeStamp, sizeof(sweepTimeStamp));
        ram.writeData((uint8_t *)sweep_buffer, sizeof(sweep_buffer));
        //ram.writeData((uint8_t*)pip0.data, sizeof(pip0.data));
        //ram.writeData((uint8_t*)pip1.data, sizeof(pip1.data));
    } 
} */