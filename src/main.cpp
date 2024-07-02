/**
 * @file main.cpp
 * @brief Main file for the 317 lab project.
 * 
 * When complete, will set sweep parameters and run the state machine.
 */
/*TODO - IN ORDER OF PRIORITY:
  - Set up state machine!!
      - Fix for baud rate issue is probably USB. Try to figure out 422 adapter.
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
//Ring buffer for PDC
RingBuffer rxBuffer;
RingBuffer txBuffer;
PDC pdc(&rxBuffer, &txBuffer);

Max1148 adc(Channel::CHAN0);

Pip pip0(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, PIP_0_SWEEP_MIN, PIP_0_SWEEP_MAX);
Pip pip1(SWEEP_DELAY, SWEEP_AVERAGES, SWEEP_STEPS, PIP_1_SWEEP_MIN, PIP_1_SWEEP_MAX);

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
bool savedSweep = false;		// If we have a saved sweep to send
bool sendFromRam = false;		// When to send from ram
bool storeToRam = true;			// Save data to the ram chip

//========== Sweep Variable ==========//
uint8_t shieldID = 0;
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

void setup() {
  // Configure serial, 230.4 kb/s BAUD
  //temporarily at 9600 for testing over USB
	Serial.begin(9600); 
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

  // Initialize time
  startTime = micros(); 

  // Configure the timer interrupt
  configureTimerInterrupt();

}

void loop() {

}

/**
 * @brief Blinks the onboard LED. Formerly used for debugging.
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
  //This sets the value that the counter reaches before triggering the interrupt
  //This sets it to 10kHz
  TC_SetRC(TC0, 0, 64.625); // 

  // Enable the interrupt RC compare interrupt
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Disable all other TC0 interrupts
  TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);

  TC_Start(TC0, 0);
}