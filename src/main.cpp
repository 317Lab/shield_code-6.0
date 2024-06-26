/**
 * @file main.cpp
 * @brief Main file for the 317 lab project.
 * 
 * When complete, will set sweep parameters and run the state machine.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Pip.hpp>
#include <PDC.hpp>
#include <Vector.h>

/*TODO - IN ORDER OF PRIORITY:
  - ADC TASKS:
      - Clean up code, write documentation
      - Confirm expected data output. It doesn't seem right that it's oscillating.
  - transfer old EEPROM library, test it
  - Set up gh repo, remember to change project name
  - test IMU with proper initialization
  - Set up state machine!!
  - figure out what's up with Joseph - maybe email Kristina?
  - At some point, IMU should turn into a class with LIS3MDL and LSM6 as members
  - Maybe figure out a good way to calculate what the maximum serialized data buffer size needs to be and initialize in the pdc constructor.
    That only matters if we end up running out of memory on the board (which we might).
*/

/*
Our DAC range is 0.55V to 2.76V. We can assume that a DAC value of 0 will give 0 at the sweep, and a value of 4095 will give 5V at the sweep. 
The DAC needs to be updated. It's dramed. In our application, it probably doesn't because we're constantly changing it. 
Wondering if the driver for this does it automatically.

See if there's any forums on using DMA with peripherals on Arduino. 

peripheral dma controller - you can buffer data in the memory. it can send the bytes while the code is doing other stuff. DOn't have to worry about
interrupts - the dma controller should handle it. Only one interrupt when it's done. 



Sweep stuff:
thinks dac refresh is the issue

State machine must be an integer number of sweeps per second. And need a bit of a buffer - a little more or less than 45 sweeps per second
In the state machine, there's a buffer period. idea is that the kicker should fall in that buffer period.
Timing will be completely different since this processor is so much faster. 
Can still use a buffer to end up at 45 sweeps per second. 
*/


/*
ADC is wired to:
CS -> 10
CH0 -> DAC0
CH1 -> DAC1
DIN and DOUT are both wired to that other device that i can't recognize

*/

/* OBJECT INITIALIZATION:
Pip pipa(565, 8, 28, 0, 4095);

*/
void blink();
/* RingBuffer rxBuffer;
RingBuffer txBuffer;
PDC pdc(&rxBuffer, &txBuffer); */

Max1148 adc;

Pip pipa(565, 8, 28, 0, 4095);

/* uint16_t test_imu_data = 0x1234;
uint32_t test_imu_timestamp = 0xFFFFFFFF;
uint16_t test_sweep[] = {0x1232, 0x1233, 0x1234};
int test_sweep_size = 3;
uint32_t test_sweep_timestamp = 0xFFFFFFFF;
uint8_t test_buffer_data = 1;
uint32_t test_buffer_timestamp = 0xFFFFFFFF; */

void setup() {
  Serial.begin(9600);
  adc.init();
  //Serial.println("adc initialized");
  //pdc.init();
  //pdc.send(test_imu_data, test_imu_timestamp, test_sweep, test_sweep_size, test_sweep_timestamp, test_buffer_data, test_buffer_timestamp);
  pipa.tester(adc); 

}

void loop() {

  blink();

}


void blink(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}
