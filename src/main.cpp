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
