/**
 * @file IMU.cpp
 * @brief Source file for the IMU library for Dartmouth's 317 Lab.
 * 
 * There is extensive sensor and register information commented in this file.
 */
#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include<IMU.hpp>

//note - it's two byte data.
/** @copydoc initIMU() */
void initIMU(LIS3MDL* mag, LSM6* gyro_acc){
  Wire.begin();
  mag->init();
  gyro_acc->init();

// Notes to help Leah and possibly you:
    // CTRL5 controls temperature sensor, magnetic resolution selection, magnetic data rate collection, latch interrupt
    // CTRL6 does magnetic full-scale selection
    // CTRL7 does high-pass filter mode (acc), filtered acc data selection, temperature sensor only mode, manetic data low-power mode, and magnetic sensor mode selection (first two don't matter for magnetometer)
    
    /////////// Magnetometer NEW ///////////
    // Temp sensor enabled.
    // OM = 11 (ultra-high resolution mode)
    // DO = 111, 80 Hz ODR (output data rate)
    // FAST_ODR = 1 (enables ODR higher than 80 hz)
    // Self-tests disabled.
    // 0xFE = 0b11111110
    // Leah, 05.09.19
    mag->writeReg(mag->CTRL_REG1, 0xFE);
    // FS = 00 (+/- 4 gauss full scale). Ideal would be +/-1 gauss.
    // 0x00 = 0b00000000
    // Leah, 05.09.19
    mag->writeReg(mag->CTRL_REG2, 0x00);
    // LP = 0 (low power mode off)
    // SIM = 0 (default mode for 4-wire interface. 1 for 3-wire interface)
    // MD = 00 (continuous-conversion mode)
    // 0x00 = 0b00000000
    // Leah, 05.09.19
    mag->writeReg(mag->CTRL_REG3, 0x00);
    // Added to select Z-axis operative mode.
    // OMZ = 11 (ultra-high performance mode)
    // BLE = 0 (default. 1 for data Msb at lower address)
    // 0x0C = 0b00001100
    // Leah, 05.09.19
    mag->writeReg(mag->CTRL_REG4, 0x0C);  


    /////////// Accelerometer NEW /////////// (moved to LSM6 from LSM303)
    // ODR_XL[3:0] = 0100, 104 Hz ODR (sample rate) high performance
    // FS_XL: 10, full scale accelerometre range of +/- 4 g.
    // BW_XL: 01, anti-aliasing filter at 200 Hz (old library value was 194 Hz)
    // 0x49 = 0b01001001
    // Leah, 05.09.19
    gyro_acc->writeReg(gyro_acc->CTRL1_XL, 0x49);
    // Zen_XL = Yen_XL = Xen_XL = 1 (all axes enabled)
    // 0x38 = 0b00111000
    // Probably not necessary but here just to be safe
    // Leah, 05.09.19
    gyro_acc->writeReg(gyro_acc->CTRL9_XL, 0x38);
    // CTRL5_C controls self-test, but the default values for both acc. and gyro are 00, which disables self-test.

}
/** @copydoc getIMU */
void sampleIMU(LIS3MDL* mag, LSM6* imu, int16_t* data){
  mag->read();
  imu->read();
  data[0]=mag->m.x;
  data[1]=mag->m.y;
  data[2]=mag->m.z;
  data[3]=imu->a.x;
  data[4]=imu->a.y;
  data[5]=imu->a.z;
  data[6]=imu->g.x;
  data[7]=imu->g.y;
  data[8]=imu->g.z;
}
