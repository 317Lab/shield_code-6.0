/**
 * @file IMU.hpp
 * @brief Header file for the IMU library for Dartmouth's 317 Lab.
 * 
 * This library is used to query the Pololu IMU. This is where we get location, acceleration, and gyroscopic data.
 * This is basically identical to the library used on the Uno based board, which was written by Max Roberts circa 2015, ed. Leah Ryu 2019.
 * 
 * Deprecated code was removed by Sean Wallace 2024-06-20.
 *
 * Author: Max Roberts
 * Date: 2015-06-22
 */

#ifndef IMU_HPP
#define IMU_HPP
#include <Arduino.h>
#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
/**
 * @brief Initializes the IMU. Sets settings for all used axes.
 * 
 * Info about registers and settings can be found commented in imu.cpp.
 */
void initIMU(LIS3MDL* compass, LSM6* gyro);
/**
 * @brief Gets the IMU data.
 */
void sampleIMU(LIS3MDL* compass, LSM6* gyro, int16_t* data);
#endif