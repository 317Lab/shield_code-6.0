/**
 * @file Pip.hpp
 * @brief Header file for the Pip library for Dartmouth's 317 Lab.
 * 
 * This library manages the voltage sweep on the PIP sensors using the Arduino Due's onboard DAC. 
 * It is basically a vastly simplified version of the Pip, DAC, and Max1147 libraries in the Isinglass/Apophis shield code. Written by
 * Max Roberts in 2015.
 * We are able to use this simplified version because of how nice the Due is :)
 * 
 * 
 * @author Sean Wallace
 * @date 2024-06-20
 */
#ifndef PIP_HPP
#define PIP_HPP
#include <Arduino.h>
#include <Wire.h>
#include <Max1148.hpp>
//default sweep parameters. can be modified with the constructor.
#define SWEEP_DEFAULT_DELAY	1000
#define SWEEP_DEFAULT_AVG_NUM 25
#define SWEEP_DEFAULT_SAMPLES 28
#define SWEEP_DEFAULT_MIN 0
#define SWEEP_DEFAULT_MAX 4095
#define SWEEP_MAX_SAMPLES 256
#define DEFAULT_ADC_CHANNEL 0;

#define DAC_PIN DAC0


class Pip{
    friend class PipController;
    private:
        /**
         * @brief The pin for the DAC output.
         */
        uint8_t dac_pin;
        /**
         * @brief The delay between steps in the sweep. Measured in microseconds.
         */
        int delay_us;
        /**
         * @brief The ADC object for the Pip to use.
         * Assuming each pip will use a different ADC channel.
         */
        Max1148 &adc;
        /**
         * @brief The number of samples to average when querying the ADC.
         */
        uint16_t avg_num;
        /**
         * @brief The number of samples to take during the sweep. AKA the number of steps in the sweep.
         */
        uint16_t num_samples;
        /**
         * @brief The minimum and maximum values for the sweep.
         */
        uint16_t sweep_min;
        uint16_t sweep_max;
        /**
         * @brief The data array for the sweep.
         */
        uint16_t data[SWEEP_MAX_SAMPLES];

        /**
         * @brief Clears the data array.
         * 
         * @param data The data array to clear.
         * @param size The size of the data array.
         */
        void clear_data(uint16_t data[], uint16_t size);

        uint16_t read_adc();
    public:
        /**
         * @brief Constructor for the Pip class. All parameters modifiable.
         */
        Pip(int delay_us, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_pin, Max1148& adc);
        
        /**
         * @brief Sweeps the DAC output from min to max.
         * Step length, delay, and number of samples are all set by the constructor.
         */
        void sweep();
        /**
         * @brief Checks DAC and ADC readings. 
         * Modify for whatever is needed to test ADC functionality without compromising ADC class security.
         */
        void tester();


};
#endif