/**
 * @file Max1148.hpp
 * @brief Header file for the Max1148 library for Dartmouth's 317 Lab. Manages ADC communication through SPI.
 * 
 *  Basically just the Max1147 library from the Isinglass/Apophis shield code rewritten for the Arduino Due. 
 *  Written by Jacob at some point unknown to me.
 * 
 * See the data sheet for more information - https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1146-MAX1149.pdf
 * 
 * @author Sean Wallace
 * @date 2024-06-25
 */
#ifndef MAX1148_HPP
#define MAX1148_HPP
#include <Arduino.h>
#include <SPI.h>

#define SPI_SPEED 14000000
#define SPI_MODE SPI_MODE0
#define ADC_CHANNEL 0
#define ADC_PIN 10
#define ADC_CS_PIN 10

/**
 * @brief The commands for the Max1148 ADC.
 * READ is a dummy byte used to read the ADC.
 * CHAN0-7 are config commands to set the ADC channel.
 * As default, the ADC is set to single ended, unipolar conversion, and internal clock mode.
 */
enum Command{
    CHAN0 = 0x8E,
    CHAN1 = 0xCE,
    CHAN2 = 0x9E,
    CHAN3 = 0xDE,
    CHAN4 = 0xAE,
    CHAN5 = 0xEE,
    CHAN6 = 0xBE,
    CHAN7 = 0xFE,
    READ = 0x00
};

class Max1148{
    /**
     * @brief ADC functions should really only be used in relation to Pip measurements, so everything is private and Pip is a friend.
     */
    friend class Pip;
    private:
        int cs_pin;
        /**
         * @brief Sets the chip select pin to low.
         */
        void csl();
        /**
         * @brief Sets the chip select pin to high.
         */
        void csh();
        /**
         * @brief Resets the ADC. Must be called within a SPI transaction
         */
        void clear();
        void power_down();
        /**
         * @brief Reads an average ADC value from the Max1148 ADC. Uses Arduino's SPI library. 
         * 
         * @param avg_num The number of samples to average.
         */
        uint16_t adc_read_avg(int avg_number);
    public:
        /**
         * @brief ADC constructor. Sets cs_pin as per ADC_CS_PIN macro.
         */
        Max1148(): cs_pin(ADC_CS_PIN) {}

        /**
         * @brief Initializes the ADC. Initializes SPI bus and CS pin.
         * This is only a separate function because you can't initialize the SPI bus in a constructor (for some reason).
         */
        void init(){
            SPI.begin();
            pinMode(cs_pin, OUTPUT);
        }
        
};        


#endif