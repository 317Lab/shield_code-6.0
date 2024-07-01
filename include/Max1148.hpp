/**
 * @file Max1148.hpp
 * @brief Header file for the Max1148 library for Dartmouth's 317 Lab. Manages ADC communication through SPI.
 * 
 * Each Max1148 object should correspond to an ADC channel. The channel is set in the constructor. 
 * This makes dealing with the 4 ADCs on the new set up quite simple.
 * 
 *  This is really just the Max1147 library from the Isinglass/Apophis shield code rewritten for the new board.
 *  Written by Jacob at some point unknown to me.
 * 
 * See the data sheet for more information - https://www.analog.com/media/en/technical-documentation/data-sheets/MAX1146-MAX1149.pdf
 * 
 * @author Sean Wallace
 * @date 2024-06-25
 */


/*
External clock mode is fine. Don't need to be concerned about noise. Means don't do the 8 us delay thing. 
Noise in the data sehet has more to do with the circuitb oard layout

Need to know that bc different control lines on the PC board. 

There are two SPI boxes. Use one for the memory and one for the ADC. Arduino should manage on its own.
    Does it give access to the second SPI? not that important.
*/
#ifndef MAX1148_HPP
#define MAX1148_HPP
#include <Arduino.h>
#include <SPI.h>

#define SPI_SPEED 2000000
#define SPI_MODE SPI_MODE0
#define ADC_CHANNEL 0
#define ADC_PIN 10
#define ADC_CS_PIN 10
#define ADC_READ 0x00

/**
 * @brief Control bytes for the Max1148 ADC.
 * Only modifiable parameter is the channel.
 * We use single ended, unipolar conversion, and external clock mode. Note that the old code used internal clock.
 */
enum class Channel : uint8_t{
    CHAN0 = 0x8F,
    CHAN1 = 0xCF,
    CHAN2 = 0x9F,
    CHAN3 = 0xDF,
    CHAN4 = 0xAF,
    CHAN5 = 0xEF,
    CHAN6 = 0xBF,
    CHAN7 = 0xFF
    };

class Max1148{
    /**
     * @brief ADC functions should really only be used in relation to Pip measurements, so everything is private and Pip is a friend.
     */
    friend class Pip;
    private:
        /**
         * @brief Chip select pin for the ADC.
         */
        int cs_pin;
        /**
         * @brief Channel for the ADC. Selected in the constructor.
         */
        uint8_t channel;
        /**
         * @brief Sets the chip select pin to low.
         */
        void csl();
        /**
         * @brief Sets the chip select pin to high.
         */
        void csh();
        /**
         * @brief Reads an average ADC value from the Max1148 ADC. Uses Arduino's SPI library. 
         * 
         * @param avg_num The number of samples to average.
         */
        uint16_t adc_read_avg(int avg_number);
        /**
         * @brief Reads a single ADC value from the Max1148 ADC. Used in adc_read_avg.
         */
        uint16_t adc_read();
    public:
        /**
         * @brief ADC constructor. Sets cs_pin as per ADC_CS_PIN macro. Chooses channel from Channel enum.
         * @param channel The channel to read from. Format: CHAN0, CHAN1, etc.
         */
        Max1148(Channel channel): cs_pin(ADC_CS_PIN) {
            this->channel = static_cast<uint8_t>(channel);
        }

        /**
         * @brief Initializes the ADC. Initializes SPI bus and CS pin.
         * This is only a separate function because you can't initialize the SPI bus in a constructor (for some reason).
         */
        void init();
        
};        


#endif