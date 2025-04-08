/**
 * @file Pip.cpp
 * @brief Source file for the Pip library for Dartmouth's 317 Lab.
 */
#include <Pip.hpp>
/** @copydoc Pip:Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_pin, Max1148& adc) */
Pip::Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_pin, Max1148& adc)
    : delay_us(delay), avg_num(avg_num), num_samples(num_samples), sweep_min(min), sweep_max(max), dac_pin(dac_pin), adc(adc){
    pinMode(dac_pin, OUTPUT);
    analogWriteResolution(12);
}

/** @copydoc Pip::clear_data(uint16_t data[], uint16_t size) */
void Pip::clear_data(uint16_t data[], uint16_t size){
    for (int i = 0; i < size; i++){
        data[i] = 0;
    }
}

uint16_t Pip::read_adc(){
    return adc.adc_read();
}