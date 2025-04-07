/**
 * @file Pip.cpp
 * @brief Source file for the Pip library for Dartmouth's 317 Lab.
 */
/*Check how linear DAC output is

memory chips meant to be handed a page at a time, not bit by bit. that was the on board memory we had access to. 
That's what explains the weird page framing things. there may now be better ways to address now.
should be able to use the 422 to program the arduino. we can't understand why not. Worried that we'll have to bring some of the
usb aspect up, because need to program it.

Note - step time is 565 us.

*/
#include <Pip.hpp>
/** @copydoc Pip:Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_pin, Max1148& adc) */
Pip::Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_pin, Max1148& adc)
    : delay_us(delay), avg_num(avg_num), num_samples(num_samples), sweep_min(min), sweep_max(max), dac_pin(dac_pin), adc(adc){
    pinMode(dac_pin, OUTPUT);
    analogWriteResolution(12);
}


/** @copydoc Pip::sweep() */
void Pip::sweep(){
    //clear data from last sweep. Shouldn't cause issues with sending since sweep won't begin until data is cleaned and loaded into PDC buffer.
    //Leaving commented because it shouldn't actually be necessary and impacts performance.
    //clear_data(data, sizeof(data) / sizeof(data[0]));

    double value = sweep_min;
	double step = (double)(sweep_max - sweep_min) / (double)(num_samples - 1);
    for (int i = 0; i < num_samples; i++){
        analogWrite(DAC_PIN, (int)value);
        value += step;
        delayMicroseconds(delay_us);
        data[i] = adc.adc_read_avg(avg_num);
    }
    analogWrite(DAC_PIN, sweep_max);
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