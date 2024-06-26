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
/**
 * @copydoc Pip::Pip()
 
 */
Pip::Pip(){
    delay_us = SWEEP_DEFAULT_DELAY;
    avg_num = SWEEP_DEFAULT_AVG_NUM;
    num_samples = SWEEP_DEFAULT_SAMPLES;
    min = SWEEP_DEFAULT_MIN;
    max = SWEEP_DEFAULT_MAX;
}
/** @copydoc Pip:Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max, uint8_t dac_channel) */
Pip::Pip(int delay, uint16_t avg_num, uint16_t num_samples, uint16_t min, uint16_t max){
    this->delay_us = delay;
    this->avg_num = avg_num;
    this->num_samples = num_samples;
    this->min = min;
    this->max = max;
    pinMode(DAC_PIN, OUTPUT);
    analogWriteResolution(12);
}
/** @copydoc Pip::sweep() */
void Pip::sweep(Max1148& adc){
    //clear data from last sweep. Shouldn't cause issues with sending since sweep won't begin until data is cleaned and loaded into PDC buffer.
    //Leaving commented because it shouldn't actually be necessary and impacts performance.
    //clear_data(data, sizeof(data) / sizeof(data[0]));

    double value = min;
	double step = (double)(max - min) / (double)(num_samples - 1);
    for (int i = 0; i < num_samples; i++){
        analogWrite(DAC_PIN, (int)value);
        value += step;
        delayMicroseconds(delay_us);
        data[i] = adc.adc_read_avg(avg_num);
    }
}
/** @copydoc Pip::clear_data(uint16_t data[], uint16_t size) */
void Pip::clear_data(uint16_t data[], uint16_t size){
    for (int i = 0; i < size; i++){
        data[i] = 0;
    }
}

void Pip::tester(Max1148& adc){
    for (int i = 0; i < 4096; i += 256) {
    analogWrite(DAC_PIN, i);
    delay(100); // Small delay to ensure the DAC has settled

    // Read the value from the ADC
    int adcValue = adc.adc_read_avg(8); // Replace with your library's method

    // Print the DAC and ADC values
    Serial.print("DAC Value: ");
    Serial.print(i);
    Serial.print(" | ADC Value: ");
    Serial.println(adcValue);

    delay(500); // Delay before the next iteration
  }

    
}