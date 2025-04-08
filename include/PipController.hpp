#ifndef PIP_CONTROLLER_HPP
#define PIP_CONTROLLER_HPP

#include <Arduino.h>
#include <Pip.hpp>
/**
 * @brief Manages simultaneous sweep for two Pip sensors.
 * A bit hacky, but allows easily managing simultaneous sweeping while keeping data separate and clean.
 */
class PipController{
	private:
		Pip& pip1;
		Pip& pip2;
	public:
		PipController(Pip& pip1, Pip& pip2) : pip1(pip1), pip2(pip2) {}
		/**
		 * @brief Sweeps both DACs simultaneously, reads both ADC channels. Note that ADC sampling alternates between each pip channel.
		 */
		void sweep(){
			double value1 = pip1.sweep_min;
			double value2 = pip2.sweep_min;
			double step1 = (double)(pip1.sweep_max - pip1.sweep_min) / (double)(pip1.num_samples - 1);
			double step2 = (double)(pip2.sweep_max - pip2.sweep_min) / (double)(pip2.num_samples - 1);

			//set delay to max delay between the pips - should always be the same since
			//both pips are configured the same
			uint16_t delay = (pip1.delay_us < pip2.delay_us) ? pip1.delay_us : pip2.delay_us;
			SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
			for (int i = 0; i < pip1.num_samples; i++){
				analogWrite(pip1.dac_pin, (int)value1);
				analogWrite(pip2.dac_pin, (int)value2);
				value1 += step1;
				value2 += step2;
				//preamp settling time - experimentally derived
				delayMicroseconds(delay);
				int total_data1 = 0;
				int total_data2 = 0;
				//avg_num should be same across both pips.
				for (int i=0; i < pip1.avg_num; i++){
					total_data1 += pip1.read_adc();
					total_data2 += pip2.read_adc();
				}
				pip1.data[i] = (uint16_t)(total_data1 / pip1.avg_num);
				pip2.data[i] = (uint16_t)(total_data2 / pip2.avg_num);
			}
			analogWrite(pip1.dac_pin, pip1.sweep_min);
			analogWrite(pip2.dac_pin, pip2.sweep_min);
			SPI.endTransaction();
		}
};
#endif