#ifndef PIP_CONTROLLER_HPP
#define PIP_CONTROLLER_HPP

#include <Arduino.h>
#include <Pip.hpp>

class PipController{
	private:
		Pip& pip1;
		Pip& pip2;
	public:
		PipController(Pip& pip1, Pip& pip2) : pip1(pip1), pip2(pip2) {}

		void sweep(){
			double value1 = pip1.sweep_min;
			double value2 = pip2.sweep_min;
			double step1 = (double)(pip1.sweep_max - pip1.sweep_min) / (double)(pip1.num_samples - 1);
			double step2 = (double)(pip2.sweep_max - pip2.sweep_min) / (double)(pip2.num_samples - 1);

			//set delay to max delay between the pips - should always be the same since
			//both pips are configured the same
			uint16_t delay = (pip1.delay_us < pip2.delay_us) ? pip1.delay_us : pip2.delay_us;
			for (int i = 0; i < pip1.num_samples; i++){
				analogWrite(pip1.dac_pin, (int)value1);
				analogWrite(pip2.dac_pin, (int)value2);
				value1 += step1;
				value2 += step2;
				delayMicroseconds(delay);
				pip1.data[i] = pip1.read_adc();
				pip2.data[i] = pip2.read_adc();
			}
		}
};
#endif