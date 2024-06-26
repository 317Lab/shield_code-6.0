#include <Max1148.hpp>
/** @copydoc adc_read_avg(int avg_num) */
uint16_t Max1148::adc_read_avg(int avg_num){
    //set up SPI, macros in Max1148.hpp
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
    //data from each sample
    uint16_t data = 0;
    //total data from all samples
    uint32_t total_data = 0;
    //send command to congfigure ADC channel
    csl();
    SPI.transfer(CHAN0);
    delayMicroseconds(20);
    for(int i = 0; i < avg_num; i++){
        //note - Jacob's code had a manual delay here. I am hoping it won't be necessary, because it seems like a major PIA to do on Arduino.
        delayMicroseconds(50);
        //send dummy byte to get first byte of data
        data = SPI.transfer(READ) << 8;
        //Max1148 is a 14 bit ADC - so we have to do two SPI transfers to get all the data.
        data |= SPI.transfer(READ);
        //add data to total. Shifted right because result is left justified - Jacob
        total_data += (uint32_t)data >> 2; 
        total_data += data;
    }
        csh();
        SPI.endTransaction();
    	return (uint16_t)(total_data / avg_num);
}

/** @copydoc Max1148::csl() */
void Max1148::csl(){
    digitalWrite(cs_pin, LOW);
}

/** @copydoc Max1148::csh() */
void Max1148::csh(){
    digitalWrite(cs_pin, HIGH);
}

/** @copydoc Max1148::clear() */
void Max1148::clear(){
    csl();
    SPI.transfer(0x00);
    csh();
}