#include <Max1148.hpp>
/** @copydoc adc_read_avg(int avg_num) */
uint16_t Max1148::adc_read_avg(int avg_num){
    int total_data = 0;
    //SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
    for(int i = 0; i < avg_num; i++){
        //separate variable for debugging
        uint16_t data = adc_read();
        total_data += data;
    }
    //SPI.endTransaction();
    return (uint16_t)(total_data / avg_num);
}

//probe left side of R3 for data in
uint16_t Max1148::adc_read(){
    //set up SPI, macros in Max1148.hpp
    //SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
    //data from each sample
    uint16_t data = 0;
    //total data from all samples
    //send command to congfigure ADC channel
    csl();
    //note - manual delay removed because we are on external clock now.
    SPI.transfer(channel);
    //delayMicroseconds(50);
    //send dummy byte to get first byte of data
    data = SPI.transfer(ADC_READ) << 8;
    //Max1148 is a 14 bit ADC - so we have to do two SPI transfers to get all the data.
    data |= SPI.transfer(ADC_READ);
    //add data to total. Shifted right because result is left justified - Jacob
    csh();
    //SPI.endTransaction();
    return data;
}

/** @copydoc Max1148::csl() */
void Max1148::csl(){
    digitalWrite(cs_pin, LOW);
}

/** @copydoc Max1148::csh() */
void Max1148::csh(){
    digitalWrite(cs_pin, HIGH);
}
