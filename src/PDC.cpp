/**
 * @file PDC.cpp
 * @brief Source file for the PDC library for Dartmouth's 317 Lab.
 */
#include <Arduino.h>
#include <PDC.hpp>
#include <Vector.h>
#include <type_traits>
/** @copydoc PDC::init() */
void PDC::init(){
    //note - UART initialization happens through Serial.begin() in the main.cpp file. Line 55 in UARTClass.cpp MUST be commented out.
    //enable pdc transmitter
    *p_UART_PTCR |= TXTEN_MASK;
    //initialize ring bffer
    _rx_buffer->_iHead = _rx_buffer->_iTail = 0;
    _tx_buffer->_iHead = _tx_buffer->_iTail = 0;
    //initialize bob data buffer
    serialized_data.setStorage(serialized_storage, sizeof(serialized_storage));
    

}



/** @copydoc PDC::write() */
size_t PDC::write(int const uc_data) {
    uint32_t* int_ptr=(uint32_t*)p_UART_TPR;
    // Check if PDC is currently transmitting
    if (*int_ptr != 0 || *p_UART_TNCR != 0) {
        // If PDC is busy, we buffer the data
        int nextWrite = (_tx_buffer->_iHead + 1) % SERIAL_BUFFER_SIZE;
        while (_tx_buffer->_iTail == nextWrite)
            ; // Spin lock if we're about to overwrite the buffer. This continues once the data is sent

        _tx_buffer->_aucBuffer[_tx_buffer->_iHead] = uc_data;
        _tx_buffer->_iHead = nextWrite;
    } else {
        // If PDC is not busy, we send the data directly
        static uint8_t dataToSend;
        dataToSend = uc_data;
        *int_ptr = (uint32_t)&dataToSend;
        *p_UART_TCR = 1;
        
    }

    // Check if there's data in the buffer and PDC is not busy
    if (*int_ptr == 0 && _tx_buffer->_iTail != _tx_buffer->_iHead) {
        // Calculate the number of bytes to send
        size_t size;
        if (_tx_buffer->_iHead > _tx_buffer->_iTail) {
            size = _tx_buffer->_iHead - _tx_buffer->_iTail;
        } else {
            size = SERIAL_BUFFER_SIZE - _tx_buffer->_iTail;
        }


        // Send buffer data through PDC
        *int_ptr = (uint32_t)&_tx_buffer->_aucBuffer[_tx_buffer->_iTail];
        *p_UART_TCR = size;

        // Update the tail position
        _tx_buffer->_iTail = (_tx_buffer->_iTail + size) % SERIAL_BUFFER_SIZE;
    }

    return 1;
}


/** @copydoc PDC::flush() */
void PDC::flush(){
    //I don't remember why I made this pointer but it works.
    int* int_ptr = (int*)p_UART_TPR;
    if(*p_UART_TCR!=0){
        while(*p_UART_TCR!=0){
            //wait for the PDC to finish transmitting
        }
    }
    if (*p_UART_TCR == 0 && _tx_buffer->_iTail != _tx_buffer->_iHead) {
        // Calculate the number of bytes to send
        size_t size;
        if (_tx_buffer->_iHead > _tx_buffer->_iTail) {
            size = _tx_buffer->_iHead - _tx_buffer->_iTail;
        } else {
            size = SERIAL_BUFFER_SIZE - _tx_buffer->_iTail;
        }


        // Set up the PDC to send the buffer data
        *int_ptr = (uint32_t)&_tx_buffer->_aucBuffer[_tx_buffer->_iTail];
        *p_UART_TCR = size;

        // Update the tail position
        _tx_buffer->_iTail = (_tx_buffer->_iTail + size) % SERIAL_BUFFER_SIZE;
    }

}

/** @copydoc PDC::convert_data() */
template <typename T>
uint8_t PDC::convert_data(const T& value) {
    //check if the template parameter is an integer type
    //static_assert(std::is_integral<T>::value, "Template parameter must be an integer type");
    //create pointer to the first byte of the passed value
    const uint8_t* bytePtr = reinterpret_cast<const uint8_t*>(&value);
    //counter mainly for debugging
    uint8_t count=0;
    //iterate through bytes, add to data buffer.
    //This works because the index here allows us to iterate across memory addresses byte by byte
    for(size_t i=0; i<sizeof(T); i++){
        if(bytePtr[i] != 0){
            serialized_data.push_back(bytePtr[i]);
        }
    }
    return count;
}

uint8_t PDC::convert_array(uint16_t* arr, int size) {
    //check if the template parameter is an integer type
    //static_assert(std::is_integral<T>::value, "Template parameter must be an integer type");
    //counter mainly for debugging
    uint8_t count=0;
    for (int j = 0; j<size; j++){
        const uint8_t* bytePtr = reinterpret_cast<const uint8_t*>(&arr[j]);

        for(size_t i=0; i<sizeof(arr[j]); i++){
            if(bytePtr[i] != 0){
                serialized_data.push_back(bytePtr[i]);
            }
        }

    }
    return count;
}

/** @copydoc PDC::process_data() */
void PDC::process_data(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, uint8_t buffer_data, uint32_t buffer_timestamp){
    //clear old buffer
    serialized_data.clear();
    //put all the data into the buffer as bytes
    convert_data(imu_data);
    convert_data(imu_timestamp);
    convert_array(sweep_data, sweep_size);
    convert_data(sweep_timestamp);
    convert_data(buffer_data);
    convert_data(buffer_timestamp);
}

/** @copydoc PDC::write_vec() */
void PDC::write_vec(Vector<uint8_t> data){
    //write all the data in the vector
    for(int i=0; i<data.size(); i++){
        write(data[i]);
    }
    flush();
}

/** @copydoc PDC::send() */
void PDC::send(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, uint8_t buffer_data, 
uint32_t buffer_timestamp){
    //put all the data into the buffer
    process_data(imu_data, imu_timestamp, sweep_data, sweep_size, sweep_timestamp, buffer_data, buffer_timestamp);
    //write the buffer to the PDC
    write_vec(serialized_data);

}

//DEPRECATED FUNCTIONS
/*
size_t PDC::write_array(uint8_t const *uc_data, int size){
    for(int i=0; i<size; i++){
        write(uc_data[i]);
    }
    flush();
    return 1;
}*/
