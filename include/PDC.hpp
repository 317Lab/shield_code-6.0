/**
 * @file PDC.hpp
 * @brief Header file for the PDC library for Dartmouth's 317 Lab.
 * 
 * This library is used to manage UART communication on the Arduino Due using the Peripheral DMA Controller (PDC).
 * This allows for non-blocking communication, so that we can run other processes while sending out data.
 * Note for future developers - any data array sent through this library must be a global variable. If the variable goes out of scope before the transfer is 
 * complete, the data will be lost.
 * Author: Sean Wallace
 * Date: 2024-07-08
 */


#ifndef PDC_HPP
#define PDC_HPP
#include <Arduino.h>

// Defining relevant UART registers
#define UART_BASE 0x400E0800
#define UART_PERIPH_TPR_ADDR (UART_BASE + 0x108) // Transmit pointer register
#define UART_PERIPH_TCR_ADDR (UART_BASE + 0x10C) // Transmit counter register
#define UART_PERIPH_TNPR_ADDR (UART_BASE + 0x118) // Transmit next pointer register
#define UART_PERIPH_TNCR_ADDR (UART_BASE + 0x11C) // Transmit next counter register
#define UART_PERIPH_PTCR_ADDR (UART_BASE + 0x120) // Peripheral transfer control register
#define UART_PERIPH_PTSR_ADDR (UART_BASE + 0x124) // Peripheral transfer status register
#define UART_SR (UART_BASE + 0x0014) // Status register

#define TXTEN (1<<8) //mask used to enable UART transmitter
#define TXBUFE (1<<11) //check if UART is ready
/**
 * @brief Manages UART transmits through peripheral DMA controller (PDC).
 */
class PDC {
private:
    // pointers to UART registers
    volatile uint32_t* const p_UART_TPR;
    volatile uint32_t* const p_UART_TCR;
    volatile uint32_t* const p_UART_TNPR;
    volatile uint32_t* const p_UART_TNCR;
    volatile uint32_t* const p_UART_PTCR;
    volatile uint32_t* const p_UART_PTSR;
    volatile uint32_t* const p_UART_SR;
    


    

public:
    /**
     * @brief Default constructor for the PDC class. Initializes all relevant register pointers
     */
    PDC()
        : p_UART_TPR((uint32_t*)UART_PERIPH_TPR_ADDR),
          p_UART_TCR((uint32_t*)UART_PERIPH_TCR_ADDR),
          p_UART_TNPR((uint32_t*)UART_PERIPH_TNPR_ADDR),
          p_UART_TNCR((uint32_t*)UART_PERIPH_TNCR_ADDR),
          p_UART_PTCR((uint32_t*)UART_PERIPH_PTCR_ADDR),
          p_UART_PTSR((uint32_t*)UART_PERIPH_PTSR_ADDR),
          p_UART_SR((uint32_t*)UART_SR)
    {}


    /**
     * @brief Turns on PDC, waits until ready. Must be called AFTER Serial.begin() in setup().
     */
    void init(){
    //enable pdc transmitter
    *p_UART_PTCR |= TXTEN;
    //wait until ready
    while(! PDC::is_on()){
        ;
    }

    }
    /**
     * @brief Adds data to the PDC buffer to be sent out through UART.
     * 
     * 
     * @tparam T - data type of buffer - either char pointer or integer array
     * @param buffer - pointer to data buffer. Note that it doesn't like pointer casts, so global pointers to sentinels must be used.
     * That's not an issue with Arduino's Serial library because it's blocking - avoiding that is the whole point of this.
     * @param size - size of buffer
     * 
     * This can also handle text in an ASCII readable format by using a char pointer:
     * @code
     * String message = "Hello World!";
     * uint8_t* p_message = (uint8_t*)message.c_str();
     * pdc.send(p_message, sizeof(message));
     * @endcode
     * But, Serial.print() is recommended for any debugging application to avoid global declarations.
     * 
     * Note for later - negative numbers are sent as two's complement. make sure Jules' parser is reading that correctly.
     */
    template <typename T>
    void send(T* buffer, int size){
        //check if UART is ready for transmit
        if(*p_UART_SR & TXBUFE){
                //set buffer and size
                *(volatile uint32_t*)p_UART_TPR = (uint32_t)buffer;
                *p_UART_TCR = size;
           
        } else{
            //wait until ready
            while(!(*p_UART_SR & TXBUFE)){
                ;
            }
            //same as above
                *(volatile uint32_t*)p_UART_TPR = (uint32_t)buffer;
                *p_UART_TCR = size;
        }
        
    }
    /**
     * @brief Adds data to the backup PDC buffer. Identical to send().
     */
    template <typename T>
    void send_next(T* buffer, int size){
                //set buffer and size
                if(*p_UART_SR & TXBUFE){
                    *(volatile uint32_t*)p_UART_TNPR = (uint32_t)buffer;
                    *p_UART_TNCR = size;
                }
           
            //same as above
            else{
                while(!(*p_UART_SR & TXBUFE)){
                    ;
                }
                *(volatile uint32_t*)p_UART_TNPR = (uint32_t)buffer;
                *p_UART_TNCR = size;
            }
        
    }
    /**
     * @brief Checks if the PDC and UART are on.
     */
    bool is_on(){
        return (*p_UART_PTSR & (1<<8));
    }
    

};

#endif