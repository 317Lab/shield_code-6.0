/**
 * @file PDC.hpp
 * @brief Header file for the PDC library for Dartmouth's 317 Lab.
 * 
 * This library is used to manage UART communication on the Arduino Due using the Peripheral DMA Controller (PDC).
 * This allows for non-blocking communication, so that we can run other processes while sending out data.
 *
 * Author: Sean Wallace
 * Date: 2024-06-20
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

#define TXTEN_MASK (1<<8) //mask used to enable UART transmitter

#define UART_ID    8 // UART Peripheral ID

class PDC {
private:
    // pointers to UART registers
    volatile void* const p_UART_TPR;
    volatile uint32_t* const p_UART_TCR;
    volatile uint32_t* const p_UART_TNPR;
    volatile uint32_t* const p_UART_TNCR;
    volatile uint32_t* const p_UART_PTCR;
    volatile uint32_t* const p_UART_PTSR;

    /**
     * @brief Arduino's RingBuffer object, used for buffering data into the PDC
     * 
    */
    RingBuffer *_rx_buffer;
    RingBuffer *_tx_buffer;

    

public:
    /**
     * @brief Default constructor for the PDC class.
     * 
     * Shouldn't ever be used, but is here for completeness.
     */
    PDC()
        : p_UART_TPR((void*)UART_PERIPH_TPR_ADDR),
          p_UART_TCR((uint32_t*)UART_PERIPH_TCR_ADDR),
          p_UART_TNPR((uint32_t*)UART_PERIPH_TNPR_ADDR),
          p_UART_TNCR((uint32_t*)UART_PERIPH_TNCR_ADDR),
          p_UART_PTCR((uint32_t*)UART_PERIPH_PTCR_ADDR),
          p_UART_PTSR((uint32_t*)UART_PERIPH_PTSR_ADDR)
    {}

    /**
     * @brief Constructor for the PDC class.
     * 
     * @param *pRx_buffer Pointer to the RingBuffer object for receiving data.
     * @param *pTx_buffer Pointer to the RingBuffer object for transmitting data.
     */
    PDC(RingBuffer *pRx_buffer, RingBuffer *pTx_buffer)
        : p_UART_TPR((void*)UART_PERIPH_TPR_ADDR),
          p_UART_TCR((uint32_t*)UART_PERIPH_TCR_ADDR),
          p_UART_TNPR((uint32_t*)UART_PERIPH_TNPR_ADDR),
          p_UART_TNCR((uint32_t*)UART_PERIPH_TNCR_ADDR),
          p_UART_PTCR((uint32_t*)UART_PERIPH_PTCR_ADDR),
          p_UART_PTSR((uint32_t*)UART_PERIPH_PTSR_ADDR)
    {
        _rx_buffer = pRx_buffer;
        _tx_buffer = pTx_buffer;
    }

    /**
     * @brief Initializes the PDC.
     * 
     * Sets up the PDC for UART communication. Enables transmit register, sets up ring buffer, and initializes data buffer.
     */
    void init();

     
    
    /**
     * @brief Writes a single byte to the PDC buffer.
     * @param uc_data Byte to write to the buffer. 
     */

    size_t write(const int uc_data);
    
    /**
     * @brief Flushes the PDC buffer.
     * 
     * Used to send any remaining data in the buffer.
     * When sending larger array, most data usually goes to buffer before PDC is available to send it. So, have to flush before moving on.
     */

    void flush();

    

    void tester(int16_t* arr, int size);

    template <typename T>
    void send(T const *arr, int size){
        for(int i=0; i<size; i++){
        const uint8_t* bytePtr = reinterpret_cast<const uint8_t*>(&arr[i]);
            for(size_t j=0; j<sizeof(arr[i]); j++){
                write(bytePtr[j]);
            }
        }
        flush();
    }
};

#endif

/* ARCHIVED CODE:
    //tried to send all the data through the vector thing
    void send(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, uint8_t buffer_data, 
    uint32_t buffer_timestamp);



    void process_data(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, 
    uint8_t buffer_data, uint32_t buffer_timestamp);
    

    template <typename T>
    uint8_t convert_data(const T& value);


    uint8_t convert_array(uint16_t* arr, int size);

    uint8_t convert_imu_array(int16_t* arr, int size);


    int8_t serialized_storage[100];


    Vector<int8_t> serialized_data;
        size_t write_array(uint8_t const *uc_data, int size);

    void write_vec(Vector<int8_t> data);

    
*/