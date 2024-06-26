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
#include <Vector.h>

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

    /**
     * @brief Processes all data into a single buffer of bytes.
     * 
     * This allows us to use the character sentinels from the old version of this code, sending data all at once.
     * Note that this is set up for little endian format.
     * 
     * Takes 8<=t<=16 us to process data.
     */
    void process_data(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, 
    uint8_t buffer_data, uint32_t buffer_timestamp);
    
   /**
     * @brief Converts data into bytes, and stores it in the serialized_data buffer.
     * 
     * Used inside process_data(). Removes empty bytes.
     * Edge cases tested: 0, 0x1234, 0xFFFFFFFF, 0x00FF00FF
     */
    template <typename T>
    uint8_t convert_data(const T& value);

    /** @brief Converts integer array into bytes, stores it in the serialized_data buffer
     * 
     * Only used for sweep data.
    */
    uint8_t convert_array(uint16_t* arr, int size);

    /**
     * @brief sets the maximum size of the serialized data
     */
    uint8_t serialized_storage[100];
    /**
     * @brief buffer that holds the shield data. 
     * 
     * Note, this isn't std::vector, because that doesn't really work on the Arduino
     * Can see how Vector is initialized in PDC::init()
     */
    Vector<uint8_t> serialized_data;

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
     * @brief Sends shield data over UART using the PDC.
     */

    void send(uint16_t imu_data, uint32_t imu_timestamp, uint16_t* sweep_data, int sweep_size, uint32_t sweep_timestamp, uint8_t buffer_data, 
    uint32_t buffer_timestamp);
    /**
     * @brief Writes a single byte to the PDC buffer.
     * @param uc_data Byte to write to the buffer. Should probably change to uint8_t.
     * Used inside send()
     */

    size_t write(const int uc_data);
    /**
     * @brief Writes an array of bytes to the PDC buffer.
     * 
     * @param uc_data Array of bytes to write to the buffer.
     * @param size Size of the array.
     * @return 1 if successful.
     * 
     * Deprecated, but may be useful for debugging.
     */
    size_t write_array(uint8_t const *uc_data, int size);
    /**
     * @brief Flushes the PDC buffer.
     * 
     * Used to send any remaining data in the buffer.
     * When sending larger array, most data usually goes to buffer before PDC is available to send it. So, have to flush before moving on.
     */

    void flush();
    /**
     * @brief Writes a Vector of bytes to the PDC buffer.
     * @param data Vector of bytes to write to the buffer.
     * Note, not std::vector. This is a custom Vector class for the Arduino.
     * 
     * Used inside send()
     * 
     */
    void write_vec(Vector<uint8_t> data);
};

#endif