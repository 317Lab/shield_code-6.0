#ifndef AT25M02_H
#define AT25M02_H

#include "Arduino.h"
#include "SPI.h"

/**
 * Provides a library to interact with Microchip's AT25M02 chip.
 * This treats the chip as a circular queue that will not overwrite it's data.
 *
 */

/*
 * Commands for the AT25M02 chip. Pulled from the data sheet for this chip.
 * All commands are MSB first.
 */
enum Command
{
	READ_STATUS = 0x05,
	LOW_PWR_WRITE_POLL = 0x05,
	WRITE_ENABLE = 0x06,
	WRITE_DISABLE = 0x04,
	WRITE_STATUS = 0x01,
	READ = 0x03,
	WRITE_BYTE = 0x02,
	WRITE_PAGE = 0x02
};
/**
 * @brief Interfaces with the AT25M02 EEPROM chip.
 */
class AT25M02
{
	public:
		AT25M02(){};
		void init();

		/*
		 * Write from the given array to the memory. This data is
		 * appended to the end of the queue.
		 * Returns true if it successfully wrote all bytes.
		 * Returns false if it could not write every byte without
		 * overwriting existing data.
		 */
		bool writeData(byte* bytes, uint32_t length);

		/*
		 * Write the given number of bytes from the ram into the
		 * destination array. These bytes are taken from the start of
		 * the queue. Returns how many bytes were read.
		 */
		int readData(byte* dest, uint32_t length);

		/*
		 * Returns how many bytes are free and available to be written
		 * to.
		 */
		uint32_t freeBytes();

		/*
		 * Returns the number of bytes currently being used.
		 */
		uint32_t usedBytes();

		/*
		 * Checks if the RAM chip is ready for a new command.
		 */
		bool isReady();

	private:
		/*
		 * Reads the status register and returns the register
		 */
		byte readStatusReg();

		uint32_t usedMemoryBytes();
		uint32_t freeMemoryBytes();
		uint32_t usedBufferBytes();
		uint32_t freeBufferBytes();

		uint32_t readWriteBuffer(byte *dest, uint32_t length);
		uint32_t readMemory(byte *dest, uint32_t length);

		/*
		 * Sets the Write Status Register
		 */
		void setWRSR(byte val);

		/*
		 * Waits until the write cycle is done.
		 */
		void waitUntilReady();

		/*
		 * Writes are page buffered.
		 * Partial page reads are supported.
		 * Start is inclusive, end is exclusive.
		 */
		uint8_t write_buffer[256];
		uint32_t wb_end;
		uint32_t mem_start;
		uint32_t mem_end;
		bool ram_full;

		SPISettings spi_settings;

		/*
		 * Just writes a page without caring about overwrite
		 */
		void writePage(uint32_t addr, byte* bytes, uint32_t length);

		/*
		 * Chips select low => enabled on this chip.
		 */
		void csl();

		/*
		 * Chip select high => disabled on this chip/
		 */
		void csh();

		/*
		 * Sends a command to the EEPROM.
		 */
		void sendCommand(Command cmd);

		// Pin out information
		int chip_select_pin;
};

#endif
