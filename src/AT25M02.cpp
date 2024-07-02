/**
 * @file AT25M02.cpp
 *
 * @brief This control's microchip's AT25M02 EEPROM Device.
 * This treats it as a circular queue, and will not overwrite existing data.
 * 
 * I (Sean) had to move initialization code out of the constructor because the compiler does not like initializing SPI in the constructor
 * It seems like most of this is written for the old version of the Arduino SPI library - probably a good idea too go through and fix that
 * at some point.
 */

#include <Arduino.h>
#include <SPI.h>
#include <AT25M02.hpp>

// Define chip select. MO,MI,SLK all are default values.
#define CHIP_SELECT_PIN 8

// Arduino to RAM data rate. In Hz.
#define SPI_DATA_RATE 5000000

// Define RAM parameters
#define PAGE_LEN 256
const uint32_t RAM_SIZE = (1L << 18);
#define NUM_PAGES (RAM_SIZE / PAGE_LEN)

/* // Constructor

AT25M02::AT25M02()
{
	
} */

// Public Methods

/**
 * @brief Initialize the AT25M02 EEPROM device.
 */
void AT25M02::init(){
	SPI.begin();
	// Set up SPI device settings
	spi_settings = SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE0);
	// Set pin out information. Could be passed in via constructor params.
	chip_select_pin = CHIP_SELECT_PIN;
	pinMode(chip_select_pin, OUTPUT);
	mem_start = 0;
	mem_end = 0;
	ram_full = false;
	wb_end = 0;
	//pretty sure this is deprecated. it breaks everything.	
	//setWRSR(0x00);
}

/*
 * Returns how many bytes are free and available to be written to.
 */
uint32_t AT25M02::freeBytes()
{
	return RAM_SIZE - usedBytes();
}

/*
 * Returns the number of bytes currently being used.
 */
uint32_t AT25M02::usedBytes()
{
	return usedMemoryBytes() + usedBufferBytes();
}

uint32_t AT25M02::usedMemoryBytes()
{
	if (mem_end >= mem_start) {
		return mem_end - mem_start;
	} else {
		return (RAM_SIZE - mem_start) + mem_end;
	}
}

uint32_t AT25M02::freeMemoryBytes()
{
	return RAM_SIZE - usedMemoryBytes();
}

uint32_t AT25M02::usedBufferBytes()
{
	return wb_end;
}

uint32_t AT25M02::freeBufferBytes()
{
	return PAGE_LEN - wb_end;
}


/*
 * Write from the given array to the memory. This data is appended to the end of
 * the queue. Returns true if it successfully wrote all bytes. Returns false if
 * it could not write every byte without overwriting existing data.
 */
bool AT25M02::writeData(byte* bytes, uint32_t length)
{
	// Bail out if we don't have enough space
	if (length > freeBytes() || ram_full) {
		return false;
	} else if (length == freeBytes()) {
		ram_full = true;
	}
	// Loop over in the input and move it first to the buffer, and then to
	// the RAM chip when the write buffer is full.
	uint32_t buf_len;
	for(;;) {
		// Move to bytes to write buffer.
		buf_len = min(length, freeBufferBytes());
		memcpy(write_buffer + wb_end, bytes, buf_len);
		wb_end += buf_len;
		bytes  += buf_len;
		length -= buf_len;
		// Break when write buffer is not full.
		// Breaking means the input buffer is exhausted.
		if (wb_end != PAGE_LEN) break;
		// Write to page.
		// Will be slow when writing multiple pages at once.
		waitUntilReady();
		writePage(mem_end, write_buffer, PAGE_LEN);
		wb_end = 0;
	}
	return true;
}

uint32_t AT25M02::readWriteBuffer(byte *dest, uint32_t length)
{
	uint32_t len = min(length, usedBufferBytes());
	if (len == 0)
		return len;
	memcpy(dest, write_buffer, len);
	memmove(write_buffer, write_buffer + len, usedBufferBytes() - len);
	wb_end -= len;
	return len;
}

uint32_t AT25M02::readMemory(byte *dest, uint32_t length)
{
	uint32_t len = min(length, usedMemoryBytes());
	if (len == 0)
		return len;
	waitUntilReady();

	// Have to pull out each byte to give to the RAM one at a time
	byte addr_byte2 = (byte) ((mem_start >> 16) & 0xFF);
	byte addr_byte1 = (byte) ((mem_start >> 8)  & 0xFF);
	byte addr_byte0 = (byte) (mem_start         & 0xFF);

	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(READ);
	SPI.transfer(addr_byte2);
	SPI.transfer(addr_byte1);
	SPI.transfer(addr_byte0);
	SPI.transfer(dest, len);
	csh();
	SPI.endTransaction();
	mem_start += len;
	ram_full = false;
	return len;
}

/*
 * Write the given number of bytes from the ram into the destination array.
 * These bytes are taken from the start of the queue.
 * Returns how many bytes were read and written to the array.
 */
int AT25M02::readData(byte* dest, uint32_t length)
{
	uint32_t memlen = readMemory(dest, length);
	uint32_t buflen = 0;
	if (memlen < length) {
		dest += memlen;
		length -= memlen;
		buflen = readWriteBuffer(dest, length);
	}
	return memlen + buflen;
}

/*
 * Checks if the RAM is ready for a new command.
 */
bool AT25M02::isReady()
{
	bool ready;
	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(READ_STATUS);
	// Bit 0 of READ_STATUS response is 0 when the device is ready
	ready = (SPI.transfer(0) & 0x01) == 0;
	csh();
	SPI.endTransaction();
	return ready;
}

// Private Methods

/*
 * Write the given data to the RAM in a page write.
 * Increments mem_end when done.
 */
void AT25M02::writePage(uint32_t addr, byte* bytes, uint32_t length)
{
	// Have to pull out each byte to give to the RAM one at a time
	byte addr_byte2 = (byte) ((addr >> 16) & 0xFF);
	byte addr_byte1 = (byte) ((addr >> 8)  & 0xFF);
	byte addr_byte0 = (byte) (addr         & 0xFF);
	// Enable writing
	sendCommand(WRITE_ENABLE);
	// Write everything over SPI
	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(WRITE_PAGE);
	SPI.transfer(addr_byte2);
	SPI.transfer(addr_byte1);
	SPI.transfer(addr_byte0);
	SPI.transfer(bytes, length);
	csh();
	SPI.endTransaction();
	mem_end += min(PAGE_LEN, length) % RAM_SIZE;
}


/*
 * Set chip select low
 */
void AT25M02::csl()
{
	digitalWrite(CHIP_SELECT_PIN, LOW);
}

/*
 * Set chip select high
 */
void AT25M02::csh()
{
	digitalWrite(CHIP_SELECT_PIN, HIGH);
}


byte AT25M02::readStatusReg()
{
	byte ret;
	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(READ_STATUS);
	ret = SPI.transfer(0);
	csh();
	SPI.endTransaction();
	return ret;
}

/*
 * Sends the specified command to the chip.
 */
void AT25M02::sendCommand(Command cmd)
{
	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(cmd);
	csh();
	SPI.endTransaction();
}

/*
 * Sets the WRSR (Write status reg)
 */
void AT25M02::setWRSR(byte val)
{
	sendCommand(WRITE_ENABLE);
	waitUntilReady();
	SPI.beginTransaction(spi_settings);
	csl();
	SPI.transfer(0x01);
	SPI.transfer(val);
	csh();
	SPI.endTransaction();
}

/*
 * Waits until the RAM is ready to write.
 */
void AT25M02::waitUntilReady()
{
	// TODO: Guard timing w/ DEBUG macro
	// int startt = micros();
	SPI.beginTransaction(spi_settings);
	csl();
	do {
		SPI.transfer(READ_STATUS);
	} while ((SPI.transfer(0) & 0x01) != 0);
	csh();
	SPI.endTransaction();
	// int endt = micros();
	// char buf[100];
	// sprintf(buf, "\nwait until ready blocked time %d\n", endt - startt);
	// Serial.write(buf);
}
