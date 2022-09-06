/*
* SPS30_Arduino_Library.h
* SPS30 Arduino Library
* https://github.com/uutzinger/SPS30_Arduino_Library
*
* This is a library adaptation from https://github.com/Sensirion/arduino-sps
* - SPS30 class
* - begin() with wire port
* - consistent return values and error checking
* - version checking
* - does not use alternative i2c implementation
* - combines includes and sources to single files
*
* Distributed as is, no warranty given.
* Urs Utzinger, September 2022
*/

/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SPS30_ARDUINO_LIBRARY_H__
#define __SPS30_ARDUINO_LIBRARY_H__

#define SPS30_DRV_VERSION_STR "3.1.1"

#include "Arduino.h"

// Uncomment the next #define if using an Teensy >= 3 or Teensy LC and want to use the dedicated I2C-Library for it
// Then you also have to include <i2c_t3.h> on your application instead of <Wire.h>
// #define USE_TEENSY3_I2C_LIB
#ifdef USE_TEENSY3_I2C_LIB
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#define I2C_LENGTH 32

#if defined BUFFER_LENGTH           // Arduino  & ESP8266 & Softwire
	#undef  I2C_LENGTH
	#define I2C_LENGTH  BUFFER_LENGTH
#endif

#if defined I2C_BUFFER_LENGTH       // ESP32
	#undef  I2C_LENGTH
	#define I2C_LENGTH  I2C_BUFFER_LENGTH
#endif

// Sensirion Arch Config ===========================================================

#ifdef ESP8266
#include <core_version.h>
#endif /* ESP8266 */

/**
 * If your platform does not provide the library stdlib.h you have to remove the
 * include and define NULL yourself (see below).
 */
#include <stdlib.h>

/**
 * #ifndef NULL
 * #define NULL ((void *)0)
 * #endif
 */

/**
 * If your platform does not provide the library stdint.h you have to
 * define the integral types yourself (see below).
 */
#include <stdint.h>

/**
 * Typedef section for types commonly defined in <stdint.h>
 * If your system does not provide stdint headers, please define them
 * accordingly. Please make sure to define int64_t and uint64_t.
 */
/* typedef unsigned long long int uint64_t;
 * typedef long long int int64_t;
 * typedef long int32_t;
 * typedef unsigned long uint32_t;
 * typedef short int16_t;
 * typedef unsigned short uint16_t;
 * typedef char int8_t;
 * typedef unsigned char uint8_t; */

#ifndef __cplusplus

/**
 * If your platform doesn't define the bool type we define it as int. Depending
 * on your system update the definition below.
 */
#if __STDC_VERSION__ >= 199901L
#include <stdbool.h>
#else

#ifndef bool
#define bool int
#define true 1
#define false 0
#endif /* bool */

#endif /* __STDC_VERSION__ */

#endif /* __cplusplus */

// Sensirion Common ================================================================

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32

// End Sensirion ===================================================================

// The default I2C address for the SCD30 is 0x61.
#define SPS30_I2C_ADDRESS 0x69
#define SPS30_MAX_SERIAL_LEN 32
#define SPS30_MAX_PRODUCT_LEN 8
/* 1s measurement intervals */
#define SPS30_MEASUREMENT_DURATION_USEC 1000000
/* 100ms delay after resetting the sensor */
#define SPS30_RESET_DELAY_USEC 100000
/** The fan is switched on but not running */
#define SPS30_DEVICE_STATUS_FAN_ERROR_MASK (1 << 4)
/** The laser current is out of range */
#define SPS30_DEVICE_STATUS_LASER_ERROR_MASK (1 << 5)
/** The fan speed is out of range */
#define SPS30_DEVICE_STATUS_FAN_SPEED_WARNING (1 << 21)

// Available commands
#define SPS30_CMD_START_MEASUREMENT         0x0010U
#define SPS30_CMD_START_MEASUREMENT_ARG     0x0300U
#define SPS30_CMD_STOP_MEASUREMENT          0x0104U
#define SPS30_CMD_READ_MEASUREMENT          0x0300U

#define SPS30_CMD_GET_DATA_READY            0x0202U
#define SPS30_CMD_AUTOCLEAN_INTERVAL        0x8004U
#define SPS30_CMD_GET_SERIAL                0xD033U
#define SPS30_CMD_GET_FIRMWARE_VERSION      0xD100U
#define SPS30_CMD_GET_PRODUCT_TYPE          0xD002U
#define SPS30_CMD_RESET                     0xD304U
#define SPS30_CMD_SLEEP                     0x1001U
#define SPS30_CMD_READ_DEVICE_STATUS_REG    0xD206U
#define SPS30_CMD_START_MANUAL_FAN_CLEANING 0x5607U
#define SPS30_CMD_WAKE_UP                   0x1103U

#define SPS30_CMD_DELAY_USEC                  5000
#define SPS30_CMD_DELAY_WRITE_FLASH_USEC     20000
#define SPS30_CMD_START_STOP_DELAY_USEC      20000
#define SPS30_CMD_RESET_DELAY_MSEC             100

#define SPS30_STATUS_OK                          0
#define SPS30_STATUS_SPEED_ERROR        0x00200000
#define SPS30_STATUS_LASER_ERROR        0x00000020
#define SPS30_STATUS_FAN_ERROR          0x00000010

#define SPS30_SERIAL_NUM_WORDS ((SPS30_MAX_SERIAL_LEN) / 2)
#define SPS30_PRODUCT_NUM_WORDS ((SPS30_MAX_PRODUCT_LEN) / 2)

struct sps30_measurement {
    float mc_1p0;
    float mc_2p5;
    float mc_4p0;
    float mc_10p0;
    float nc_0p5;
    float nc_1p0;
    float nc_2p5;
    float nc_4p0;
    float nc_10p0;
    float typical_particle_size;
};

struct sps30_version {
    uint8_t major;                  // Firmware major level
    uint8_t minor;                  // Firmware minor level
    uint32_t product_type;          // 0x00080000 
};

class SPS30
{
public:
	SPS30(void);

	void begin() { return begin(&Wire); }
#ifdef USE_TEENSY3_I2C_LIB
    void begin(i2c_t3 *wirePort);
#else
	void begin(TwoWire *wirePort); // By default use Wire port
#endif

    uint8_t fw_major()       { return(_version.major); }
    uint8_t fw_minor()       { return(_version.minor); }
	char*   serial_number()  { return(_serial_number); }
	char*   product_type()   { return(_product_type); }
    char*   driver_version() { return(_SPS30_DRV_VERSION_STR); }

	/**
	 * sps_get_driver_version() - Return the driver version
	 * @returns  Driver version string
	 */
	const char* sps_get_driver_version(void);

	/**
	 * sps30_probe() - check if SPS sensor is available and initialize it
	 *
	 * Note that Pin 4 must be pulled to ground for the sensor to operate in i2c
	 * mode (this driver). When left floating, the sensor operates in UART mode
	 * which is not compatible with this i2c driver.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool probe(void);

	/**
	 * sps30_start_measurement() - start measuring
	 *
	 * Once the measurement is started, measurements are retrievable once per second
	 * with sps30_read_measurement.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool start_measurement(void);

	/**
	 * sps30_stop_measurement() - stop measuring
	 *
	 * Stops measuring and puts the sensor back into idle mode.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool stop_measurement(void);

	/**
	 * sps30_read_datda_ready() - reads the current data-ready flag
	 *
	 * The data-ready flag indicates that new (not yet retrieved) measurements are
	 * available
	 *
	 * @data_ready: Memory where the data-ready flag (0|1) is stored.
	 * @returns  true on success, false otherwise
	 */
	bool read_data_ready(uint16_t* data_ready);

	/**
	 * sps30_read_measurement() - read a measurement
	 *
	 * Read the last measurement.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool read_measurement(struct sps30_measurement* measurement);

	/**
	 * sps30_get_fan_auto_cleaning_interval() - read the current(*) auto-cleaning
	 * interval
	 *
	 * Note that interval_seconds must be discarded when the return code is
	 * non-zero.
	 *
	 * (*) Note that due to a firmware bug on FW<2.2, the reported interval is only
	 * updated on sensor restart/reset. If the interval was thus updated after the
	 * last reset, the old value is still reported. Power-cycle the sensor or call
	 * sps30_reset() first if you need the latest value.
	 *
	 * @interval_seconds:   Memory where the interval in seconds is stored
	 * @returns  true on success, false otherwise
	 */
	bool get_fan_auto_cleaning_interval(uint32_t* interval_seconds);

	/**
	 * sps30_set_fan_auto_cleaning_interval() - set the current auto-cleaning
	 * interval
	 *
	 * @interval_seconds:   Value in seconds used to sets the auto-cleaning
	 *                      interval, 0 to disable auto cleaning
	 * @returns  true on success, false otherwise
	 */
	bool set_fan_auto_cleaning_interval(uint32_t interval_seconds);

	/**
	 * sps30_get_fan_auto_cleaning_interval_days() - convenience function to read
	 * the current(*) auto-cleaning interval in days
	 *
	 * note that the value is simply cut, not rounded or calculated nicely, thus
	 * using this method is not precise when used in conjunction with
	 * sps30_set_fan_auto_cleaning_interval instead of
	 * sps30_set_fan_auto_cleaning_interval_days
	 *
	 * Note that interval_days must be discarded when the return code is non-zero.
	 *
	 * (*) Note that due to a firmware bug, the reported interval is only updated on
	 * sensor restart/reset. If the interval was thus updated after the last reset,
	 * the old value is still reported. Power-cycle the sensor or call sps30_reset()
	 * first if you need the latest value.
	 *
	 * @interval_days:  Memory where the interval in days is stored
	 * @returns  true on success, false otherwise
	 */
	bool get_fan_auto_cleaning_interval_days(uint8_t* interval_days);

	/**
	 * sps30_set_fan_auto_cleaning_interval_days() - convenience function to set the
	 * current auto-cleaning interval in days
	 *
	 * @interval_days:  Value in days used to sets the auto-cleaning interval, 0 to
	 *                  disable auto cleaning
	 * @returns  true on success, false otherwise
	 */
	bool set_fan_auto_cleaning_interval_days(uint8_t interval_days);

	/**
	 * sps30_start_manual_fan_cleaning() - Immediately trigger the fan cleaning
	 *
	 * Note that this command can only be run when the sensor is in measurement
	 * mode, i.e. after sps30_start_measurement() without subsequent
	 * sps30_stop_measurement().
	 *
	 * @returns  true on success, false otherwise
	 */
	bool start_manual_fan_cleaning(void);

	/**
	 * sps30_reset() - reset the SGP30
	 *
	 * The sensor reboots to the same state as before the reset but takes a few
	 * seconds to resume measurements.
	 *
	 * The caller should wait at least SPS30_RESET_DELAY_USEC microseconds before
	 * interacting with the sensor again in order for the sensor to restart.
	 * Interactions with the sensor before this delay might fail.
	 *
	 * Note that the interface-select configuration is reinterpreted, thus Pin 4
	 * must be pulled to ground during the reset period for the sensor to remain in
	 * i2c mode.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool reset(void);

	/**
	 * sps30_sleep() - Send the (idle) sensor to sleep
	 *
	 * The sensor will reduce its power consumption to a minimum, but must be woken
	 * up again with sps30_wake_up() prior to resuming operations. It will only
	 * suceed if the sensor is idle, i.e. not currently measuring.
	 * Note that this command only works on firmware 2.0 or more recent.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool sleep(void);

	/**
	 * sps30_wake_up() - Wake up the sensor from sleep
	 *
	 * Use this command to wake up the sensor from sleep mode into idle mode.
	 * Note that this command only works on firmware 2.0 or more recent.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool wake_up(void);

	/**
	 * sps30_read_device_status_register() - Read the Device Status Register
	 *
	 * Reads the Device Status Register which reveals info, warnings and errors
	 * about the sensor's current operational state. Note that the flags are
	 * self-clearing, i.e. they reset to 0 when the condition is resolved.
	 * Note that this command only works on firmware 2.2 or more recent.
	 *
	 * @device_status_flags:    Memory where the device status flags are written
	 *                          into
	 *
	 * @returns  true on success, false otherwise
	 */
	bool read_device_status_register(uint32_t* device_status_flags);

	/**
	 * i2c_general_call_reset() - Send a general call reset.
	 *
	 * @warning This will reset all attached I2C devices on the bus which support
	 *          general call reset.
	 *
	 * @returns  true on success, false otherwise
	 */
	bool i2c_general_call_reset(void);

private:

	/**
	 * read_firmware_version - read the firmware version
	 * @major:  Memory where the firmware major version is written into
	 * @minor:  Memory where the firmware minor version is written into
	 *
	 * @returns  true on success, false otherwise
	 */
	bool read_firmware_version(sps30_version *v);

	/**
	 * check the firmware version
	 * @major:  minimum firmware major version
	 * @minor:  minimum firmware minor version
	 *
	 * @returns  true if firmware version is larger or equal than minimum
	 */
    bool fw_check(uint8_t major, uint8_t minor);

	/**
	 * read_serial() - retrieve the serial number
	 *
	 * Note that serial must be discarded when the return code is non-zero.
	 *
	 * @serial: Memory where the serial number is written into as hex string (zero
	 *          terminated). Must be at least SPS30_MAX_SERIAL_LEN long.
	 * @returns  true on success, false otherwise
	 */
	bool read_serial(char* serial);

	/**
	 * read_product_type() - retrieve the product type
	 *
	 * @prodcut: Memory where the product type is written into as hex string.
	 *           Must be at least 8 chars long.
	 * @returns  true on success, false otherwise
	 */
    bool read_product_type(char* product);

	/**
	 * bytes_to_uint16_t() - Convert an array of bytes to an uint16_t
	 *
	 * Convert an array of bytes received from the sensor in big-endian/MSB-first
	 * format to an uint16_t value in the correct system-endianness.
	 *
	 * @param bytes An array of at least two bytes (MSB first)
	 * @return      The byte array represented as uint16_t
	 */
	uint16_t bytes_to_uint16_t(const uint8_t* bytes);

	/**
	 * bytes_to_uint32_t() - Convert an array of bytes to an uint32_t
	 *
	 * Convert an array of bytes received from the sensor in big-endian/MSB-first
	 * format to an uint32_t value in the correct system-endianness.
	 *
	 * @param bytes An array of at least four bytes (MSB first)
	 * @return      The byte array represented as uint32_t
	 */
	uint32_t bytes_to_uint32_t(const uint8_t* bytes);

	/**
	 * bytes_to_float() - Convert an array of bytes to a float
	 *
	 * Convert an array of bytes received from the sensor in big-endian/MSB-first
	 * format to an float value in the correct system-endianness.
	 *
	 * @param bytes An array of at least four bytes (MSB first)
	 * @return      The byte array represented as float
	 */
	float bytes_to_float(const uint8_t* bytes);

	uint8_t generate_crc(const uint8_t* data, uint16_t count);

    /**
	 * @returns  true on success, false otherwise
	 */
	bool check_crc(const uint8_t* data, uint16_t count,
									uint8_t checksum);

	/**
	 * fill_cmd_send_buf() - create the i2c send buffer for a command and
	 *                                 a set of argument words. The output buffer
	 *                                 interleaves argument words with their
	 *                                 checksums.
	 * @buf:        The generated buffer to send over i2c. Then buffer length must
	 *              be at least SENSIRION_COMMAND_LEN + num_args *
	 *              (SENSIRION_WORD_SIZE + CRC8_LEN).
	 * @cmd:        The i2c command to send. It already includes a checksum.
	 * @args:       The arguments to the command. Can be NULL if none.
	 * @num_args:   The number of word arguments in args.
	 *
	 * @return      The number of bytes written to buf
	 */
	uint16_t fill_cmd_send_buf(uint8_t* buf, uint16_t cmd,
										const uint16_t* args, uint8_t num_args);

	/**
	 * i2c_read_words() - read data words from sensor
	 *
	 * @address:    Sensor i2c address
	 * @data_words: Allocated buffer to store the read words.
	 *              The buffer may also have been modified on STATUS_FAIL return.
	 * @num_words:  Number of data words to read (without CRC bytes)
	 *
	 * @returns  number of words read
	 */
	int8_t i2c_read_words(uint8_t address, uint16_t* data_words,
		    			  uint16_t num_words);

	/**
	 * i2c_read_words_as_bytes() - read data words as byte-stream from
	 *                                       sensor
	 *
	 * Read bytes without adjusting values to the uP's word-order.
	 *
	 * @address:    Sensor i2c address
	 * @data:       Allocated buffer to store the read bytes.
	 *              The buffer may also have been modified on STATUS_FAIL return.
	 * @num_words:  Number of data words(!) to read (without CRC bytes)
	 *              Since only word-chunks can be read from the sensor the size
	 *              is still specified in sensor-words (num_words = num_bytes *
	 *              SENSIRION_WORD_SIZE)
	 *
	 * @returns  number of words read
	 */
	int8_t i2c_read_words_as_bytes(uint8_t address, uint8_t* data,
											uint16_t num_words);

	/**
	 * i2c_write_cmd() - writes a command to the sensor
	 * @address:    Sensor i2c address
	 * @command:    Sensor command
	 *
	 * @return:  true on success, false otherwise
	 */
	bool i2c_write_cmd(uint8_t address, uint16_t command);

	/**
	 * i2c_write_cmd_with_args() - writes a command with arguments to the
	 *                                       sensor
	 * @address:    Sensor i2c address
	 * @command:    Sensor command
	 * @data:       Argument buffer with words to send
	 * @num_words:  Number of data words to send (without CRC bytes)
	 *
	 * @return:  true on success, false otherwise
	 */
	bool i2c_write_cmd_with_args(uint8_t address, uint16_t command,
											const uint16_t* data_words,
											uint16_t num_words);

	/**
	 * i2c_delayed_read_cmd() - send a command, wait for the sensor to
	 *                                    process and read data back
	 * @address:    Sensor i2c address
	 * @cmd:        Command
	 * @delay:      Time in microseconds to delay sending the read request
	 * @data_words: Allocated buffer to store the read data
	 * @num_words:  Data words to read (without CRC bytes)
	 *
	 * @return:  number of words read
	 */
	int8_t i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
										uint32_t delay_us, uint16_t* data_words,
										uint16_t num_words);
	/**
	 * i2c_read_cmd() - reads data words from the sensor after a command
	 *                            is issued
	 * @address:    Sensor i2c address
	 * @cmd:        Command
	 * @data_words: Allocated buffer to store the read data
	 * @num_words:  Data words to read (without CRC bytes)
	 *
	 * @return:  number of words read
	 */
	int8_t i2c_read_cmd(uint8_t address, uint16_t cmd,
								uint16_t* data_words, uint16_t num_words);


    // Sensirion I2C ========================================================================

	/**
	 * Execute one read transaction on the I2C bus, reading a given number of bytes.
	 * If the device does not acknowledge the read command, an error shall be
	 * returned.
	 *
	 * @param address 7-bit I2C address to read from
	 * @param data    pointer to the buffer where the data is to be stored
	 * @param count   number of bytes to read from I2C and store in the buffer
	 * @returns number of bytes read
	 */
	int8_t i2c_read(uint8_t address, uint8_t* data, uint16_t count);

	/**
	 * Execute one write transaction on the I2C bus, sending a given number of
	 * bytes. The bytes in the supplied buffer must be sent to the given address. If
	 * the slave device does not acknowledge any of the bytes, an error shall be
	 * returned.
	 *
	 * @param address 7-bit I2C address to write to
	 * @param data    pointer to the buffer containing the data to write
	 * @param count   number of bytes to read from the buffer and send over I2C
	 * @returns true on success, false otherwise
	 */
	bool i2c_write(uint8_t address, const uint8_t* data,
							uint16_t count);

	// privat variables
    char*         _SPS30_DRV_VERSION_STR = SPS30_DRV_VERSION_STR;
    char          _serial_number[SPS30_MAX_SERIAL_LEN];
    char          _product_type[SPS30_MAX_PRODUCT_LEN+1];
	uint8_t 	  _I2C_Max_bytes;
	bool          _running;
	sps30_version _version;
#ifdef USE_TEENSY3_I2C_LIB
	i2c_t3       *_i2cPort; // The generic connection to user's chosen I2C hardware
#else
	TwoWire      *_i2cPort;																		 // The generic connection to user's chosen I2C hardware
#endif

};

#endif  // End of __SPS30_ARDUINO_LIBRARY_H__ definition check