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
* - combines includes and source to single files
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

#include "SPS30_Arduino_Library.h"

SPS30::SPS30(void)
{
    // Constructor

    // I2C buffer size
    _I2C_Max_bytes = 20; 
    if (I2C_LENGTH >= 64) { _I2C_Max_bytes = 40; } // total length

    _running = false;
}

// Initialize the Wire port
#ifdef USE_TEENSY3_I2C_LIB
void SPS30::begin(i2c_t3 *wirePort)
#else
void SPS30::begin(TwoWire *wirePort)
#endif
{
    _i2cPort = wirePort; // Grab which port the user wants us to use
    _i2cPort->setClock(100000);
    _i2cPort->setClockStretchLimit(200000);
}

bool SPS30::probe(void) {
    bool success = true;
    
    // Try to wake up, but ignore failure if it is not in sleep mode
    (void)wake_up();

    success = read_serial(_serial_number);
    success = success && read_firmware_version(&_version); 
    success = success && read_product_type(_product_type);

    return ( success);
}

bool SPS30::read_firmware_version(sps30_version *v) {
    uint16_t version;
    int8_t rxWordCount;

    v->major = 0;
    v->minor = 0;
    rxWordCount = i2c_read_cmd(SPS30_I2C_ADDRESS, 
                               SPS30_CMD_GET_FIRMWARE_VERSION, 
                               &version, SENSIRION_NUM_WORDS(version));
    v->major = (version & 0xff00) >> 8;
    v->minor = (version & 0x00ff);    
    if (rxWordCount == SENSIRION_NUM_WORDS(version) ){ return true;  }
    else                                             { return false; }    
}

bool SPS30::fw_check(uint8_t major, uint8_t minor) {
    if (major > _version.major) return(false);
    if (minor > _version.minor) return(false);
    return(true);
}

bool SPS30::read_serial(char* serial_number) {
    int8_t rxWordCount;

    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_GET_SERIAL) ) {
        rxWordCount = i2c_read_words_as_bytes(
            SPS30_I2C_ADDRESS, (uint8_t*)serial_number, SPS30_SERIAL_NUM_WORDS);

        if (rxWordCount > 0 ) { return true; }
        else                  { return false; }
    } else { return false; }
}

bool SPS30::read_product_type(char *product_type) {
    int8_t rxWordCount;

    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_GET_PRODUCT_TYPE) ) {
        rxWordCount = i2c_read_words_as_bytes(SPS30_I2C_ADDRESS, (uint8_t *)product_type, SPS30_PRODUCT_NUM_WORDS);
        product_type[SPS30_MAX_PRODUCT_LEN] = '\0'; // terminate with null character
        if (rxWordCount > 0 ) { return true; }
        else                  { return false; }
    } else { return false; }
}

bool SPS30::start_measurement(void) {
    const uint16_t arg = SPS30_CMD_START_MEASUREMENT_ARG;

    if ( i2c_write_cmd_with_args(
        SPS30_I2C_ADDRESS, SPS30_CMD_START_MEASUREMENT, &arg,
        SENSIRION_NUM_WORDS(arg)) ) {
        delayMicroseconds(SPS30_CMD_START_STOP_DELAY_USEC);
        _running = true;
        return true;    
    } else { return false; }
}

bool SPS30::stop_measurement(void) {
    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_STOP_MEASUREMENT) ) {
        delayMicroseconds(SPS30_CMD_START_STOP_DELAY_USEC);
        _running = false;
        return true;
    } else { return false; }
}

 bool SPS30::read_data_ready(uint16_t* data_ready) {
    int8_t rxWordCount;
    rxWordCount = i2c_read_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_GET_DATA_READY,
                               data_ready, SENSIRION_NUM_WORDS(*data_ready));
    if (rxWordCount == SENSIRION_NUM_WORDS(*data_ready) ){ return true;  }
    else                                                 { return false; }
}

bool SPS30::read_measurement(struct sps30_measurement* measurement) {
    uint8_t data[10][4];
    int8_t rxWordCount;

    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_READ_MEASUREMENT) ) {
        rxWordCount = i2c_read_words_as_bytes(SPS30_I2C_ADDRESS, &data[0][0],
                                              SENSIRION_NUM_WORDS(data));
        
        measurement->mc_1p0  = bytes_to_float(data[0]);
        measurement->mc_2p5  = bytes_to_float(data[1]);
        measurement->mc_4p0  = bytes_to_float(data[2]);
        measurement->mc_10p0 = bytes_to_float(data[3]);
        measurement->nc_0p5  = bytes_to_float(data[4]);
        measurement->nc_1p0  = bytes_to_float(data[5]);
        measurement->nc_2p5  = bytes_to_float(data[6]);
        measurement->nc_4p0  = bytes_to_float(data[7]);
        measurement->nc_10p0  = bytes_to_float(data[8]);
        measurement->typical_particle_size = bytes_to_float(data[9]);

        if ( rxWordCount == SENSIRION_NUM_WORDS(data) ) { return true;  } 
        else                                            { return false; }
    } else { return false; }
}

bool SPS30::get_fan_auto_cleaning_interval_days(uint8_t* interval_days) {
    uint32_t interval_seconds;

    if (get_fan_auto_cleaning_interval(&interval_seconds) ) {
        *interval_days = interval_seconds / (24 * 60 * 60);
        return true;
    } else { return false; }
}

bool SPS30::get_fan_auto_cleaning_interval(uint32_t* interval_seconds) {
    uint8_t data[4];
    int8_t rxWordCount;

    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_AUTOCLEAN_INTERVAL) ) {
        delayMicroseconds(SPS30_CMD_DELAY_USEC);

        rxWordCount = i2c_read_words_as_bytes(SPS30_I2C_ADDRESS, data,
                                              SENSIRION_NUM_WORDS(data));
        *interval_seconds = bytes_to_uint32_t(data);

        if ( rxWordCount == SENSIRION_NUM_WORDS(data) ) { return true;  } 
        else                                            { return false; }
    } else { return false; }
}

bool SPS30::set_fan_auto_cleaning_interval_days(uint8_t interval_days) {
    return set_fan_auto_cleaning_interval((uint32_t)interval_days * 24 * 60 * 60);
}

bool SPS30::set_fan_auto_cleaning_interval(uint32_t interval_seconds) {
    const uint16_t data[] = {(uint16_t)((interval_seconds & 0xFFFF0000) >> 16),
                             (uint16_t)(interval_seconds & 0x0000FFFF)};

    if ( i2c_write_cmd_with_args(SPS30_I2C_ADDRESS,
                                 SPS30_CMD_AUTOCLEAN_INTERVAL, data,
                                 SENSIRION_NUM_WORDS(data)) ) {
        delayMicroseconds(SPS30_CMD_DELAY_WRITE_FLASH_USEC);
        return true;    
    } else {
        return false;
    }
}

bool SPS30::start_manual_fan_cleaning(void) {
    if ( i2c_write_cmd(SPS30_I2C_ADDRESS,
                       SPS30_CMD_START_MANUAL_FAN_CLEANING) ) {
        delayMicroseconds(SPS30_CMD_DELAY_USEC);
        return true;
    } else {
        return false;
    }
}

bool SPS30::reset(void) {
    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_RESET) ) {
        delay(SPS30_CMD_RESET_DELAY_MSEC);
        return true;
    } else {
        return false;
    }
 }

bool SPS30::sleep(void) {
    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_SLEEP) ) {
        delayMicroseconds(SPS30_CMD_DELAY_USEC);
        return true;
    } else{ 
        return false;
    }
}

bool SPS30::wake_up(void) {
    bool success;

    /* wake-up must be sent twice within 100ms, ignore first return value */
    (void) i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_WAKE_UP);
    if ( i2c_write_cmd(SPS30_I2C_ADDRESS, SPS30_CMD_WAKE_UP) ) { 
        delayMicroseconds(SPS30_CMD_DELAY_USEC);
        return true;
    } else {
        return false; 
    }
}

bool SPS30::read_device_status_register(uint32_t* device_status_flags) {
    bool success;
    int8_t rxWordCount;
    uint16_t word_buf[2];

    rxWordCount = i2c_delayed_read_cmd(
        SPS30_I2C_ADDRESS, SPS30_CMD_READ_DEVICE_STATUS_REG, SPS30_CMD_DELAY_USEC,
        word_buf, SENSIRION_NUM_WORDS(word_buf));

    *device_status_flags = (((uint32_t)word_buf[0]) << 16) | word_buf[1];
    if ( rxWordCount == SENSIRION_NUM_WORDS(word_buf) ) { return true;  }
    else                                                { return false; }
}

// Common ======================================================================

uint16_t SPS30::bytes_to_uint16_t(const uint8_t* bytes) {
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

uint32_t SPS30::bytes_to_uint32_t(const uint8_t* bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

float SPS30::bytes_to_float(const uint8_t* bytes) {
    union {
        uint32_t u32_value;
        float float32;
    } tmp;

    tmp.u32_value = bytes_to_uint32_t(bytes);
    return tmp.float32;
}

uint8_t SPS30::generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if ((crc & 0x80) !=0) { crc = (crc << 1) ^ CRC8_POLYNOMIAL; } 
            else                  { crc = (crc << 1); }
        }
    }
    return crc;
}

bool SPS30::check_crc(const uint8_t* data, uint16_t count,
                      uint8_t checksum) {
    if (generate_crc(data, count) != checksum) { return false; }
    else {                                       return true; }
}

// I2C high level ======================================================================

bool SPS30::i2c_general_call_reset(void) {
    const uint8_t data = 0x06;
    return i2c_write(0, &data, (uint16_t)sizeof(data));
}

uint16_t SPS30::fill_cmd_send_buf(uint8_t* buf, uint16_t cmd,
                                  const uint16_t* args, uint8_t num_args) {
    uint8_t crc;
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = generate_crc((uint8_t*)&buf[idx - 2],
                            SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}

bool SPS30::i2c_write_cmd(uint8_t address, uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    fill_cmd_send_buf(buf, command, NULL, 0);
    return i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
}

bool SPS30::i2c_write_cmd_with_args(uint8_t address, uint16_t command,
                                    const uint16_t* data_words,
                                    uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    uint16_t buf_size;

    buf_size = fill_cmd_send_buf(buf, command, data_words, num_words);
    return i2c_write(address, buf, buf_size);
}

int8_t SPS30::i2c_read_cmd(uint8_t address, uint16_t cmd,
                           uint16_t* data_words, uint16_t num_words) {
    return i2c_delayed_read_cmd(address, cmd, 0, data_words, num_words);
}

int8_t SPS30::i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
                                   uint32_t delay_us, uint16_t* data_words,
                                   uint16_t num_words) {

    uint8_t buf[SENSIRION_COMMAND_SIZE];

    fill_cmd_send_buf(buf, cmd, NULL, 0);
    if (i2c_write(address, buf, SENSIRION_COMMAND_SIZE) ) {
        if (delay_us > 0) { delayMicroseconds(delay_us); }
        return i2c_read_words(address, data_words, num_words);
     } else {
        return 0; 
    }
}

int8_t SPS30::i2c_read_words(uint8_t address, uint16_t* data_words, 
                           uint16_t num_words) {
    bool success;
    int8_t rxWordCount;
    uint8_t i;
    const uint8_t* word_bytes;

    rxWordCount = i2c_read_words_as_bytes(address, (uint8_t*)data_words, 
                                          num_words);

    for (i = 0; i < num_words; ++i) {
        word_bytes    = (uint8_t*)&data_words[i];
        data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
    }

    return rxWordCount;
}

int8_t SPS30::i2c_read_words_as_bytes(uint8_t address, uint8_t* data, 
                                      uint16_t num_words) {
    bool success = true;
    int8_t rxByteCount;
    uint16_t i, j;
    uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
    uint8_t* const buf8 = (uint8_t*)word_buf;

    rxByteCount = i2c_read(address, buf8, size);

    /* check the CRC for each word */
    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {
        
        if ( check_crc(&buf8[i], SENSIRION_WORD_SIZE, 
                       buf8[i + SENSIRION_WORD_SIZE]) == false ) {
            success = false; 
        } 

        data[j++] = buf8[i];
        data[j++] = buf8[i + 1];
    }

    if ( success ) { return  (rxByteCount/(SENSIRION_WORD_SIZE + CRC8_LEN)); }
    else           { return -(rxByteCount/(SENSIRION_WORD_SIZE + CRC8_LEN)); }
}

// I2C low level ======================================================================

int8_t SPS30::i2c_read(uint8_t address, uint8_t *data, uint16_t count) {
    uint8_t readData[count];
    int8_t rxByteCount = 0;

    // 2 bytes RH, 1 CRC, 2 bytes T, 1 CRC
    _i2cPort->requestFrom(address, count);

    while (_i2cPort->available()) {  // wait till all arrive
        readData[rxByteCount++] = _i2cPort->read();
        if (rxByteCount >= count) { break; }
    }

    memcpy(data, readData, rxByteCount);
 
    return rxByteCount;
}

bool SPS30::i2c_write(uint8_t address, const uint8_t *data, 
                      uint16_t count) {
    _i2cPort->beginTransmission(address);
    _i2cPort->write(data, count);
    if (_i2cPort->endTransmission() == 0) { return true; } // success
    else { return false; }  // Sensor did not ACK
}
