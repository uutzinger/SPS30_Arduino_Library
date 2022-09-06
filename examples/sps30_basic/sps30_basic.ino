//////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPS30 Example Program                                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Supported Hardware:
//  - ESP8266 micro controller.                     https://github.com/esp8266/Arduino
//  - SPS30 Sensirion particle sensor,              https://github.com/uutzinger/SCD30_Arduino_Library
//
// Wire/I2C:
//  Multiple i2c pins are supported, ESP8266 Arduino supports only one active wire interface, however the pins can
//  be specified prior to each communication. The i2c clock can also be adapted prior to each communcation.
//  At startup, all pin combinations are scanned for possible scl & sda connections of the attached sensors.
//  Supported sensors are searched on all possible hardware configurations and registered. 
//  An option to provide a delay after switching the wire port is provided.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// This software is provided as is, no warranty on its proper operation is implied. Use it at your own risk.
// Urs Utzinger
// 2022 August: Initial SPS30 driver release
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
TwoWire myWire = TwoWire(); //  inintialize the two wire system.

#include "SPS30_Arduino_Library.h"

#define sps30_i2cClockSpeed        100000                  // slow speed
#define sps30_i2cClockStretchLimit 200000                  // long clock stretch
#define yieldI2C()                 delayMicroseconds(100); // block system for 0.1 ms when switching I2C port

unsigned long currentTime;                                 // loop start time
unsigned long intervalSPS30;                               // measurement interval
unsigned long lastSPS30;                                   // last time we interacted with sensor

uint16_t      data_ready;                                  // sps30 data ready register (16bit)
uint32_t      st;                                          // sps30 status register (32bit)
char          tmpStr[256];                                 // Buffer for formatting text that will be sent to USB serial or telnet
uint32_t      autoCleanIntervalSPS30;                      // seconds between autocleaning

int sdaPin = D4;
int sclPin = D3;

const char waitmsg[] PROGMEM = {"Waiting 10 seconds, skip by hitting enter"};  // Allows user to open serial terminal to observe the debug output before the loop starts

SPS30 sps30;                                               // the particle sensor
struct sps30_measurement valSPS30;                         // the sensor values

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

  Serial.begin(115200);
  serialTrigger(waitmsg, 10000);  
  Serial.println("SPS30 Example Program");

  myWire.begin(sdaPin, sclPin);
  myWire.setClock(sps30_i2cClockSpeed);
  myWire.setClockStretchLimit(sps30_i2cClockStretchLimit);

  myWire.beginTransmission(0x69);
  if (myWire.endTransmission() == 0) { Serial.println(F("SPS30: found.")); }
  else                               { Serial.println(F("SPS30: not found.")); }


  sps30.begin(&myWire);

  // propbe
  if ( sps30.probe() )               { Serial.println(F("SPS30: probed."));      
                                       Serial.print(F("SPS30: serial: "));
                                       Serial.println(sps30.serial_number());
                                       Serial.print(F("SPS30: firmware: "));
                                       Serial.print(sps30.fw_major());
                                       Serial.print(F("."));
                                       Serial.println(sps30.fw_minor());
                                       Serial.print(F("SPS30: product type: "));
                                       Serial.println(sps30.product_type());
                                       Serial.print(F("SPS30: driver: "));
                                       Serial.println(sps30.driver_version());
  } else                             { Serial.println(F("SPS30: error probe.")); } 

  // reset
  if (sps30.reset() )                { Serial.println(F("SPS30: reset."));  }
  else                               { Serial.println(F("SPS30: error reset."));} 

  // get cleaning interval
  if (sps30.get_fan_auto_cleaning_interval(&autoCleanIntervalSPS30) ) {
                                       Serial.print(F("SPS30: current auto clean interval: "));
                                       Serial.println(autoCleanIntervalSPS30); 
  } else                             { Serial.println(F("SPS30: coulnd not obtain autoclean information."));  }
 
  // Start the device, will create internal readings every 1 sec
  if ( sps30.start_measurement() )   { Serial.println(F("SPS30: measurement started.")); } 
  else                               { Serial.println(F("SPS30: could not start measurement.")); }

  lastSPS30 = millis();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  currentTime = millis();
  
  if ((currentTime - lastSPS30) > 1000) {
    if (sps30.read_data_ready(&data_ready) ) { 
                                      Serial.println(F("SPS30: checked for data ready.")); 
    } else {                          Serial.println(F("SPS30: could not check for data ready.")); }
    if (data_ready) { 
      // read data
      if (sps30.read_measurement(&valSPS30) ) {  
        lastSPS30 = currentTime;
        Serial.print("PM  1.0: ");
        Serial.println(valSPS30.mc_1p0);
        Serial.print("PM  2.5: ");
        Serial.println(valSPS30.mc_2p5);
        Serial.print("PM  4.0: ");
        Serial.println(valSPS30.mc_4p0);
        Serial.print("PM 10.0: ");
        Serial.println(valSPS30.mc_10p0);
        Serial.print("NC  0.5: ");
        Serial.println(valSPS30.nc_0p5);
        Serial.print("NC  1.0: ");
        Serial.println(valSPS30.nc_1p0);
        Serial.print("NC  2.5: ");
        Serial.println(valSPS30.nc_2p5);
        Serial.print("NC  4.0: ");
        Serial.println(valSPS30.nc_4p0);
        Serial.print("NC 10.0: ");
        Serial.println(valSPS30.nc_10p0);
        Serial.print("Typical partical size: ");
        Serial.println(valSPS30.typical_particle_size);
        Serial.println();  
      }

      // obtain device status
      if ( ( (sps30.fw_major()==2) && (sps30.fw_minor()>=2) ) || (sps30.fw_major()>2) ) {
        if (sps30.read_device_status_register(&st) ) { 
          Serial.println(F("SPS30: read status."));
          if (st == SPS30_STATUS_OK)             { Serial.println(F("SPS30: ok.")); }
          else {
              if (st & SPS30_STATUS_SPEED_ERROR) { Serial.println(F("SPS30: warning, fan is turning too fast or too slow."));  }
              if (st & SPS30_STATUS_LASER_ERROR) { Serial.println(F("SPS30: error laser failure.")); }
              if (st & SPS30_STATUS_FAN_ERROR)   { Serial.println(F("SPS30: error fan failure, fan is mechanically blocked or broken.")); }
          } // status ok/notok
        } else                                   { Serial.println(F("SPS30: error reading status.")); }
      } // Firmware >= 2.2

    } else {
      lastSPS30 = lastSPS30 + 100; // retry in 100ms
    } // end check if data available
  } // end time
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void serialTrigger(const char* mess, int timeout) {
  unsigned long startTime = millis();
  bool clearSerial = true;
  Serial.println(); Serial.println(mess);
  while ( !Serial.available() && (millis() - startTime < timeout) ) {
    delay(1000);
  }
  while (Serial.available()) { Serial.read(); }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////