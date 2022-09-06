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

#define ERROR_COUNT 24                                     // How often we will attempt reconnecting the sensor
#define sps30_i2cspeed             100000                  // slow speed
#define sps30_i2cClockStretchLimit 200000                  // long clock stretch
#define yieldI2C()                 delayMicroseconds(100); // block system for 0.1 ms when switching I2C port

TwoWire *sps30_port = 0;                                   // pointer to the i2c port, might be useful for other microcontrollers
uint8_t sps30_i2c[2];                                      // the pins for the i2c port, set during initialization

enum SensorStates{IS_IDLE = 0, IS_MEASURING, IS_BUSY, DATA_AVAILABLE, GET_BASELINE, IS_SLEEPING, IS_WAKINGUP, WAIT_STABLE, HAS_ERROR};
  // IS_IDLE        the sensor is powered up
  // IS_MEASURING   the sensor is creating data autonomously
  // IS_BUSY        the sensor is producing data and will not respond to commands
  // DATA_AVAILABLE new data is available in sensor registers
  // GET_BASELINE   the sensor is creating baseline
  // IS_SLEEPING    the sensor or parts of the sensot are in sleep mode
  // IS_WAKINGUP    the sensor is getting out of sleep mode
  // WAIT_STABLE    readings are not stable yet    
  // HAS_ERROR      the communication with the sensor failed

unsigned long currentTime;                                 // loop start time
unsigned long startUpdate;
unsigned long intervalSPS30;                               // measurement interval
unsigned long timeSPS30Stable;                             // time when readings are stable, is adjusted automatically based on particle counts
unsigned long lastSPS30;                                   // last time we interacted with sensor
unsigned long wakeSPS30;                                   // time when wakeup was issued
unsigned long wakeTimeSPS30;                               // time when sensor is supposed to be woken up
unsigned long timeToStableSPS30;                           // how long it takes to get stable readings, automatically pupulated based on total particles
unsigned long errorRecSPS30;                               // time when error recovery should be attempted
unsigned long sps30_lastError;                             // last time we ran into an error
unsigned long maxUpdateSPS30;                              // how long does it take to update the driver within system interval
unsigned long deltaUpdate;                                 // current update time
unsigned long AllmaxUpdateSPS30;                           // historic max update time
unsigned long lastSYS;                                     // last time we displayed update variables
unsigned long intervalSYS;                                 // how long is the window to calculate max update time

uint8_t       sps30_error_cnt = 0;                         // how many times did we attempt to recover from current error
uint8_t       sps30_timeout_cnt = 0;                       // how many times did we have to extend the waiting period
bool          sps30_avail = false;                         // sps30 has been found is still available
bool          sps30NewData = false;                        // we have new data
char          serial[SPS30_MAX_SERIAL_LEN];                // sps30 serial number string
uint16_t      data_ready;                                  // sps30 data ready register (16bit)
uint32_t      st;                                          // sps30 status register (32bit)
char          tmpStr[256];                                 // Buffer for formatting text that will be sent to USB serial or telnet
uint32_t      autoCleanIntervalSPS30;                      // seconds between autocleaning

const char waitmsg[] PROGMEM = {"Waiting 10 seconds, skip by hitting enter"};  // Allows user to open serial terminal to observe the debug output before the loop starts

SPS30 sps30;                                               // the particle sensor
struct sps30_measurement valSPS30;                         // the sensor values
volatile SensorStates stateSPS30 = IS_BUSY;                // the state of sensor

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{

  Serial.begin(115200);
  serialTrigger(waitmsg, 10000);  
  Serial.println("SPS30 Example Program");

  // search for the sensor on all pins
  uint8_t portArray[]      = {5, 4, 0, 2, 14, 12};
  char    portMap[6][3]    = {"D1", "D2", "D3", "D4", "D5", "D6"};
  uint8_t error, address;
  sps30_avail = false;

  Serial.println();
  for (uint8_t i = 0; i < sizeof(portArray); i++) {
    for (uint8_t j = 0; j < sizeof(portArray); j++) {
      if (i != j) {
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("Scanning (SDA:SCL) - %s:%s"), portMap[i], portMap[j]); 
        Serial.println(tmpStr);
        myWire.begin(portArray[i], portArray[j]);
        myWire.setClock(50000); // scan at slow speed
        myWire.setClockStretchLimit(200000);
        for (address = 1; address < 127; address++ )  {
          myWire.beginTransmission(address);
          error = myWire.endTransmission();
          if (error == 0) {
            // found a device
            if (address == 0x69) {
              sps30_avail  = true;  // Senserion Particle
              sps30_port   = &myWire;  
              sps30_i2c[0] = portArray[i];  
              sps30_i2c[1] = portArray[j];
            } else {
              snprintf_P(tmpStr, sizeof(tmpStr), PSTR("Found unknonw device - %d!"), address); 
              Serial.println(tmpStr);
            }
          }
        }
      }
    }
  }

  if (sps30_avail)  { snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30 available: %d SDA %d SCL %d"), uint32_t(sps30_port),  sps30_i2c[0],  sps30_i2c[1]);  } else { snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30 not available")); }  
  Serial.println(tmpStr);

  // SPS30 Initialize Particle Sensor  
  if (sps30_avail) { 
    if (initializeSPS30() == false) {
      Serial.println("SPS30: could not initialize.");
    }
  } 
  
  lastSPS30 = millis();
  lastSYS = millis();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  currentTime = millis();
  
  startUpdate = millis();
  if (sps30_avail) { 
    if (updateSPS30()  == false) { Serial.println("SPS30: could not update."); }
  }
  deltaUpdate = millis() - startUpdate;
  if (maxUpdateSPS30    < deltaUpdate) { maxUpdateSPS30 = deltaUpdate; }
  if (AllmaxUpdateSPS30 < deltaUpdate) { AllmaxUpdateSPS30 = deltaUpdate; }

  if (sps30NewData) {
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
    sps30NewData = false;
  }

  if ((currentTime - lastSYS) >= intervalSYS) {
    lastSYS = currentTime;
    snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: update max: %4u all time max: %4u."), maxUpdateSPS30, AllmaxUpdateSPS30);
    Serial.println(tmpStr);
    maxUpdateSPS30 = 0;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void switchI2C(TwoWire *myPort, int sdaPin, int sclPin, uint32_t i2cSpeed, uint32_t i2cStretch) {
  myPort->begin(sdaPin, sclPin);
  myPort->setClock(i2cSpeed);
  myPort->setClockStretchLimit(i2cStretch);
  yieldI2C();
}

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

bool initializeSPS30() { 

  intervalSPS30 = 4000;
  intervalSYS   = 10000;

  snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: interval: %lums."), intervalSPS30);
  Serial.println(tmpStr); 

  switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);
  sps30.begin(sps30_port);

  if ( sps30.probe() ) { 

    Serial.println(F("SPS30: detected."));
    snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: serial: %s major: %d minor: %d."), 
               sps30.serial_number(), sps30.fw_major(), sps30.fw_minor()); 
    Serial.println(tmpStr);
    snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: product type: %s."), sps30.product_type()); 
    Serial.println(tmpStr);
    snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: driver: %s."), sps30.driver_version()); 
    Serial.println(tmpStr);
  } else { 

    // reset
    if ( sps30.reset() ) { Serial.println(F("SPS30: reset.")); }
    else { Serial.println(F("SPS30: could not reset."));  }

    // probe again
    if ( sps30.probe() ) {

      Serial.println(F("SPS30: detected."));
      snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: serial: %s major: %d minor: %d."), 
                sps30.serial_number(), sps30.fw_major(), sps30.fw_minor()); 
      Serial.println(tmpStr);
      snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: product type: %s ."), sps30.product_type()); 
      Serial.println(tmpStr);
      snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: driver: %s ."), sps30.driver_version()); 
      Serial.println(tmpStr);
    } else {

      Serial.println(F("SPS30: could not probe / connect. resetting I2C."));

      // reset i2c bus
      if ( sps30.i2c_general_call_reset() ) { Serial.println(F("SPS30: reset I2C bus.")); } 
      else { Serial.println(F("SPS30: could not reset I2C bus.")); }
      switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);
      delay(2000);

      // probe again
      if ( sps30.probe() ) {

        Serial.println(F("SPS30: detected."));
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: serial: %s major: %d minor: %d."), 
                  sps30.serial_number(), sps30.fw_major(), sps30.fw_minor()); 
        Serial.println(tmpStr);
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: product type: %s ."), sps30.product_type()); 
        Serial.println(tmpStr);
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: driver: %s ."), sps30.driver_version()); 
        Serial.println(tmpStr);
      } else {
        Serial.println(F("SPS30: could not probe / connect. giving up."));
        stateSPS30 = HAS_ERROR;
        errorRecSPS30 = currentTime + 5000;
        return(false);
      }
    }
  }

  // reset
  if ( sps30.reset() ) {  Serial.println(F("SPS30: reset.")); }
  else {
    Serial.println(F("SPS30: could not reset."));
    stateSPS30 = HAS_ERROR;
    errorRecSPS30 = currentTime + 5000;
    return(false);
  }

  // get cleaning interval
  if ( sps30.get_fan_auto_cleaning_interval(&autoCleanIntervalSPS30) ) {
    snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: auto clean interval: %us."), autoCleanIntervalSPS30); 
    Serial.println(tmpStr); 
  } else { Serial.println(F("SPS30: coulnd not obtain autoclean information."));  }

  if (autoCleanIntervalSPS30 != (7 * 24 * 60 * 60)) {
    if ( sps30.set_fan_auto_cleaning_interval_days(7) ) {
      Serial.println(F("SPS30: coulnd not set autocleaning interval.")); 
    } else { Serial.println("SPS30: auto-clean interval set to 7 days."); }

  }

  // Start the device, will create internal readings every 1 sec
  if ( sps30.start_measurement() ) { 
    Serial.println(F("SPS30: measurement started."));
    stateSPS30 = IS_BUSY; 
  } else { 
    Serial.println(F("SPS30: could not start measurement."));
    stateSPS30 = HAS_ERROR;
    errorRecSPS30 = currentTime + 5000;
    return(false);
  }

  lastSPS30 = currentTime;

  delay(50);
  return(true);
} // end initialize

//////////////////////////////////////////////////////////////////////////////////////////////////////////

bool updateSPS30() {

  bool success = true;  // when ERROR recovery fails, success becomes false

  switch(stateSPS30) { 
    
    case IS_BUSY: { //--------------------- getting system going
      if ((currentTime - lastSPS30) > 1000) { // start command needs 20ms to complete but it takes 1 sec to produce data
        switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);

        // check data available
        if ( sps30.read_data_ready(&data_ready) ) { Serial.println(F("SPS30: checked for data.")); }
        else { 
          Serial.println(F("SPS30: error checking for data."));
          stateSPS30 = HAS_ERROR; 
          errorRecSPS30 = currentTime + 5000; 
          break; 
        }
        if (data_ready) { 

          // read data
          if ( sps30.read_measurement(&valSPS30) ) { 
            Serial.println(F("SPS30: data read."));

            // adjust time to get stable readings, it takes longer  with lower concentration to get a precise reading
            if (valSPS30.nc_10p0 < 100.)      { timeToStableSPS30 = 30000; } // hard coded from data sheet
            else if (valSPS30.nc_10p0 < 200.) { timeToStableSPS30 = 16000; } // hard coded from data sheet
            else                              { timeToStableSPS30 =  8000; } // hard coded from data sheet
            timeSPS30Stable = currentTime +  timeToStableSPS30;

            snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: nc PM10 %f."), valSPS30.nc_10p0); 
            Serial.println(tmpStr); 
            snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: current time: %lu."), currentTime); 
            Serial.println(tmpStr);
            snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: time when stable: %lu."), timeSPS30Stable); 
            Serial.println(tmpStr);

            sps30_error_cnt = 0;
            sps30_timeout_cnt = 0;
            lastSPS30 = currentTime; 
            stateSPS30 = WAIT_STABLE;            
            break;
          } else {
            Serial.println(F("SPS30: error reading data."));
            stateSPS30 = HAS_ERROR; 
            errorRecSPS30 = currentTime + 5000; 
            break; 
          }
        } else {
          lastSPS30 = lastSPS30 + 100; // retry in 100ms
          break; 
        }
      } // end time
      break; 
    } // is BUSY

    case WAIT_STABLE: { //--------------------- checking if system is stable, read data
      if (currentTime >= timeSPS30Stable) {
        switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);

        // check if data available
        if ( sps30.read_data_ready(&data_ready) ) { Serial.println(F("SPS30: checked for data.")); }
        else {
          Serial.println(F("SPS30: error checking for data."));
          stateSPS30 = HAS_ERROR; 
          errorRecSPS30 = currentTime + 5000; 
          break; 
        }
        if (data_ready) { 
          if ( sps30.read_measurement(&valSPS30) ) { 
            Serial.println(F("SPS30: data read."));
            sps30_error_cnt = 0;
            sps30NewData   = true;
            lastSPS30 = currentTime; 
            Serial.println(F("SPS30: going to idle."));
            stateSPS30 = IS_IDLE;          
          } else {
            Serial.println("SPS30: error reading data."); 
            stateSPS30 = HAS_ERROR; 
            errorRecSPS30 = currentTime + 5000; 
            break;
          }
        } else {
          timeSPS30Stable = timeSPS30Stable + 100; // retry in 100ms
          if (sps30_timeout_cnt++ > ERROR_COUNT) { // check if number of retries exceeded
            stateSPS30 = HAS_ERROR; 
            errorRecSPS30 = currentTime + 5000; 
            success = false; 
            Serial.println(F("SPS30: data ready timeout."));
            break;
          } // timeout count exceeded
        }

        // obtain device status
        if ( ( (sps30.fw_major()==2) && (sps30.fw_minor()>=2) ) || (sps30.fw_major()>2) ) {
          if ( sps30.read_device_status_register(&st) ) { // takes 20ms
            Serial.println(F("SPS30: read status."));
            if (st == SPS30_STATUS_OK)             { Serial.println(F("SPS30: ok.")); }
            else {
                if (st & SPS30_STATUS_SPEED_ERROR) { Serial.println(F("SPS30: warning, fan is turning too fast or too slow."));  }
                if (st & SPS30_STATUS_LASER_ERROR) { Serial.println(F("SPS30: error laser failure.")); }
                if (st & SPS30_STATUS_FAN_ERROR)   { Serial.println(F("SPS30: error fan failure, fan is mechanically blocked or broken.")); }
            } // status ok/notok
          } else                                   { Serial.println(F("SPS30: error reading status.")); } // read status
        } // Firmware >= 2.2
      } //if time stable

      break;
    } // wait stable
                
    case IS_IDLE : { //--------------------- after reading data either go to sleep or go back to read again
      if ( (intervalSPS30<60000) || (sps30.fw_major()<2) ) { // short measurement interval or no sleep function
        timeSPS30Stable = (unsigned long) (lastSPS30 + intervalSPS30); // not going to sleep, sensors remains stable
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: lastSPS30: %lu."), lastSPS30);
        Serial.println(tmpStr);
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: current time:: %lu."), currentTime);
        Serial.println(tmpStr);
        snprintf_P(tmpStr, sizeof(tmpStr), PSTR("SPS30: time when stable: %lu."), timeSPS30Stable); 
        Serial.println(tmpStr);
        Serial.println(F("SPS30: going to wait until stable."));
        stateSPS30 = WAIT_STABLE;             
      } else {
        // going to sleep until intervalSPS30, but need to wake up earlier to stabilize sensor readings
        wakeTimeSPS30 = (unsigned long) (lastSPS30 + intervalSPS30 - 50 - timeToStableSPS30);
        switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);
        if ( sps30.stop_measurement() ) { // go to idle, takes 20ms
          Serial.println(F("SPS30: stopped measurement."));
        } else {
          Serial.println(F("SPS30: error could not stop measurement."));
          stateSPS30 = HAS_ERROR;
          errorRecSPS30 = currentTime + 5000;
        }
        if ( sps30.sleep() ) { // takes 5ms
          Serial.println(F("SPS30: going to sleep."));
          stateSPS30 = IS_SLEEPING;
        } else {
          Serial.println(F("SPS30: error could not go to sleep."));
          stateSPS30 = HAS_ERROR;
          errorRecSPS30 = currentTime + 5000;
        }
      }
      break;
    }
      
    case IS_SLEEPING : { //--------------------- in slow mode we put system to sleep if new sensor version
      if (currentTime >= wakeTimeSPS30) { // Wake up if sleep time exceeded
        Serial.println(F("SPS30: waking up."));
        switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);
        if ( sps30.wake_up() ) { // takes 5ms
          wakeSPS30 = currentTime;
          stateSPS30 = IS_WAKINGUP;
        } else {
          Serial.println(F("SPS30: error could not wakeup."));
          stateSPS30 = HAS_ERROR; 
          errorRecSPS30 = currentTime + 5000;
        }
      } // time interval
      break; 
    } // case is sleeping
      
    case IS_WAKINGUP : {  // ------------------  startup the sensor
      if ( currentTime >= ( wakeSPS30 + 50) ) { // Give some time (50ms) to wake up 
        switchI2C(sps30_port, sps30_i2c[0], sps30_i2c[1], sps30_i2cspeed, sps30_i2cClockStretchLimit);
        if ( sps30.start_measurement() ) { 
          Serial.println(F("SPS30: measurement started."));
          stateSPS30 = IS_BUSY; 
        } else {
          Serial.println(F("SPS30: error could not start SPS30 measurements."));
          stateSPS30 = HAS_ERROR; 
          errorRecSPS30 = currentTime + 5000;
        }
        lastSPS30 = currentTime;
      } // end current time     
      break;
    }

    case HAS_ERROR : { //--------------------  trying to recover sensor
      if (currentTime > errorRecSPS30) {
        if (sps30_error_cnt++ > ERROR_COUNT) { // check if number of retries exceeded
          success = false; 
          sps30_avail = false; 
          Serial.println(F("SPS30: reinitialization attempts exceeded, SPS30 no longer available."));
          break;
        } // give up after ERROR_COUNT tries

        sps30_lastError = currentTime;

        if (initializeSPS30()) { // initialize, this does sensor reset and i2c bus reset if necessary
          sps30_error_cnt = 0;
          Serial.println(F("SPS30: recovered."));
        } else {
          Serial.println(F("SPS30: could not recover."));
          stateSPS30 = HAS_ERROR; 
          errorRecSPS30 = currentTime + 5000;
        }
      } // end error revocery time
      break;
    }

    default: {
      Serial.println(F("SPS30 error invalid switch statement."));
      break;
    }

  } // end cases

  return success;
} // end update