Arduino Library for the SPS30 Particle Sensor 
===========================================================

![Sensirion](https://cdn.sparkfun.com//assets/parts/1/3/4/6/3/15103-Particulate_Matter_Sensor_-_SPS30-01.jpg)

[*Sensirion Particualte Matter Sensor - SPS30*](https://www.sparkfun.com/products/15103)

The Sensirion Particulate Matter Sensor SPS30 is a compact, high quality, optical particle sensor that uses laser scattering and Sensirion's innovative contamination resistance technology to achieve superior binning and particle measurement. This sensor allows users to measure mass concentration and number of particles of 1 µg/m^3, 2.5 µg/m^3, 4 µg/m^3, and 10 µg/m^3.

To learn more about the SPS30, please visit https://www.sensirion.com/sps30/.
For support questions on the SPS30, please visit https://sensirion.com/contact.

Library arranged by Urs Utzinger ([Urs](https://github.com/uutzinger)).

**Important Notes:** 

* The SPS30 requires 5V supply voltage +/-0.5V in order to provide correct
  output values; the SPS30 should be compatible with 3.3V I2C levels, so as 
  long as the supply voltage is correct the SPS30 should work fine with 3.3V 
  setups
* Make sure that the SPS30's Pin 4 ("Interface select") is connected to GND, on
  power-up of the sensor, otherwise the sensor works in UART instead of I2C
  mode and this driver will not detect the sensor. Note that the
  interface-select configuration is read on every start of the sensor including
  after a soft-reset.

Original Code
-------------

This library is based on https://github.com/Sensirion/arduino-sps which is using the code from Sensirion's 
[embedded-sps](https://github.com/Sensirion/embedded-sps) library.

Compatibility
------------

This library has been tested on the following platforms:
- ESP8266

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.

License Information
-------------------

Copyright (c) 2018, Sensirion AG
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of Sensirion AG nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

