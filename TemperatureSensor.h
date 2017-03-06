/*
 * Created 12/27/2015 - TemperatureSensor.h
 * 
 * Vapoduino - Arduino based controller for a Vaporizer.
 * 
 * Copyright (C) 2014 Benedikt Schlagberger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Class talking to the max31865 chip and reading temperatures.
 */

#ifndef TemperatureSensor_h
#define TemperatureSensor_h

#include "Arduino.h"

/*******************************************************************************
 **                          CONFIGURATION PARAMETERS                         **
 *******************************************************************************/

#define REFERENCE_RESISTOR 400
#define PT_RESISTANCE 100

/**
 * Configuration of the MAX31865 from MSB to LSB:
 * BIT      FUNCTION            ASSIGNMENT
 *  7       VBIAS               0=OFF            1=ON
 *  6       Conversion Mode     0=Normally OFF   1=AUTO
 *  5       1-Shot              0= -             1=1-Shot
 *  4       3-Wire              0=2- or 4-Wire   1=3-wire
 * 3,2      Faultdetection      set both to 0
 *  1       Fault Status Clear  set to 1
 *  0       50/60Hz filter      0=60Hz           1=50Hz
 */
#define MAX31865_CONFIG 0b11000011

//*******************************************************************************


// Registers defined in Table 1 on page 12 of the data sheet
#define CONFIGURATION_REGISTER 0x00
#define RTD_MSB_REGISTER 0x01
#define RTD_LSB_REGISTER 0x02
#define HIGH_FAULT_THRESHOLD_MSB_REGISTER 0x03
#define HIGH_FAULT_THRESHOLD_LSB_REGISTER 0x04
#define LOW_FAULT_THRESHOLD_MSB_REGISTER 0x05
#define LOW_FAULT_THRESHOLD_LSB_REGISTER 0x06
#define FAULT_STATUS_REGISTER 0x07

#define READ_MODE 0x00
#define WRITE_MODE 0x80

// Selfclearing bits for the configuation register
#define START_1_SHOT 0x20
#define CLEAR_FAULT_STATUS 0x02

// Constants for Callendar-Van Dusen equation
#define CONST_A 0.00390830
#define CONST_B -0.0000005775
#define CONST_C -0.00000000000418301;

class TemperatureSensor {
  public:
    TemperatureSensor(uint8_t CSPin, uint8_t powerPin, uint8_t averageCount);
    boolean powerUp(void);
    void powerDown(void);
    float getTemp(void);
  private:
    byte _spiRead(byte address);
    void _spiWrite(byte address, byte value);
    boolean _sendConfig(byte configuration);
    float _calculateTemp(uint16_t rtdValue);
    void _resolveFault(byte fault);
    uint16_t _calculateAvg(uint16_t currentValue);

    uint8_t _CSPin;
    uint8_t _powerPin;
    
    // Array for averaging the reads
    uint16_t* _values;
    uint8_t _counter;
    uint8_t _averageCount;
};

#endif
