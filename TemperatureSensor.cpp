/*
 * Created 12/27/2015 - TemperatureSensor.cpp
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
 
#include "Arduino.h"
#include "TemperatureSensor.h"
#include <SPI.h>

TemperatureSensor::TemperatureSensor(uint8_t CSPin, uint8_t powerPin, uint8_t averageCount) {
    _CSPin = CSPin;
    _powerPin = powerPin;
    _averageCount = averageCount;
  
    // Configuring in- and outputs
    pinMode(_CSPin, OUTPUT);
    pinMode(_powerPin, OUTPUT);

    // Power down MAX31865
    powerDown();
    
    // Deselecting MAX31865
    digitalWrite(_CSPin, HIGH);
  
    // Configuring SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE1);
//    SPI.setClockDivider(SPI_CLOCK_DIV128);

    _values = (uint16_t*) malloc(_averageCount * sizeof(typeof(uint16_t)));
    _counter = 0;
}

boolean TemperatureSensor::powerUp(void) {
    digitalWrite(_powerPin, LOW);
    
    // Wait for MAX31865 to set up
    delay(20);
    
    // Initial configuration of MAX31865
    if (!_sendConfig(MAX31865_CONFIG)) {
        Serial.println("Communication failed!");
        Serial.println(_spiRead(CONFIGURATION_REGISTER), BIN);
        return false;
    }
    
    // fill average data
    for (int i = 0; i < _averageCount; i++) {
        getTemp();
    }
    
    // Setting up average calculation
    _counter = 0;
    
    // TODO: Set threshold for temperature reading
    
//    _spiWrite(LOW_FAULT_THRESHOLD_MSB_REGISTER, 0x00);
//    _spiWrite(LOW_FAULT_THRESHOLD_LSB_REGISTER, 0x00);
//    _spiWrite(HIGH_FAULT_THRESHOLD_MSB_REGISTER, 0xFF);
//    _spiWrite(HIGH_FAULT_THRESHOLD_LSB_REGISTER, 0xFF);
    return true;
}

void TemperatureSensor::powerDown(void) {
    digitalWrite(_powerPin, HIGH);
}

/**
 * Reads the rtd value from the MAX31865 chip and calculates the temperature by using
 * the Callendar-Van Dusen equation for approximation.
 */
float TemperatureSensor::getTemp(void) {
    // read value
    byte lsb = _spiRead(RTD_LSB_REGISTER);
    uint16_t rtd_value = _spiRead(RTD_MSB_REGISTER) << 7;
    
    // shift out the fault bit
    rtd_value += (lsb >> 1);
    
    if ((lsb & 1)) {
        Serial.print("\nFault Detected! RTD value: ");
        Serial.println(rtd_value);
        delay(100);
        byte fault = _spiRead(FAULT_STATUS_REGISTER);
        
        _resolveFault(fault);
        
        // return NaN
        return rtd_value;
    } else {
        // check if data is valid
        if (rtd_value == 0) {
            Serial.println("MAX31865 is stuck or not responding. Trying to reconfigure.");
            if (!_sendConfig(MAX31865_CONFIG)) {
                Serial.println("Communication failed!");
                Serial.println(_spiRead(CONFIGURATION_REGISTER), BIN);
                while(true);
            }
            // return NaN
            return 0.0 / 0.0;
        }
        
        if (_averageCount > 1) {
            rtd_value = _calculateAvg(rtd_value);
        }
        
        return _calculateTemp(rtd_value);
    }
}

/**
 * Sends the given value to the given registeraddress via SPI.
 */
void TemperatureSensor::_spiWrite(byte address, byte value) {
    digitalWrite(_CSPin, LOW);
    SPI.transfer(WRITE_MODE + address);
    SPI.transfer(value);
    digitalWrite(_CSPin, HIGH);
}

/**
 * Reads from the given registeraddress via SPI and returns
 * the value.
 */
byte TemperatureSensor::_spiRead(byte address) {
    digitalWrite(_CSPin, LOW);
    SPI.transfer(READ_MODE + address);
    byte value = SPI.transfer(0x00);
    digitalWrite(_CSPin, HIGH);
    digitalWrite(8, LOW);
    return value;
}

/** 
 * Sends the given configuration to the MAX31865.
 */
boolean TemperatureSensor::_sendConfig(byte configuration) {
    _spiWrite(CONFIGURATION_REGISTER, configuration);
    
    byte returned_value = _spiRead(CONFIGURATION_REGISTER);
    
    // exclude self clearing bit (fault status clear)
    if ((returned_value & 0b11111101) == configuration) {
      return true;
    }
}

/**
 * Calculates the temperature from the given rtd value by using the 
 * Callendar-Van Dusen equation for approximation.
 */ 
float TemperatureSensor::_calculateTemp(uint16_t rtdValue) {    
    // calculate recistance of RTD
    float resistance = (((float) rtdValue) * ((float) REFERENCE_RESISTOR)) / 32768;
    
    // Use Callendar-Van Dusen equation for approximation
    float temp = -PT_RESISTANCE * CONST_A
            + sqrt(PT_RESISTANCE * PT_RESISTANCE * CONST_A * CONST_A - 4 * PT_RESISTANCE * CONST_B * (PT_RESISTANCE - resistance));
    temp /= (2 * PT_RESISTANCE * CONST_B);

    return temp;
}

/**
 * Prints information for the given fault status register value to Serial and resets
 * the configuration register to the initial value.
 */
void TemperatureSensor::_resolveFault(byte fault) {
    byte temp = (fault & 0x80);
    if (temp > 0) {
        Serial.print("High Threshold reached. RTD is possibly disconnected from RTD+ or RTD-.\nThreshold is set to ");
        int threshold = _spiRead(HIGH_FAULT_THRESHOLD_MSB_REGISTER) << 7;
        threshold += _spiRead(HIGH_FAULT_THRESHOLD_LSB_REGISTER) >> 1;
        Serial.println(threshold);
    }
    temp = fault & 0x40;
    if (temp > 0) {
        Serial.print("Low Threshold reached. RTD+ and RTD- is possibly shorted.\nThreshold is set to ");
        int threshold = _spiRead(LOW_FAULT_THRESHOLD_MSB_REGISTER) << 7;
        threshold += _spiRead(LOW_FAULT_THRESHOLD_LSB_REGISTER) >> 1;
        Serial.println(threshold);
    }
    temp = fault & 0x20;
    if (temp > 0) {
        Serial.println("REFIN- > 0.85 * Vbias.");
    }
    temp = fault & 0x10;
    if (temp > 0) {
        Serial.println("REFIN- < 0.85 * Vbias. FORCE- is possibly open.");
    }
    temp = fault & 0x08;
    if (temp > 0) {
        Serial.println("RTDIN- < 0.85 x Vbias. FORCE- is possibly open.");
    }
    temp = fault & 0x04;
    if (temp > 0) {
        Serial.println("Overvoltage/undervoltage fault");
    }
    
    // rewriting config and clearing fault bit
    Serial.println("Resetting");
    _sendConfig(MAX31865_CONFIG);
}

/**
 * Uses an array to store the last AVG_VALUES_COUNT values and return
 * the mean result of these values.
 */
uint16_t TemperatureSensor::_calculateAvg(uint16_t currentValue) {
  if (_counter == _averageCount) {
    _counter = 0;
  }
    _values[_counter] = currentValue;
    _counter++;
    
    long rtdSum = 0;    
    for (uint8_t i = 0; i < _averageCount; i++) {
        rtdSum += _values[i];
    }
    
    return uint16_t (rtdSum / _averageCount);
}
