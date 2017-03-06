/*
 * Created 02/11/2015 - HeatingElement.cpp
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
#include "HeatingElement.h"

HeatingElement::HeatingElement(uint8_t mosfetPin) {
    _mosfetPin = mosfetPin;
  
    // Configuring pin as output
    pinMode(_mosfetPin, OUTPUT);
    
    // Turning heat off
    stopHeat();
}

/**
 * Sets the heat to the specified value. Can be 0 to 255.
 */
void HeatingElement::setHeat(uint8_t heat) {
    analogWrite(_mosfetPin, heat);
}

/**
 * Stops the heating.
 */
void HeatingElement::stopHeat(void) {
    digitalWrite(_mosfetPin, LOW);
}

