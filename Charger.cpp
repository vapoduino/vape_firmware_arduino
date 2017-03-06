/*
 * Created 02/12/2016 - Charger.cpp
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

#include "Charger.h"
 
Charger::Charger(uint8_t enablePin, uint8_t statePin, uint8_t standbyPin) {
    _enablePin = enablePin;
    _statePin = statePin;
    _standbyPin = standbyPin;
  
    // Configuring pin as output
    pinMode(_enablePin, OUTPUT);
    pinMode(_statePin, INPUT);
    pinMode(_standbyPin, INPUT);
    
    digitalWrite(_enablePin, HIGH);
    digitalWrite(_statePin, HIGH);
    digitalWrite(_standbyPin, HIGH);
}

boolean Charger::isCharging(void) {
    return digitalRead(_statePin) == LOW;
}

boolean Charger::isStandby(void) {
    return (digitalRead(_standbyPin) == LOW);
}

