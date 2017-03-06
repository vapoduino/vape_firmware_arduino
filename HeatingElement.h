/*
 * Created 02/11/2015 - HeatingElement.h
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


#ifndef HeatingElement_h
#define HeatingElement_h

#include "Arduino.h"

/**
 * Class controlling the heating coil.
 */
class HeatingElement {
  public:
    HeatingElement(uint8_t mosfetPin);
    void setHeat(uint8_t heat);
    void stopHeat(void);
  private:
    uint8_t _mosfetPin;
};

#endif
