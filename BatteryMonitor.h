/*
 * Created 12/27/2015 - BatteryMonitor.h
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
 * Class to monitor the battery.
 */

#ifndef BatteryMonitor_h
#define BatteryMonitor_h

#include "Arduino.h"

#define MAXIMUM_VOLTAGE 4.1f
#define MINIMUM_VOLTAGE 3.7f

class BatteryMonitor {
  public:
    BatteryMonitor(int analogPin, double divider);
    double getVoltage();
    int getPercentage();
  private:
    int _pin;
    double _divider;
};

#endif
